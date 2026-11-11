#include "balancing/control/filter/balancing_attitude_filter.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define B_ATTITUDE_TAU 6.28318530717958647692f

/*
 * 数学fixtureで使用する機体座標はX前、Y左、Z上。
 * この符号とBNO055の物理取付が一致するかはまだ測定していない。
 * 取付マップの軸並べ替え・符号反転は外部で検証した設定でのみ行う。
 * Yaw積分角を加速度から推定することは原理的に不可能なので扱わない。
 */
static bool valid_mount(const BImuMount *map)
{
    if (map == NULL || !map->verified) {
        return false;
    }
    unsigned mask = 0u;
    int sign_product = 1;
    unsigned inversions = 0u;
    for (unsigned i = 0u; i < 3u; ++i) {
        if (map->sensor_index[i] >= 3u ||
            (map->direction[i] != 1 && map->direction[i] != -1)) {
            return false;
        }
        const unsigned bit = 1u << map->sensor_index[i];
        if ((mask & bit) != 0u) {
            return false; /* 同じセンサ軸を複数機体軸へ割当てない。 */
        }
        mask |= bit;
        sign_product *= (int)map->direction[i];
        for (unsigned j = 0u; j < i; ++j) {
            inversions += (unsigned)(map->sensor_index[j] > map->sensor_index[i]);
        }
    }
    /*
     * 回転行列の行列式は+1である必要がある。軸入替の偶奇と
     * 各軸符号の積から判定し、鏡像変換（det=-1）を拒否する。
     * 実際の取付角が90度刻みでない場合はこの単純モデルでは扱えない。
     */
    return mask == 7u && (inversions % 2u == 0u ? 1 : -1) * sign_product == 1;
}

static bool valid_config(const BAttitudeFilterConfig *c)
{
    return c != NULL && c->verified && valid_mount(&c->mount) &&
           c->bias_samples_required >= 2u &&
           isfinite(c->gravity_min_m_s2) && c->gravity_min_m_s2 > 0.0f &&
           isfinite(c->gravity_max_m_s2) &&
           c->gravity_max_m_s2 > c->gravity_min_m_s2 &&
           isfinite(c->gyro_alpha) &&
           c->gyro_alpha >= 0.0f && c->gyro_alpha < 1.0f &&
           isfinite(c->max_sample_dt_s) && c->max_sample_dt_s > 0.0f;
}

static bool mapped_sample(const BImuMount *map, const BImuSample *sample,
                          float accel[3], float gyro[3])
{
    if (sample == NULL) {
        return false;
    }
    /* 値を置換し終えてから後段へ渡す。NaNを一軸でも通すと
     * atan2や積分へ連鎖するため、6つすべての入力を事前確認する。 */
    for (unsigned i = 0u; i < 3u; ++i) {
        if (!isfinite(sample->accel_m_s2[i]) ||
            !isfinite(sample->gyro_rad_s[i])) {
            return false;
        }
    }
    for (unsigned i = 0u; i < 3u; ++i) {
        const unsigned source = map->sensor_index[i];
        const float sign = (float)map->direction[i];
        accel[i] = sign * sample->accel_m_s2[source];
        gyro[i] = sign * sample->gyro_rad_s[source];
    }
    return true;
}

static bool gravity_usable(const BAttitudeFilterConfig *cfg, const float a[3])
{
    /*
     * 重力方向でPitch/Rollを補正できるのは、加速度が主に重力を表す時。
     * ノルムが設定範囲外なら静止角としては信頼せず、校正時は拒否、
     * 動作中はジャイロのみを使う。これは静止証明そのものではない。
     */
    const float magnitude = hypotf(hypotf(a[0], a[1]), a[2]);
    return isfinite(magnitude) &&
           magnitude >= cfg->gravity_min_m_s2 &&
           magnitude <= cfg->gravity_max_m_s2;
}

static float gravity_pitch(const float a[3])
{
    /* X前・Y左・Z上の数学fixture:
     * pitch = atan2(-aX, sqrt(aY^2+aZ^2))。取付符号は外部照合が必要。 */
    return atan2f(-a[0], hypotf(a[1], a[2]));
}

static float gravity_roll(const float a[3])
{
    /* roll = atan2(aY,aZ)。逆さ姿勢でも±π境界での角度差を扱う。 */
    return atan2f(a[1], a[2]);
}

static float wrap_angle(float angle)
{
    /* -π..πへ正規化。測定角度の差をそのまま平均すると
     * π境界の両側が0へ混ざるため、角度差を最短方向で扱う。 */
    return remainderf(angle, B_ATTITUDE_TAU);
}

bool b_attitude_init(BAttitudeFilter *filter, const BAttitudeFilterConfig *config)
{
    if (filter == NULL) {
        return false;
    }
    /* 不正な再初期化要求でも旧バイアスを保持して推定を継続しない。 */
    memset(filter, 0, sizeof(*filter));
    if (!valid_config(config)) {
        return false;
    }
    filter->config = *config;
    filter->initialized = true;
    return true;
}

bool b_attitude_add_static_sample(BAttitudeFilter *filter, const BImuSample *sample,
                                  bool stationary_proven, bool new_measurement_proven,
                                  uint64_t stamp_us, uint64_t now_us)
{
    /*
     * 単に加速度ノルムが重力相当でも、機体の等速回転や並進は識別不能。
     * 静止が外部で確認されたサンプルだけをバイアスの平均に使う。
     * 同じセンサ値を再度読んだだけならカウントを増やさない。
     */
    if (filter == NULL || !filter->initialized || filter->bias_ready ||
        !stationary_proven || !new_measurement_proven ||
        stamp_us > now_us ||
        (filter->has_last_calibration_stamp &&
         stamp_us <= filter->last_calibration_stamp_us)) {
        return false;
    }
    float a[3];
    float gyro[3];
    if (!mapped_sample(&filter->config.mount, sample, a, gyro) ||
        !gravity_usable(&filter->config, a)) {
        return false;
    }

    double next_sum[3];
    for (unsigned i = 0u; i < 3u; ++i) {
        next_sum[i] = filter->gyro_sum_rad_s[i] + (double)gyro[i];
        if (!isfinite(next_sum[i])) {
            return false;
        }
    }
    const uint32_t next_count = filter->bias_samples_collected + 1u;
    if (next_count == filter->config.bias_samples_required) {
        /* 平均静止値を以後の角速度から差し引く。
         * 収束や熱ドリフトの妥当性は実機で別途検証が必要。 */
        float bias[3];
        for (unsigned i = 0u; i < 3u; ++i) {
            const double average = next_sum[i] / (double)next_count;
            if (!isfinite(average) || fabs(average) > FLT_MAX) {
                return false;
            }
            bias[i] = (float)average;
        }
        for (unsigned i = 0u; i < 3u; ++i) {
            filter->gyro_bias_rad_s[i] = bias[i];
        }
        filter->bias_ready = true;
    }
    for (unsigned i = 0u; i < 3u; ++i) {
        filter->gyro_sum_rad_s[i] = next_sum[i];
    }
    filter->bias_samples_collected = next_count;
    filter->last_calibration_stamp_us = stamp_us;
    filter->has_last_calibration_stamp = true;
    return true;
}

bool b_attitude_update(BAttitudeFilter *filter, const BImuSample *sample,
                       bool new_measurement_proven, uint64_t stamp_us,
                       uint64_t now_us, BAttitudeEstimate *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (filter == NULL || !filter->initialized || !filter->bias_ready ||
        !new_measurement_proven || stamp_us > now_us ||
        (filter->latest.valid && stamp_us <= filter->latest.stamp_us) ||
        (!filter->latest.valid && filter->has_last_calibration_stamp &&
         stamp_us <= filter->last_calibration_stamp_us)) {
        return false;
    }
    float a[3];
    float gyro[3];
    if (!mapped_sample(&filter->config.mount, sample, a, gyro)) {
        return false;
    }
    float corrected[3];
    for (unsigned i = 0u; i < 3u; ++i) {
        corrected[i] = gyro[i] - filter->gyro_bias_rad_s[i];
        if (!isfinite(corrected[i])) {
            return false;
        }
    }

    const bool accel_ok = gravity_usable(&filter->config, a);
    const float acc_pitch = accel_ok ? gravity_pitch(a) : 0.0f;
    const float acc_roll = accel_ok ? gravity_roll(a) : 0.0f;
    BAttitudeEstimate next = {0};
    next.valid = true;
    next.accel_fused = accel_ok;
    next.stamp_us = stamp_us;
    /*
     * ここに格納する値はセンサ軸を機体軸へ回したbody angular rate。
     * Euler角微分は一般にRoll/Pitch/Yawの姿勢に依存するため、
     * body X=Roll微分、body Y=Pitch微分、body Z=Yaw微分が成立するのは
     * 小角・結合無視の近似領域に限る。実機で大姿勢へ適用してはいけない。
     */
    next.roll_rate_rad_s = corrected[0];
    next.pitch_rate_rad_s = corrected[1];
    next.yaw_rate_rad_s = corrected[2];

    if (!filter->latest.valid) {
        /* 最初の絶対Pitch/Rollは重力方向が使える場合だけ確定。
         * 静止校正済みでも重力範囲外のサンプルでは初期姿勢を捏造しない。 */
        if (!accel_ok) {
            return false;
        }
        next.pitch_rad = acc_pitch;
        next.roll_rad = acc_roll;
    } else {
        /*
         * サンプル時刻の差を積分時間とし、500 Hzの制御呼出し頻度を
         * IMUの新規測定周期として代用しない。長い欠落があるときは
         * 不確実な予測で姿勢をつながず失敗し、明示的な再初期化を必要とする。
         */
        const uint64_t delta_us = stamp_us - filter->latest.stamp_us;
        const double dt_s = (double)delta_us * 0.000001;
        if (!(dt_s > 0.0) || dt_s > (double)filter->config.max_sample_dt_s) {
            return false;
        }
        const float predicted_pitch =
            wrap_angle(filter->latest.pitch_rad +
                       next.pitch_rate_rad_s * (float)dt_s);
        const float predicted_roll =
            wrap_angle(filter->latest.roll_rad +
                       next.roll_rate_rad_s * (float)dt_s);
        if (!isfinite(predicted_pitch) || !isfinite(predicted_roll)) {
            return false;
        }
        if (accel_ok) {
            /*
             * 相補フィルタを最短角度差で補正：
             * angle = predict + (1-alpha) * wrap(acc_angle - predict)。
             * 重力ノルムが範囲外のときは補正を止め、ジャイロのみ積分する。
             */
            const float correction = 1.0f - filter->config.gyro_alpha;
            next.pitch_rad = wrap_angle(
                predicted_pitch + correction *
                wrap_angle(acc_pitch - predicted_pitch));
            next.roll_rad = wrap_angle(
                predicted_roll + correction *
                wrap_angle(acc_roll - predicted_roll));
        } else {
            next.pitch_rad = predicted_pitch;
            next.roll_rad = predicted_roll;
        }
    }

    if (!isfinite(next.pitch_rad) || !isfinite(next.roll_rad)) {
        return false;
    }
    /* 全ての演算が成功した場合だけ内部状態と元の測定時刻を確定する。
     * 失敗時は旧推定値が残るが、出力outはゼロで呼出側が失敗を判断できる。 */
    filter->latest = next;
    *out = next;
    return true;
}
