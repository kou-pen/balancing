#include "balancing/control/estimator/balancing_observation.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * モジュール内の値はSI単位だが、C620のロータ角だけは0..8191の生カウント。
 * ここで位置換算に必要な幾何を固定しないこと。実車輪半径・減速比・符号は
 * 外部で検証したBWheelObservationConfigにのみ保持する。
 */
#define B_OBS_ENCODER_COUNTS 8192
#define B_OBS_HALF_TURN 4096
#define B_OBS_TAU 6.2831853071795864769

/*
 * スナップショットの受信鮮度は「最後に新規と確定した測定」の時刻で判定する。
 * 受付時刻now_usをstamp_usに上書きすると、同じセンサ値の再読出しが続いても
 * 古い測定が永久に新しい扱いになるため、同時刻・逆行・未来時刻を拒否する。
 * 初回サンプルはold.valid=falseなら時刻0も許可する。
 */
static bool valid_stamp(BSampleStamp old, uint64_t sample_us, uint64_t now_us)
{
    return sample_us <= now_us && (!old.valid || sample_us > old.stamp_us);
}

/* NaN/Infが一軸でも混入するとフィルタやLQRへ伝播するため全6成分を検査する。
 * 物理的な範囲、取付姿勢、キャリブレーションの正しさはこの関数では保証しない。 */
static bool valid_imu(const BImuSample *sample)
{
    if (sample == NULL) {
        return false;
    }
    for (unsigned axis = 0u; axis < 3u; ++axis) {
        if (!isfinite(sample->accel_m_s2[axis]) ||
            !isfinite(sample->gyro_rad_s[axis])) {
            return false;
        }
    }
    return true;
}

/*
 * ここでverifiedをチェックしても実機の校正を証明するわけではない。
 * 左右の軸方向は±1のみとし、0や倍率を誤って混ぜない。
 * 物理量への換算を実施する前に、半径・ロータ/車輪比が正の有限値か検査する。
 */
static bool valid_wheel_config(const BWheelObservationConfig *config)
{
    return config != NULL && config->verified &&
           isfinite(config->wheel_radius_m) &&
           isfinite(config->motor_revs_per_wheel_rev) &&
           (config->direction == 1.0f || config->direction == -1.0f) &&
           config->wheel_radius_m > 0.0f &&
           config->motor_revs_per_wheel_rev > 0.0f;
}

/* 初期化で全測定をinvalidにする。前回起動・設定の観測値を再使用しない。 */
void b_observation_init(BObservationStore *store)
{
    if (store != NULL) {
        memset(store, 0, sizeof(*store));
    }
}

bool b_observation_imu(BObservationStore *store, const BImuSample *value,
                       bool new_measurement_proven, uint64_t stamp_us,
                       uint64_t now_us)
{
    /*
     * I2Cトランザクション成功≠センサ新規更新。
     * BNO055で新規性をどう証明するかは実機側に委ね、証拠がなければ
     * 前回の値もタイムスタンプも更新しない（STALEを隠さない）。
     */
    if (store == NULL || !new_measurement_proven || !valid_imu(value) ||
        !valid_stamp(store->stamps.imu, stamp_us, now_us)) {
        return false;
    }
    /* 単一所有者の処理内で値→測定時刻の順に更新。
     * 割込みとの並列書込み・読出しは未対応。呼出元で直列化する。 */
    store->imu = *value;
    store->stamps.imu = (BSampleStamp){true, stamp_us};
    return true;
}

bool b_observation_wheel(BObservationStore *store, unsigned wheel_index,
                         const BWheelObservationConfig *config,
                         uint16_t encoder_raw, int16_t rpm,
                         uint64_t stamp_us, uint64_t now_us)
{
    /*
     * エンコーダ値と車輪RPMはC620フレーム復号後の測定値を想定する。
     * 同一stampの再投入、時計の逆行、未来stampを拒否し、誤ったサンプルで
     * 位置や鮮度を進めない。検証済み設定なしでは位置を積算しない。
     */
    if (store == NULL || wheel_index >= 2u || !valid_wheel_config(config) ||
        encoder_raw >= B_OBS_ENCODER_COUNTS ||
        !valid_stamp(store->stamps.wheel[wheel_index], stamp_us, now_us)) {
        return false;
    }

    /*
     * C620はロータ1回転内の角度だけを返すため、受信間に何周したかは復元不能。
     * 「前回との差は半回転未満」という前提の下でのみ最短差分を選択する。
     * 通信間隔と実最大速度がこの前提を満たすかは実機で別途確認する必要がある。
     */
    const BSampleStamp old_stamp = store->stamps.wheel[wheel_index];
    const BWheelObservation old = store->wheel[wheel_index];
    if (old_stamp.valid) {
        /*
         * 積算開始後に半径・ギヤ比・符号を変更すると、同じカウント差でも
         * 前回までの移動距離と尺度が変わる。暗黙の再換算はせず拒否し、
         * 新しい設定を使うときは観測ストアの明示的な初期化を要求する。
         */
        const BWheelObservationConfig *saved = &store->wheel_config[wheel_index];
        if (saved->wheel_radius_m != config->wheel_radius_m ||
            saved->motor_revs_per_wheel_rev != config->motor_revs_per_wheel_rev ||
            saved->direction != config->direction) {
            return false;
        }
    }
    /*
     * 初回サンプルでは絶対車輪位置が分からないため積算起点を0mにする。
     * 継続サンプルでは[-4095,+4095]の最短ロータ差分に折り返す。
     * 差がちょうど±4096のときは正逆が区別できず、曖昧なまま積算しない。
     */
    int counts = 0;
    if (old_stamp.valid) {
        counts = (int)encoder_raw - (int)old.encoder_raw;
        if (counts > B_OBS_HALF_TURN) {
            counts -= B_OBS_ENCODER_COUNTS;
        } else if (counts < -B_OBS_HALF_TURN) {
            counts += B_OBS_ENCODER_COUNTS;
        }
        if (counts == B_OBS_HALF_TURN || counts == -B_OBS_HALF_TURN) {
            return false; /* 半回転は正転・逆転の双方に解がある。 */
        }
    }

    /*
     * 1ロータ回転当たりの車輪移動量[m] =
     * 2π * 車輪半径 / (ロータ回転数/車輪回転数) * 方向符号。
     * RPMは1分当たりのロータ回転数なので、周速度[m/s]は60で割る。
     * 途中計算にdoubleを用いて蓄積誤差・桁あふれを確認した後floatへ格納。
     */
    const double meters_per_rotor_turn =
        B_OBS_TAU * (double)config->wheel_radius_m /
        (double)config->motor_revs_per_wheel_rev * (double)config->direction;
    const double position =
        (double)old.position_m + meters_per_rotor_turn * (double)counts /
        (double)B_OBS_ENCODER_COUNTS;
    const double velocity = meters_per_rotor_turn * (double)rpm / 60.0;
    /* floatへの狭め込み時にもInfを生まないよう、格納前に範囲を検査する。
     * 失敗時はstoreの値・時刻・設定のいずれも変更しない。 */
    if (!isfinite(meters_per_rotor_turn) ||
        !isfinite(position) || !isfinite(velocity) ||
        fabs(position) > FLT_MAX || fabs(velocity) > FLT_MAX) {
        return false;
    }

    BWheelObservation updated = {
        .encoder_raw = encoder_raw,
        .rpm = rpm,
        .position_m = (float)position,
        .velocity_m_s = (float)velocity
    };
    /* 正当性をすべて確認できた場合だけ、値・使用設定・新規時刻を確定する。
     * configを保存する理由は次回呼出しの無断設定変更を検出するため。 */
    store->wheel[wheel_index] = updated;
    store->wheel_config[wheel_index] = *config;
    store->stamps.wheel[wheel_index] = (BSampleStamp){true, stamp_us};
    return true;
}

bool b_observation_freeze(const BObservationStore *store, uint64_t now_us,
                          BObservationSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    /*
     * 失敗したコピーを成功済みのスナップショットと取り違えないよう先にゼロ化。
     * コピー時刻now_usは測定時刻としては使わない。
     */
    memset(snapshot, 0, sizeof(*snapshot));
    if (store == NULL) {
        return false;
    }
    /*
     * 未来時刻が混じると異なる時計・競合読取り・不正更新の疑いがある。
     * 一部だけのコピーはしない。各測定の鮮度期限はここで判定せず
     * b_check_freshness()の責務に残す（元の測定時刻を保持する）。
     */
    if ((store->stamps.imu.valid && store->stamps.imu.stamp_us > now_us) ||
        (store->stamps.power.valid && store->stamps.power.stamp_us > now_us)) {
        return false;
    }
    for (unsigned i = 0u; i < 4u; ++i) {
        if (store->stamps.rs[i].valid && store->stamps.rs[i].stamp_us > now_us) {
            return false;
        }
    }
    for (unsigned i = 0u; i < 2u; ++i) {
        if (store->stamps.wheel[i].valid && store->stamps.wheel[i].stamp_us > now_us) {
            return false;
        }
    }

    /*
     * 単一所有者の制御ループ内で一貫した1周期分を作る。
     * このmemcpy相当のコピー自体はIRQと競合する場合の原子性を保証しない。
     * 受信イベントを直列化した状態でfreezeを呼ぶことが統合側の前提。
     */
    snapshot->stamps = store->stamps;
    snapshot->imu = store->imu;
    for (unsigned i = 0u; i < 2u; ++i) {
        snapshot->wheel[i] = store->wheel[i];
    }
    snapshot->frozen_at_us = now_us;
    return true;
}
