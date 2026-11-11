#include "balancing/control/filter/balancing_attitude_filter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_GRAVITY 9.81f

/*
 * 以下の軸・係数・バイアスは数値テスト用であり、BNO055の実取付方向や
 * フィルタ係数を決定しない。センサ新規性もテスト入力で明示的に模擬する。
 */
static BAttitudeFilterConfig fixture_config(void)
{
    const BAttitudeFilterConfig cfg = {
        .mount = {{0u, 1u, 2u}, {1, 1, 1}, true},
        .bias_samples_required = 4u,
        .gravity_min_m_s2 = 8.0f,
        .gravity_max_m_s2 = 11.0f,
        .gyro_alpha = 0.9f,
        .max_sample_dt_s = 0.01f,
        .verified = true
    };
    return cfg;
}

static BImuSample level_with_bias(void)
{
    const BImuSample x = {
        .accel_m_s2 = {0.0f, 0.0f, TEST_GRAVITY},
        .gyro_rad_s = {0.1f, -0.2f, 0.05f}
    };
    return x;
}

static void close_to(float got, float expected, float tol)
{
    if (!(isfinite(got) && fabsf(got - expected) <= tol)) {
        fprintf(stderr, "got=%g expected=%g tolerance=%g\n",
                (double)got, (double)expected, (double)tol);
        assert(false);
    }
}

static void calibrated_filter(BAttitudeFilter *filter)
{
    const BAttitudeFilterConfig cfg = fixture_config();
    const BImuSample sample = level_with_bias();
    assert(b_attitude_init(filter, &cfg));
    for (unsigned i = 1; i <= 4; ++i) {
        const uint64_t stamp = (uint64_t)i * 1000u;
        assert(b_attitude_add_static_sample(filter, &sample, true, true,
                                            stamp, stamp));
    }
    assert(filter->bias_ready);
    assert(filter->bias_samples_collected == 4u);
    close_to(filter->gyro_bias_rad_s[0], 0.1f, 1e-6f);
    close_to(filter->gyro_bias_rad_s[1], -0.2f, 1e-6f);
    close_to(filter->gyro_bias_rad_s[2], 0.05f, 1e-6f);
}

/* 静止確認のない加速度ノルムだけからバイアスを学習してはいけない。
 * バイアス確定前に角度・Yaw角速度を取り出そうとしても拒否する。 */
static void test_initial_gates_and_static_provenance(void)
{
    BAttitudeFilter filter = {0};
    const BAttitudeFilterConfig cfg = fixture_config();
    const BImuSample good = level_with_bias();
    BAttitudeEstimate out = {0};
    assert(b_attitude_init(&filter, &cfg));
    assert(!b_attitude_update(&filter, &good, true, 1000u, 1000u, &out));
    assert(!out.valid);
    assert(!b_attitude_add_static_sample(&filter, &good, false, true,
                                         1000u, 1000u));
    assert(!b_attitude_add_static_sample(&filter, &good, true, false,
                                         1000u, 1000u));
    assert(!b_attitude_add_static_sample(&filter, &good, true, true,
                                         2000u, 1000u));
    BImuSample invalid = good;
    invalid.gyro_rad_s[1] = NAN;
    assert(!b_attitude_add_static_sample(&filter, &invalid, true, true,
                                         1000u, 1000u));
    invalid = good;
    invalid.accel_m_s2[2] = 20.0f;
    assert(!b_attitude_add_static_sample(&filter, &invalid, true, true,
                                         1000u, 1000u));
    assert(filter.bias_samples_collected == 0u);
    assert(b_attitude_add_static_sample(&filter, &good, true, true,
                                        1000u, 1000u));
    assert(!b_attitude_add_static_sample(&filter, &good, true, true,
                                         1000u, 2000u));
    assert(filter.bias_samples_collected == 1u);
    assert(!filter.bias_ready);
}

/* 同一サンプルを繰り返し使用すると積分dtを捏造することになる。
 * 実新規時刻の差でだけ角速度を積分し、静止ジャイロの平均値を差し引く。 */
static void test_complementary_math_and_gyro_only(void)
{
    BAttitudeFilter filter;
    calibrated_filter(&filter);
    const BImuSample level = level_with_bias();
    BAttitudeEstimate out = {0};
    assert(!b_attitude_update(&filter, &level, true, 4000u, 4000u, &out));
    assert(b_attitude_update(&filter, &level, true, 5000u, 5000u, &out));
    assert(out.valid && out.accel_fused && out.stamp_us == 5000u);
    close_to(out.pitch_rad, 0.0f, 1e-6f);
    close_to(out.roll_rad, 0.0f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.0f, 1e-6f);

    const float pitch = 0.1f;
    const float roll = 0.05f;
    BImuSample tilting = level;
    tilting.accel_m_s2[0] = -TEST_GRAVITY * sinf(pitch);
    tilting.accel_m_s2[1] = TEST_GRAVITY * cosf(pitch) * sinf(roll);
    tilting.accel_m_s2[2] = TEST_GRAVITY * cosf(pitch) * cosf(roll);
    tilting.gyro_rad_s[1] += 0.5f;
    tilting.gyro_rad_s[2] += 0.2f;
    assert(b_attitude_update(&filter, &tilting, true, 7000u, 7000u, &out));
    /* 2 ms × 0.5 rad/s = 0.001radのジャイロ予測に重力角の10%を足す。 */
    close_to(out.pitch_rad, 0.001f + 0.1f * (pitch - 0.001f), 2e-5f);
    close_to(out.roll_rad, 0.005f, 2e-5f);
    close_to(out.pitch_rate_rad_s, 0.5f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.2f, 1e-6f);

    /* 加速度が動的に20 m/s²になったときは、偽の重力傾斜で補正しない。
     * ただしジャイロの新規サンプルとして積分は継続する。 */
    BImuSample disturbed = tilting;
    disturbed.accel_m_s2[0] = 0.0f;
    disturbed.accel_m_s2[1] = 0.0f;
    disturbed.accel_m_s2[2] = 20.0f;
    const float before = out.pitch_rad;
    assert(b_attitude_update(&filter, &disturbed, true, 9000u, 9000u, &out));
    assert(!out.accel_fused && out.stamp_us == 9000u);
    close_to(out.pitch_rad, before + 0.001f, 2e-5f);

    BImuSample invalid = disturbed;
    invalid.gyro_rad_s[0] = INFINITY;
    assert(!b_attitude_update(&filter, &invalid, true, 10000u, 10000u, &out));
    assert(!out.valid && filter.latest.stamp_us == 9000u);
    assert(!b_attitude_update(&filter, &tilting, false, 10000u, 10000u, &out));
    assert(!b_attitude_update(&filter, &tilting, true, 9000u, 10000u, &out));
    assert(!b_attitude_update(&filter, &tilting, true, 11000u, 10000u, &out));
    assert(!b_attitude_update(&filter, &tilting, true, 30000u, 30000u, &out));
    assert(filter.latest.stamp_us == 9000u); /* 時刻欠落を勝手に補間しない */
}

/* ±π近辺で素朴な線形平均を使うと約0radへ誤って飛ぶ。
 * 補正差を最短角度で取るため、境界の両側でも連続していることを検証。 */
static void test_roll_wraparound(void)
{
    BAttitudeFilter filter;
    calibrated_filter(&filter);
    const BImuSample bias = level_with_bias();
    BAttitudeEstimate out = {0};

    const float near_pi = 179.0f * TEST_PI / 180.0f;
    BImuSample sample = bias;
    sample.accel_m_s2[1] = TEST_GRAVITY * sinf(near_pi);
    sample.accel_m_s2[2] = TEST_GRAVITY * cosf(near_pi);
    assert(b_attitude_update(&filter, &sample, true, 5000u, 5000u, &out));
    assert(out.roll_rad > 3.0f);

    sample.accel_m_s2[1] = -TEST_GRAVITY * sinf(near_pi);
    sample.accel_m_s2[2] = TEST_GRAVITY * cosf(near_pi);
    assert(b_attitude_update(&filter, &sample, true, 7000u, 7000u, &out));
    assert(fabsf(out.roll_rad) > 3.0f); /* 0radへジャンプしない */
}

/* 取付座標は任意の反転ではなく右手系の剛体回転が必要。
 * 軸重複と鏡像を拒否し、置換と符号で別の正しい取付方位を表現する。 */
static void test_mount_validation_and_bias_mapping(void)
{
    BAttitudeFilter filter = {0};
    BAttitudeFilterConfig cfg = fixture_config();
    cfg.mount.sensor_index[1] = 0u;
    assert(!b_attitude_init(&filter, &cfg)); /* 重複 */
    cfg = fixture_config();
    cfg.mount.sensor_index[0] = 1u;
    cfg.mount.sensor_index[1] = 0u;
    assert(!b_attitude_init(&filter, &cfg)); /* 奇置換のみなら鏡像 */
    cfg.mount.direction[1] = -1;
    assert(b_attitude_init(&filter, &cfg));
    const BImuSample sample = level_with_bias();
    for (unsigned i = 1u; i <= 4u; ++i) {
        const uint64_t stamp = (uint64_t)i * 1000u;
        assert(b_attitude_add_static_sample(&filter, &sample, true, true,
                                            stamp, stamp));
    }
    close_to(filter.gyro_bias_rad_s[0], -0.2f, 1e-6f);
    close_to(filter.gyro_bias_rad_s[1], -0.1f, 1e-6f);
    close_to(filter.gyro_bias_rad_s[2], 0.05f, 1e-6f);
}

static void test_invalid_configs_and_missing_gravity(void)
{
    BAttitudeFilter f = {0};
    BAttitudeFilterConfig cfg = fixture_config();
    cfg.verified = false;
    assert(!b_attitude_init(&f, &cfg));
    assert(!f.initialized);
    cfg = fixture_config();
    cfg.gyro_alpha = 1.0f;
    assert(!b_attitude_init(&f, &cfg));
    cfg = fixture_config();
    cfg.gravity_min_m_s2 = NAN;
    assert(!b_attitude_init(&f, &cfg));
    cfg = fixture_config();
    cfg.max_sample_dt_s = 0.0f;
    assert(!b_attitude_init(&f, &cfg));

    calibrated_filter(&f);
    BImuSample sample = level_with_bias();
    sample.accel_m_s2[2] = 20.0f;
    BAttitudeEstimate out = {0};
    assert(!b_attitude_update(&f, &sample, true, 5000u, 5000u, &out));
    assert(!out.valid && !f.latest.valid);
    /* 再初期化で過去のバイアス・更新履歴は引き継がない。 */
    cfg = fixture_config();
    assert(b_attitude_init(&f, &cfg));
    assert(!f.bias_ready && !f.latest.valid);
}

int main(void)
{
    test_initial_gates_and_static_provenance();
    test_complementary_math_and_gyro_only();
    test_roll_wraparound();
    test_mount_validation_and_bias_mapping();
    test_invalid_configs_and_missing_gravity();
    puts("PASS: IMU static bias, mount transform, complementary pitch/roll, yaw rate and time guards");
    return 0;
}
