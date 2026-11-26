#include "balancing/control/filter/ImuAttitudeSource.hpp"

#include <assert.h>
#include <cmath>
using namespace balancing;
using namespace balancing::attitude_math;
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_GRAVITY 9.81f

/*
 * 以下の軸・係数・バイアスは数値テスト用であり、BNO055の実取付方向や
 * フィルタ係数を決定しない。センサ新規性もテスト入力で明示的に模擬する。
 */
static AttitudeFilterConfig fixture_config(void)
{
    const AttitudeFilterConfig cfg = {
         {{0u, 1u, 2u}, {1, 1, 1}, true},
         4u,
         8.0f,
         11.0f,
         0.9f,
         0.01f,
         true
    };
    return cfg;
}

static ImuSample level_with_bias(void)
{
    const ImuSample x = {
         {0.0f, 0.0f, TEST_GRAVITY},
         {0.1f, -0.2f, 0.05f}
    };
    return x;
}

static void close_to(float got, float expected, float tol)
{
    if (!(std::isfinite(got) && std::fabs(got - expected) <= tol)) {
        fprintf(stderr, "got=%g expected=%g tolerance=%g\n",
                (double)got, (double)expected, (double)tol);
        assert(false);
    }
}

static void calibrated_filter(AttitudeFilterState& filter)
{
    const AttitudeFilterConfig cfg = fixture_config();
    const ImuSample sample = level_with_bias();
    assert(initialize(filter, cfg));
    for (unsigned i = 1; i <= 4; ++i) {
        const uint64_t stamp = (uint64_t)i * 1000u;
        assert(addStaticSample(filter, sample, true, true,
                                            stamp, stamp));
    }
    assert(filter.bias_ready);
    assert(filter.bias_samples_collected == 4u);
    close_to(filter.gyro_bias_rad_s[0], 0.1f, 1e-6f);
    close_to(filter.gyro_bias_rad_s[1], -0.2f, 1e-6f);
    close_to(filter.gyro_bias_rad_s[2], 0.05f, 1e-6f);
}

/* 静止確認のない加速度ノルムだけからバイアスを学習してはいけない。
 * バイアス確定前に角度・Yaw角速度を取り出そうとしても拒否する。 */
static void test_initial_gates_and_static_provenance(void)
{
    AttitudeFilterState filter{};
    const AttitudeFilterConfig cfg = fixture_config();
    const ImuSample good = level_with_bias();
    AttitudeEstimate out{};
    assert(initialize(filter, cfg));
    assert(!update(filter, good, true, 1000u, 1000u, out));
    assert(!out.valid);
    assert(!addStaticSample(filter, good, false, true,
                                         1000u, 1000u));
    assert(!addStaticSample(filter, good, true, false,
                                         1000u, 1000u));
    assert(!addStaticSample(filter, good, true, true,
                                         2000u, 1000u));
    ImuSample invalid = good;
    invalid.gyro_rad_s[1] = NAN;
    assert(!addStaticSample(filter, invalid, true, true,
                                         1000u, 1000u));
    invalid = good;
    invalid.accel_m_s2[2] = 20.0f;
    assert(!addStaticSample(filter, invalid, true, true,
                                         1000u, 1000u));
    assert(filter.bias_samples_collected == 0u);
    assert(addStaticSample(filter, good, true, true,
                                        1000u, 1000u));
    assert(!addStaticSample(filter, good, true, true,
                                         1000u, 2000u));
    assert(filter.bias_samples_collected == 1u);
    assert(!filter.bias_ready);
}

/* 同一サンプルを繰り返し使用すると積分dtを捏造することになる。
 * 実新規時刻の差でだけ角速度を積分し、静止ジャイロの平均値を差し引く。 */
static void test_complementary_math_and_gyro_only(void)
{
    AttitudeFilterState filter;
    calibrated_filter(filter);
    const ImuSample level = level_with_bias();
    AttitudeEstimate out{};
    assert(!update(filter, level, true, 4000u, 4000u, out));
    assert(update(filter, level, true, 5000u, 5000u, out));
    assert(out.valid && out.accel_fused && out.stamp_us == 5000u);
    close_to(out.pitch_rad, 0.0f, 1e-6f);
    close_to(out.roll_rad, 0.0f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.0f, 1e-6f);

    const float pitch = 0.1f;
    const float roll = 0.05f;
    ImuSample tilting = level;
    tilting.accel_m_s2[0] = -TEST_GRAVITY * std::sin(pitch);
    tilting.accel_m_s2[1] = TEST_GRAVITY * std::cos(pitch) * std::sin(roll);
    tilting.accel_m_s2[2] = TEST_GRAVITY * std::cos(pitch) * std::cos(roll);
    tilting.gyro_rad_s[1] += 0.5f;
    tilting.gyro_rad_s[2] += 0.2f;
    assert(update(filter, tilting, true, 7000u, 7000u, out));
    /* 2 ms × 0.5 rad/s = 0.001radのジャイロ予測に重力角の10%を足す。 */
    close_to(out.pitch_rad, 0.001f + 0.1f * (pitch - 0.001f), 2e-5f);
    close_to(out.roll_rad, 0.005f, 2e-5f);
    close_to(out.pitch_rate_rad_s, 0.5f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.2f, 1e-6f);

    /* 加速度が動的に20 m/s²になったときは、偽の重力傾斜で補正しない。
     * ただしジャイロの新規サンプルとして積分は継続する。 */
    ImuSample disturbed = tilting;
    disturbed.accel_m_s2[0] = 0.0f;
    disturbed.accel_m_s2[1] = 0.0f;
    disturbed.accel_m_s2[2] = 20.0f;
    const float before = out.pitch_rad;
    assert(update(filter, disturbed, true, 9000u, 9000u, out));
    assert(!out.accel_fused && out.stamp_us == 9000u);
    close_to(out.pitch_rad, before + 0.001f, 2e-5f);

    ImuSample invalid = disturbed;
    invalid.gyro_rad_s[0] = INFINITY;
    assert(!update(filter, invalid, true, 10000u, 10000u, out));
    assert(!out.valid && filter.latest.stamp_us == 9000u);
    assert(!update(filter, tilting, false, 10000u, 10000u, out));
    assert(!update(filter, tilting, true, 9000u, 10000u, out));
    assert(!update(filter, tilting, true, 11000u, 10000u, out));
    assert(!update(filter, tilting, true, 30000u, 30000u, out));
    assert(filter.latest.stamp_us == 9000u); /* 時刻欠落を勝手に補間しない */
}

/* ±π近辺で素朴な線形平均を使うと約0radへ誤って飛ぶ。
 * 補正差を最短角度で取るため、境界の両側でも連続していることを検証。 */
static void test_roll_wraparound(void)
{
    AttitudeFilterState filter;
    calibrated_filter(filter);
    const ImuSample bias = level_with_bias();
    AttitudeEstimate out{};

    const float near_pi = 179.0f * TEST_PI / 180.0f;
    ImuSample sample = bias;
    sample.accel_m_s2[1] = TEST_GRAVITY * std::sin(near_pi);
    sample.accel_m_s2[2] = TEST_GRAVITY * std::cos(near_pi);
    assert(update(filter, sample, true, 5000u, 5000u, out));
    assert(out.roll_rad > 3.0f);

    sample.accel_m_s2[1] = -TEST_GRAVITY * std::sin(near_pi);
    sample.accel_m_s2[2] = TEST_GRAVITY * std::cos(near_pi);
    assert(update(filter, sample, true, 7000u, 7000u, out));
    assert(std::fabs(out.roll_rad) > 3.0f); /* 0radへジャンプしない */
}

/* 取付座標は任意の反転ではなく右手系の剛体回転が必要。
 * 軸重複と鏡像を拒否し、置換と符号で別の正しい取付方位を表現する。 */
static void test_mount_validation_and_bias_mapping(void)
{
    AttitudeFilterState filter{};
    AttitudeFilterConfig cfg = fixture_config();
    cfg.mount.sensor_index[1] = 0u;
    assert(!initialize(filter, cfg)); /* 重複 */
    cfg = fixture_config();
    cfg.mount.sensor_index[0] = 1u;
    cfg.mount.sensor_index[1] = 0u;
    assert(!initialize(filter, cfg)); /* 奇置換のみなら鏡像 */
    cfg.mount.direction[1] = -1;
    assert(initialize(filter, cfg));
    const ImuSample sample = level_with_bias();
    for (unsigned i = 1u; i <= 4u; ++i) {
        const uint64_t stamp = (uint64_t)i * 1000u;
        assert(addStaticSample(filter, sample, true, true,
                                            stamp, stamp));
    }
    close_to(filter.gyro_bias_rad_s[0], -0.2f, 1e-6f);
    close_to(filter.gyro_bias_rad_s[1], -0.1f, 1e-6f);
    close_to(filter.gyro_bias_rad_s[2], 0.05f, 1e-6f);
}

static void test_invalid_configs_and_missing_gravity(void)
{
    AttitudeFilterState f{};
    AttitudeFilterConfig cfg = fixture_config();
    cfg.verified = false;
    assert(!initialize(f, cfg));
    assert(!f.initialized);
    cfg = fixture_config();
    cfg.gyro_alpha = 1.0f;
    assert(!initialize(f, cfg));
    cfg = fixture_config();
    cfg.gravity_min_m_s2 = NAN;
    assert(!initialize(f, cfg));
    cfg = fixture_config();
    cfg.max_sample_dt_s = 0.0f;
    assert(!initialize(f, cfg));

    calibrated_filter(f);
    ImuSample sample = level_with_bias();
    sample.accel_m_s2[2] = 20.0f;
    AttitudeEstimate out{};
    assert(!update(f, sample, true, 5000u, 5000u, out));
    assert(!out.valid && !f.latest.valid);
    /* 再初期化で過去のバイアス・更新履歴は引き継がない。 */
    cfg = fixture_config();
    assert(initialize(f, cfg));
    assert(!f.bias_ready && !f.latest.valid);
}

int main()
{
    test_initial_gates_and_static_provenance();
    test_complementary_math_and_gyro_only();
    test_roll_wraparound();
    test_mount_validation_and_bias_mapping();
    test_invalid_configs_and_missing_gravity();
    puts("PASS: IMU static bias, mount transform, complementary pitch/roll, yaw rate and time guards");
    return 0;
}
