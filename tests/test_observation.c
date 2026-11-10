#include "balancing/control/controller/balancing_core.h"
#include "balancing/control/estimator/balancing_observation.h"
#include "balancing/communication/balancing_motor_can.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_TAU 6.2831853071795864769f

static void nearf(float actual, float expected, float tolerance)
{
    if (!(isfinite(actual) && isfinite(expected) &&
          fabsf(actual - expected) <= tolerance)) {
        fprintf(stderr, "actual=%f expected=%f tolerance=%f\n",
                (double)actual, (double)expected, (double)tolerance);
        assert(0);
    }
}

/* Artificial wheel dimensions; these are NOT verified M3508/C620 dimensions. */
static BWheelObservationConfig test_wheel(float sign)
{
    const BWheelObservationConfig config = {
        .wheel_radius_m = 0.1f,
        .motor_revs_per_wheel_rev = 10.0f,
        .direction = sign,
        .verified = true
    };
    return config;
}

static BImuSample test_imu(float rate)
{
    const BImuSample sample = {{0.0f, 0.0f, 9.81f}, {0.0f, rate, 0.0f}};
    return sample;
}

static void test_freeze_and_sample_freshness(void)
{
    BObservationStore store;
    BObservationSnapshot snapshot = {0};
    b_observation_init(&store);
    assert(b_observation_freeze(&store, 0u, &snapshot));
    assert(!snapshot.stamps.imu.valid);
    assert(!snapshot.stamps.wheel[0].valid);
    const BImuSample imu = test_imu(0.25f);
    assert(!b_observation_imu(&store, &imu, false, 1000u, 2000u));
    assert(!store.stamps.imu.valid);
    assert(b_observation_imu(&store, &imu, true, 1000u, 2000u));
    assert(!b_observation_imu(&store, &imu, true, 1000u, 2200u));
    assert(!b_observation_imu(&store, &imu, true, 999u, 2200u));
    assert(!b_observation_imu(&store, &imu, true, 2500u, 2200u));
    assert(b_observation_freeze(&store, 3000u, &snapshot));
    assert(snapshot.frozen_at_us == 3000u);
    assert(snapshot.stamps.imu.stamp_us == 1000u);
    nearf(snapshot.imu.gyro_rad_s[1], 0.25f, 1e-7f);

    BFreshness f = b_check_freshness(&snapshot.stamps, 51000u);
    assert(f.imu); /* age == 50000us, inclusive */
    assert(!f.wheel[0] && !f.rs[0] && !f.power);
    assert(!b_freshness_all(f));
    f = b_check_freshness(&snapshot.stamps, 51001u);
    assert(!f.imu);
    f = b_check_freshness(&snapshot.stamps, 999u);
    assert(!f.imu); /* future sample never valid */

    const BImuSample updated = test_imu(-0.3f);
    assert(b_observation_imu(&store, &updated, true, 4000u, 5000u));
    nearf(snapshot.imu.gyro_rad_s[1], 0.25f, 1e-7f);
    assert(snapshot.stamps.imu.stamp_us == 1000u); /* frozen copy */
    assert(b_observation_freeze(&store, 5000u, &snapshot));
    nearf(snapshot.imu.gyro_rad_s[1], -0.3f, 1e-7f);
    assert(snapshot.stamps.imu.stamp_us == 4000u);
    assert(!b_observation_freeze(&store, 3999u, &snapshot));
    assert(!snapshot.stamps.imu.valid);
}

static void test_invalid_imu_never_refreshes(void)
{
    BObservationStore store;
    b_observation_init(&store);
    const BImuSample good = test_imu(0.125f);
    assert(b_observation_imu(&store, &good, true, 1u, 1u));
    BImuSample bad = good;
    bad.gyro_rad_s[2] = NAN;
    assert(!b_observation_imu(&store, &bad, true, 10u, 10u));
    assert(!b_observation_imu(&store, NULL, true, 10u, 10u));
    assert(!b_observation_imu(NULL, &good, true, 10u, 10u));
    assert(store.stamps.imu.stamp_us == 1u);
    nearf(store.imu.gyro_rad_s[1], 0.125f, 1e-7f);
}

static void test_encoder_wrap_speed_and_symmetry(void)
{
    BObservationStore store;
    b_observation_init(&store);
    const BWheelObservationConfig left = test_wheel(1.0f);
    const BWheelObservationConfig right = test_wheel(-1.0f);
    assert(b_observation_wheel(&store, 0, &left, 8190u, 60, 1000u, 1000u));
    assert(b_observation_wheel(&store, 1, &right, 2u, 60, 1000u, 1000u));
    nearf(store.wheel[0].position_m, 0.0f, 0.0f);
    nearf(store.wheel[1].position_m, 0.0f, 0.0f);
    assert(b_observation_wheel(&store, 0, &left, 2u, 60, 2000u, 2000u));
    assert(b_observation_wheel(&store, 1, &right, 8190u, 60, 2000u, 2000u));
    const float delta_m = TEST_TAU * 0.1f * 4.0f / (8192.0f * 10.0f);
    nearf(store.wheel[0].position_m, delta_m, 1e-7f);
    nearf(store.wheel[1].position_m, delta_m, 1e-7f);
    nearf(store.wheel[0].velocity_m_s, TEST_TAU * 0.1f / 10.0f, 1e-6f);
    nearf(store.wheel[1].velocity_m_s, -TEST_TAU * 0.1f / 10.0f, 1e-6f);

    BObservationSnapshot snapshot = {0};
    assert(b_observation_freeze(&store, 2100u, &snapshot));
    assert(snapshot.stamps.wheel[0].stamp_us == 2000u);
    const BFreshness at_limit = b_check_freshness(&snapshot.stamps, 22000u);
    const BFreshness after_limit = b_check_freshness(&snapshot.stamps, 22001u);
    assert(at_limit.wheel[0] && at_limit.wheel[1]);
    assert(!after_limit.wheel[0] && !after_limit.wheel[1]);
}

static void test_real_c620_codec_to_observation(void)
{
    const BMotorCanFrame frame = {
        .id = 0x201u,
        .data = {0x1fu, 0xfeu, 0x00u, 0x3cu, 0x00u, 0x00u, 30u, 0u},
        .dlc = 8u,
        .extended = false
    };
    BC620Feedback fb = {0};
    assert(b_c620_parse_feedback(&frame, &fb));
    assert(fb.motor_id == 1u && fb.encoder_raw == 8190u && fb.rpm == 60);
    BObservationStore store;
    b_observation_init(&store);
    const BWheelObservationConfig config = test_wheel(1.0f);
    assert(b_observation_wheel(&store, fb.motor_id - 1u, &config,
                               fb.encoder_raw, fb.rpm, 1000u, 1500u));
    nearf(store.wheel[0].position_m, 0.0f, 0.0f);
    assert(store.stamps.wheel[0].stamp_us == 1000u);
    assert(!store.stamps.wheel[1].valid);
}

static void test_invalid_encoder_and_config_fail_closed(void)
{
    BObservationStore store;
    b_observation_init(&store);
    BWheelObservationConfig config = test_wheel(1.0f);
    assert(b_observation_wheel(&store, 0, &config, 0u, 0, 1000u, 1000u));
    assert(!b_observation_wheel(&store, 0, &config, 4096u, 0, 2000u, 2000u));
    assert(!b_observation_wheel(&store, 0, &config, 8192u, 0, 2000u, 2000u));
    assert(!b_observation_wheel(&store, 2, &config, 1u, 0, 2000u, 2000u));
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 1000u, 2000u));
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2001u, 2000u));
    assert(!b_observation_wheel(NULL, 0, &config, 1u, 0, 2000u, 2000u));
    config.verified = false;
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2000u, 2000u));
    config = test_wheel(1.0f);
    config.motor_revs_per_wheel_rev = 0.0f;
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2000u, 2000u));
    config = test_wheel(1.0f);
    config.direction = 0.0f;
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2000u, 2000u));
    config = test_wheel(-1.0f);
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2000u, 2000u));
    config = test_wheel(1.0f);
    config.wheel_radius_m = 0.2f;
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2000u, 2000u));
    config = test_wheel(1.0f);
    config.motor_revs_per_wheel_rev = 20.0f;
    assert(!b_observation_wheel(&store, 0, &config, 1u, 0, 2000u, 2000u));
    assert(store.stamps.wheel[0].stamp_us == 1000u);
    assert(store.wheel[0].encoder_raw == 0u);
    nearf(store.wheel[0].position_m, 0.0f, 0.0f);
}

int main(void)
{
    test_freeze_and_sample_freshness();
    test_invalid_imu_never_refreshes();
    test_encoder_wrap_speed_and_symmetry();
    test_real_c620_codec_to_observation();
    test_invalid_encoder_and_config_fail_closed();
    puts("PASS: IMU/wheel observation copies, independent timestamps, C620 codec, wrap and freshness");
    return 0;
}
