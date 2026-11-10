#include "balancing/app/balancing_operator_input.h"
#include "balancing/control/controller/balancing_core.h"
#include "balancing/control/estimator/balancing_observation.h"
#include "balancing/control/filter/balancing_attitude_filter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/*
 * Issue #10の実機なし連結試験。
 * 新規IMU/エンコーダ入力→時刻付き観測→500 Hz用コピー→
 * 姿勢フィルタ／操作元モデルまで、既存の公開APIだけを呼び出す。
 * 実CAN、実I2C、HID報告、IRQ排他、駆動出力は含まない。
 * 試験値は人工fixtureであり、実機のパラメータ・安全閾値ではない。
 */
static BImuSample sample_imu(float pitch_gyro_rad_s)
{
    BImuSample sample = {
        .accel_m_s2 = {0.0f, 0.0f, 9.81f},
        .gyro_rad_s = {0.0f, pitch_gyro_rad_s, 0.0f}
    };
    return sample;
}

static BAttitudeFilterConfig filter_fixture(void)
{
    BAttitudeFilterConfig cfg = {
        .mount = {{0u, 1u, 2u}, {1, 1, 1}, true},
        .bias_samples_required = 4u,
        .gravity_min_m_s2 = 8.0f,
        .gravity_max_m_s2 = 11.0f,
        .gyro_alpha = 0.95f,
        .max_sample_dt_s = 0.02f,
        .verified = true
    };
    return cfg;
}

static BOperatorInputConfig operator_fixture(void)
{
    BOperatorInputConfig cfg = {
        .input_timeout_us = 10000u,
        .neutral_hold_us = 4000u,
        .max_speed_m_s = 2.0f,
        .max_yaw_rate_rad_s = 1.0f,
        .neutral_speed_m_s = 0.02f,
        .neutral_yaw_rad_s = 0.02f,
        .speed_slew_m_s2 = 4.0f,
        .yaw_slew_rad_s2 = 2.0f,
        .verified = true
    };
    return cfg;
}

static BWheelObservationConfig wheel_fixture(float direction)
{
    BWheelObservationConfig cfg = {
        .wheel_radius_m = 0.1f,
        .motor_revs_per_wheel_rev = 10.0f,
        .direction = direction,
        .verified = true
    };
    return cfg;
}

static void test_sensor_filter_flow_and_freshness(void)
{
    BObservationStore store;
    BObservationSnapshot frozen = {0};
    BAttitudeFilter filter;
    BAttitudeEstimate estimated = {0};
    const BAttitudeFilterConfig filter_cfg = filter_fixture();
    const BWheelObservationConfig left = wheel_fixture(1.0f);
    const BWheelObservationConfig right = wheel_fixture(-1.0f);

    b_observation_init(&store);
    assert(b_attitude_init(&filter, &filter_cfg));

    /* 最初の4件は外部静止確認済みの新規サンプルとして校正へ渡す。
     * 同じセンサの読出しを繰り返して4件分に水増ししない。 */
    for (unsigned i = 1u; i <= 4u; ++i) {
        const uint64_t sample_us = (uint64_t)i * 2000u;
        BImuSample imu = sample_imu(0.125f);
        assert(b_observation_imu(&store, &imu, true, sample_us, sample_us));
        assert(b_observation_freeze(&store, sample_us, &frozen));
        assert(frozen.stamps.imu.valid);
        assert(frozen.stamps.imu.stamp_us == sample_us);
        assert(b_attitude_add_static_sample(&filter, &frozen.imu, true, true,
                                            frozen.stamps.imu.stamp_us, sample_us));
        assert(!b_attitude_add_static_sample(&filter, &frozen.imu, true, true,
                                             frozen.stamps.imu.stamp_us, sample_us));
    }
    assert(filter.bias_ready);

    /* 5回目の新規サンプルで初めて姿勢推定が有効となる。500 Hzの制御tickを
     * いくら追加しても新規センサ更新がなければタイムスタンプは変わらない。 */
    const BImuSample new_sample = sample_imu(0.2f);
    assert(b_observation_imu(&store, &new_sample, true, 10000u, 10000u));
    assert(b_observation_freeze(&store, 10000u, &frozen));
    assert(b_attitude_update(&filter, &frozen.imu, true,
                             frozen.stamps.imu.stamp_us, 10000u, &estimated));
    assert(estimated.valid && estimated.stamp_us == 10000u);
    assert(fabsf(estimated.pitch_rate_rad_s - 0.075f) < 1e-5f);
    const BAttitudeEstimate original = estimated;
    assert(b_observation_freeze(&store, 12000u, &frozen));
    assert(frozen.stamps.imu.stamp_us == 10000u);
    assert(!b_attitude_update(&filter, &frozen.imu, true,
                              frozen.stamps.imu.stamp_us, 12000u, &estimated));
    assert(!estimated.valid);
    assert(filter.latest.stamp_us == original.stamp_us);

    /* C620の実受信は別の周期でも更新できる。未取得のRS06と電源は
     * 自動的にvalidへ昇格しないため、全系統のARM準備には使えない。 */
    assert(b_observation_wheel(&store, 0u, &left, 8190u, 60, 9000u, 12000u));
    assert(b_observation_wheel(&store, 1u, &right, 2u, 60, 9000u, 12000u));
    assert(b_observation_wheel(&store, 0u, &left, 2u, 60, 11000u, 12000u));
    assert(b_observation_wheel(&store, 1u, &right, 8190u, 60, 11000u, 12000u));
    assert(b_observation_freeze(&store, 12000u, &frozen));
    assert(frozen.stamps.wheel[0].stamp_us == 11000u);
    assert(frozen.stamps.wheel[1].stamp_us == 11000u);
    assert(frozen.wheel[0].position_m > 0.0f);
    assert(frozen.wheel[1].position_m > 0.0f);

    const BFreshness recent = b_check_freshness(&frozen.stamps, 12000u);
    assert(recent.imu && recent.wheel[0] && recent.wheel[1]);
    assert(!recent.power && !recent.rs[0]);
    assert(!b_freshness_all(recent));

    /* 通信が途絶した後もfreezeの時刻で古いデータを新しく見せない。 */
    assert(b_observation_freeze(&store, 65000u, &frozen));
    assert(frozen.frozen_at_us == 65000u);
    assert(frozen.stamps.imu.stamp_us == 10000u);
    const BFreshness stale = b_check_freshness(&frozen.stamps, 65000u);
    assert(!stale.imu && !stale.wheel[0] && !stale.wheel[1]);
}

static void test_operator_and_mode_integration_boundary(void)
{
    BOperatorInput input;
    const BOperatorInputConfig cfg = operator_fixture();
    BOperatorTarget command = {0};
    BModeMachine machine;
    b_mode_init(&machine);
    assert(machine.mode == B_MODE_DISARMED);
    assert(b_operator_init(&input, &cfg));

    /* 操作元の受信が正常でも、それだけでは状態機械はARMしない。
     * FAULTや実機の受信系統を捏造して安全ゲートを通さない。 */
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 2000u, 2000u));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 6000u, 6000u));
    assert(b_operator_tick(&input, machine.mode, 6000u, &command));
    assert(command.operator_ready);
    assert(command.speed_m_s == 0.0f);

    BModeInput safety = {0};
    safety.arm_edge = false;
    const BModeOutput mode_result = b_mode_step(&machine, &safety, 6000u);
    assert(mode_result.mode == B_MODE_DISARMED);
    assert(!mode_result.drive_allowed);
    assert(!mode_result.request_rs_enable);

    /* 通信は新規の非中立入力へ更新してもDISARMEDなら0を保持。 */
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             1.0f, -0.4f, true, 8000u, 8000u));
    assert(b_operator_tick(&input, machine.mode, 8000u, &command));
    assert(command.speed_m_s == 0.0f);
    assert(command.yaw_rate_rad_s == 0.0f);

    /* 入力が期限切れになっても操作元は勝手に切り替わらない。
     * その後、非中立入力だけ復帰しても操作目標を許可しない。 */
    assert(b_operator_tick(&input, machine.mode, 20000u, &command));
    assert(!command.source_fresh && command.waiting_neutral);
    assert(input.selected == B_OPERATOR_USB_DUALSENSE);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             1.0f, 0.4f, true, 22000u, 22000u));
    assert(b_operator_tick(&input, machine.mode, 22000u, &command));
    assert(!command.operator_ready);
    assert(!b_operator_select(&input, B_OPERATOR_CAN_REMOTE, B_MODE_BALANCING));
    assert(machine.mode == B_MODE_DISARMED);
}

int main(void)
{
    test_sensor_filter_flow_and_freshness();
    test_operator_and_mode_integration_boundary();
    puts("PASS: Issue #10 integrated sensors, IMU math, operator loss and safe mode boundary");
    return 0;
}
