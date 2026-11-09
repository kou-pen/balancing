#include "balancing_core.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define NEAR(actual, expected) assert(fabsf((actual) - (expected)) < 0.0001f)

static BSampleStamp stamp(uint64_t time_us)
{
    BSampleStamp sample = {true, time_us};
    return sample;
}

static BObservedStamps all_stamps(uint64_t time_us)
{
    BObservedStamps v = {0};
    unsigned i;
    v.imu = stamp(time_us);
    v.power = stamp(time_us);
    for (i = 0; i < 4; ++i) {
        v.rs[i] = stamp(time_us);
    }
    for (i = 0; i < 2; ++i) {
        v.wheel[i] = stamp(time_us);
    }
    return v;
}

static BModeInput all_guards_ready(uint64_t now_us)
{
    BObservedStamps stamps = all_stamps(now_us);
    BModeInput input = {0};
    input.fresh = b_check_freshness(&stamps, now_us);
    input.power_on = true;
    input.calibration_verified = true;
    input.model_verified = true;
    input.limits_verified = true;
    input.posture_ok = true;
    return input;
}

static void test_freshness(void)
{
    uint64_t now = UINT64_C(1000000);
    BObservedStamps o = all_stamps(now);
    BFreshness fresh = b_check_freshness(&o, now);
    assert(b_freshness_all(fresh));

    o.imu.stamp_us = now - 50000u;
    o.rs[0].stamp_us = now - 30000u;
    o.wheel[0].stamp_us = now - 20000u;
    o.power.stamp_us = now - 500000u;
    assert(b_freshness_all(b_check_freshness(&o, now)));

    o.imu.stamp_us -= 1u;
    assert(!b_check_freshness(&o, now).imu);
    o.imu = stamp(now);
    o.rs[2].stamp_us = now - 30001u;
    assert(!b_check_freshness(&o, now).rs[2]);
    o.rs[2] = stamp(now);
    o.wheel[1].valid = false;
    assert(!b_check_freshness(&o, now).wheel[1]);
    o.wheel[1] = stamp(now);
    o.power.stamp_us = now - 500001u;
    assert(!b_check_freshness(&o, now).power);
    o.power = stamp(now + 1u);
    assert(!b_freshness_all(b_check_freshness(&o, now)));
    assert(!b_freshness_all(b_check_freshness(NULL, now)));
}

static void test_arm_handshake_and_disarm(void)
{
    const uint64_t t = UINT64_C(1000000);
    BModeMachine machine;
    BModeInput input = all_guards_ready(t);
    BModeOutput output;
    unsigned i;

    b_mode_init(&machine);
    assert(machine.mode == B_MODE_DISARMED);

    input.arm_edge = true;
    input.model_verified = false;
    output = b_mode_step(&machine, &input, t);
    assert(output.arm_rejected && !output.drive_allowed);
    assert(!output.request_rs_enable);
    assert(machine.mode == B_MODE_DISARMED);

    input.model_verified = true;
    /* 車輪1台の測定が無効であればARMを拒否する。 */
    input.fresh.wheel[1] = false;
    output = b_mode_step(&machine, &input, t);
    assert(output.arm_rejected && !output.request_rs_enable);
    input.fresh.wheel[1] = true;
    output = b_mode_step(&machine, &input, t);
    assert(output.request_rs_enable && !output.drive_allowed);
    assert(machine.arm_pending && output.mode == B_MODE_DISARMED);

    input.arm_edge = false;
    for (i = 0; i < 4; ++i) {
        input.rs_operational[i] = true;
        input.rs_enable_sent[i] = stamp(t + 500u);
        input.rs_reply[i] = stamp(t); /* ARM要求以前の観測では駄目 */
    }
    output = b_mode_step(&machine, &input, t + 1000u);
    assert(!output.drive_allowed && machine.arm_pending);

    for (i = 0; i < 3; ++i) {
        input.rs_reply[i] = stamp(t + 1000u);
    }
    output = b_mode_step(&machine, &input, t + 2000u);
    assert(!output.drive_allowed);

    input.rs_reply[3] = stamp(t + 1000u);
    output = b_mode_step(&machine, &input, t + 2000u);
    assert(output.drive_allowed && output.mode == B_MODE_BALANCING);

    input.disarm_edge = true;
    output = b_mode_step(&machine, &input, t + 3000u);
    assert(!output.drive_allowed && output.request_rs_disable);
    assert(output.mode == B_MODE_DISARMED);
}

static void test_fault_latch_and_rearm(void)
{
    uint64_t t = UINT64_C(2000000);
    BModeMachine machine;
    BModeInput input = all_guards_ready(t);
    BModeOutput output;
    unsigned i;

    b_mode_init(&machine);
    input.arm_edge = true;
    output = b_mode_step(&machine, &input, t);
    assert(output.request_rs_enable);
    input.arm_edge = false;
    for (i = 0; i < 4; ++i) {
        input.rs_operational[i] = true;
        input.rs_enable_sent[i] = stamp(t + 500u);
        input.rs_reply[i] = stamp(t + 1000u);
    }
    output = b_mode_step(&machine, &input, t + 1000u);
    assert(output.drive_allowed);

    input.fresh.imu = false;
    input.fresh.imu = false;
    output = b_mode_step(&machine, &input, t + 2000u);
    assert(!output.drive_allowed && output.request_rs_disable);
    assert(output.mode == B_MODE_FAULT && output.fault_cause == B_FAULT_GUARD);

    input.arm_edge = true;
    output = b_mode_step(&machine, &input, t + 3000u);
    assert(output.mode == B_MODE_FAULT && !output.drive_allowed);
    assert(!output.request_rs_enable);

    input.clear_fault_edge = true;
    output = b_mode_step(&machine, &input, t + 4000u);
    assert(output.mode == B_MODE_FAULT);
    input.clear_authorized = true;
    output = b_mode_step(&machine, &input, t + 5000u);
    assert(output.mode == B_MODE_DISARMED && !output.drive_allowed);
    assert(output.fault_cause == B_FAULT_NONE);

    input.estop_active = true;
    output = b_mode_step(&machine, &input, t + 6000u);
    assert(output.mode == B_MODE_FAULT);
    assert(output.request_power_off && output.request_rs_disable);
    assert(output.fault_cause == B_FAULT_ESTOP);
    output = b_mode_step(&machine, &input, t + 7000u);
    assert(output.mode == B_MODE_FAULT); /* 解除要求中でもESTOP優先 */
}

static void test_pending_cancellation(void)
{
    BModeMachine machine;
    BModeInput input = all_guards_ready(10000u);
    BModeOutput output;
    b_mode_init(&machine);
    input.arm_edge = true;
    output = b_mode_step(&machine, &input, 10000u);
    assert(output.request_rs_enable);
    input.arm_edge = false;
    input.fresh.imu = false;
    output = b_mode_step(&machine, &input, 12000u);
    assert(output.mode == B_MODE_DISARMED);
    assert(output.request_rs_disable && output.arm_rejected);
    assert(!machine.arm_pending);
}

static void test_rate_limit(void)
{
    float value = -999.0f;
    assert(b_rate_limit(0.0f, 10.0f, 2.0f, 0.1f, &value));
    NEAR(value, 0.2f);
    assert(b_rate_limit(value, -10.0f, 2.0f, 0.1f, &value));
    NEAR(value, 0.0f);
    assert(!b_rate_limit(0.0f, NAN, 2.0f, 0.1f, &value));
    NEAR(value, 0.0f);
    assert(!b_rate_limit(0.0f, 1.0f, 0.0f, 0.1f, &value));
    assert(!b_rate_limit(0.0f, 1.0f, 2.0f, -0.1f, &value));
}

static void test_wheel_allocation(void)
{
    BWheelAllocation w;
    unsigned i;
    assert(b_wheel_allocate(8.0f, 3.0f, 5.0f, 6.0f, &w));
    NEAR(w.base_each_nm, 4.0f);
    NEAR(w.yaw_applied_nm, 1.0f);
    NEAR(w.left_nm, 3.0f);
    NEAR(w.right_nm, 5.0f);
    assert(w.saturated);

    assert(b_wheel_allocate(50.0f, 2.0f, 5.0f, 6.0f, &w));
    NEAR(w.base_each_nm, 5.0f);
    NEAR(w.yaw_applied_nm, 0.0f);
    assert(w.saturated);

    assert(!b_wheel_allocate(1.0f, 0.0f, -1.0f, 5.0f, &w));
    NEAR(w.left_nm, 0.0f);

    /* 再現可能な入力網羅：左右上限および倒立・Yaw要求が異なっても必ず上限内。 */
    for (i = 1; i <= 10000; ++i) {
        float a = (float)((int)(i % 131u) - 65) / 10.0f;
        float y = (float)((int)((i * 7u) % 103u) - 51) / 10.0f;
        float ll = (float)((i % 37u) + 1u) / 10.0f;
        float rl = (float)((i % 23u) + 1u) / 10.0f;
        assert(b_wheel_allocate(a, y, ll, rl, &w));
        assert(fabsf(w.left_nm) <= ll + 0.0001f);
        assert(fabsf(w.right_nm) <= rl + 0.0001f);
        NEAR(w.left_nm + w.right_nm, 2.0f * w.base_each_nm);
        assert(fabsf(w.base_each_nm) <= fminf(ll, rl) + 0.0001f);
    }
}

static void test_lqr_no_unverified_gains(void)
{
    BLqrConfig c = {0};
    BControlState state = {0};
    BControlState target = {0};
    BControlTorque output = {123.0f, 456.0f};

    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_THETA] = 2.0f;
    c.gain[B_OUTPUT_HIP_TOTAL][B_STATE_POSITION] = 3.0f;
    c.max_position_error_m = 0.2f;
    state.theta_rad = 0.1f;
    state.position_m = 1.0f;

    assert(!b_lqr_compute(&c, &state, &target, &output));
    NEAR(output.wheel_total_nm, 0.0f);
    NEAR(output.hip_total_nm, 0.0f);

    /* 数学的な試験係数。実機K_300を意味しない。 */
    c.verified = true;
    assert(b_lqr_compute(&c, &state, &target, &output));
    NEAR(output.wheel_total_nm, -0.2f);
    NEAR(output.hip_total_nm, -0.6f);

    state.theta_rad = NAN;
    assert(!b_lqr_compute(&c, &state, &target, &output));
    NEAR(output.wheel_total_nm, 0.0f);
    NEAR(output.hip_total_nm, 0.0f);
}


static void test_lqr_named_state_mapping(void)
{
    BLqrConfig c = {0};
    BControlState state = {0};
    BControlState target = {0};
    BControlTorque result = {0};

    /* 各フィールドが別の列へ正しく渡ることを独立した係数で検証。 */
    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_THETA] = 1.0f;
    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_THETA_RATE] = 2.0f;
    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_POSITION] = 3.0f;
    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_VELOCITY] = 4.0f;
    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_PITCH] = 5.0f;
    c.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_PITCH_RATE] = 6.0f;
    c.gain[B_OUTPUT_HIP_TOTAL][B_STATE_PITCH] = 7.0f;
    c.max_position_error_m = 1.0f;
    c.verified = true;

    state.theta_rad = 0.1f;
    state.theta_rate_rad_s = 0.2f;
    state.position_m = 0.3f;
    state.velocity_m_s = 0.4f;
    state.pitch_rad = 0.5f;
    state.pitch_rate_rad_s = 0.6f;

    assert(b_lqr_compute(&c, &state, &target, &result));
    NEAR(result.wheel_total_nm, -9.1f);
    NEAR(result.hip_total_nm, -3.5f);

    /* 目標状態の位置だけを変えても、対応する列だけに反映される。 */
    target.position_m = 0.1f;
    assert(b_lqr_compute(&c, &state, &target, &result));
    NEAR(result.wheel_total_nm, -8.8f);
    NEAR(result.hip_total_nm, -3.5f);
}

static void test_scheduler_delay_and_clock_regression(void)
{
    BModeMachine machine;
    BModeInput input = all_guards_ready(1000000u);
    BModeOutput output;
    unsigned i;

    b_mode_init(&machine);
    input.arm_edge = true;
    output = b_mode_step(&machine, &input, 1000000u);
    assert(output.request_rs_enable);
    input.arm_edge = false;
    for (i = 0; i < 4; ++i) {
        input.rs_operational[i] = true;
        input.rs_enable_sent[i] = stamp(1000500u);
        input.rs_reply[i] = stamp(1001000u);
    }
    output = b_mode_step(&machine, &input, 1001000u);
    assert(output.drive_allowed);
    output = b_mode_step(&machine, &input, 1007000u);
    assert(output.mode == B_MODE_FAULT);
    assert(output.fault_cause == B_FAULT_SCHEDULER);
    assert(output.request_rs_disable && !output.drive_allowed);

    /* 時計が逆行する場合も、稼働中は必ず駆動を停止。 */
    b_mode_init(&machine);
    input.arm_edge = true;
    output = b_mode_step(&machine, &input, 2000000u);
    assert(output.request_rs_enable);
    input.arm_edge = false;
    for (i = 0; i < 4; ++i) {
        input.rs_enable_sent[i] = stamp(2000500u);
        input.rs_reply[i] = stamp(2001000u);
    }
    output = b_mode_step(&machine, &input, 2001000u);
    assert(output.drive_allowed);
    output = b_mode_step(&machine, &input, 2000000u);
    assert(output.mode == B_MODE_FAULT);
    assert(output.fault_cause == B_FAULT_SCHEDULER);
}


static void test_rs_enable_dispatch_confirmation(void)
{
    const uint64_t t = UINT64_C(3000000);
    BModeMachine machine;
    BModeInput input = all_guards_ready(t);
    BModeOutput output;
    unsigned i;

    b_mode_init(&machine);
    input.arm_edge = true;
    output = b_mode_step(&machine, &input, t);
    assert(output.request_rs_enable);
    input.arm_edge = false;

    /* 新しい正常応答でも、送信証跡がなければARMしてはいけない。 */
    for (i = 0; i < 4; ++i) {
        input.rs_operational[i] = true;
        input.rs_reply[i] = stamp(t + 1000u);
    }
    output = b_mode_step(&machine, &input, t + 1000u);
    assert(!output.drive_allowed && machine.arm_pending);

    /* 送信前に届いた応答では解除されない。 */
    for (i = 0; i < 4; ++i) {
        input.rs_enable_sent[i] = stamp(t + 1500u);
    }
    output = b_mode_step(&machine, &input, t + 2000u);
    assert(!output.drive_allowed && machine.arm_pending);

    /* 4台全てで送信後の新規正常応答が揃ったときだけ許可。 */
    for (i = 0; i < 4; ++i) {
        input.rs_reply[i] = stamp(t + 2500u);
    }
    output = b_mode_step(&machine, &input, t + 3000u);
    assert(output.drive_allowed && output.mode == B_MODE_BALANCING);
}

int main(void)
{
    test_freshness();
    test_arm_handshake_and_disarm();
    test_fault_latch_and_rearm();
    test_pending_cancellation();
    test_rate_limit();
    test_wheel_allocation();
    test_lqr_no_unverified_gains();
    test_lqr_named_state_mapping();
    test_scheduler_delay_and_clock_regression();
    test_rs_enable_dispatch_confirmation();
    puts("合格：鮮度監視、ARM応答、FAULT、目標制限、車輪配分、LQR数学処理");
    return 0;
}
