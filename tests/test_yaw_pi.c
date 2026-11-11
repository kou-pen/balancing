#include "balancing/control/controller/balancing_core.h"
#include "balancing/control/controller/balancing_yaw_pi.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    if (!isfinite(actual) || fabsf(actual - expected) > 1e-4f) {
        fprintf(stderr, "actual %.8g expected %.8g\n", (double)actual, (double)expected);
        assert(0);
    }
}

/* 人工的なN m上限に対して、飽和悪化方向だけ積分を止める。
 * 誤差反転時に積分が解消できないと、旋回停止後にも偏差が残る。 */
static void test_conditional_integration_and_wheel_priority(void)
{
    BYawPiState state = {0};
    const BYawPiGains gains = {1.0f, 1.0f, 1.0f, true};
    BYawPiInput input = {
        .target_rate_rad_s = 0.2f, .measured_rate_rad_s = 0.0f,
        .dt_s = 0.1f, .available_delta_nm = 1.0f,
        .balancing = true, .gyro_fresh = true
    };
    BYawPiOutput out = {0};
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(out.delta_nm, 0.22f);
    near(state.integral_nm, 0.02f);
    assert(!out.saturated);
    assert(!out.integral_frozen);
    for (int i = 1; i < 10; ++i) {
        assert(b_yaw_pi_step(&state, &gains, &input, &out));
    }
    near(state.integral_nm, 0.20f);

    /* LQRが車輪上限の大半を占めている条件を模擬する。
     * 8Nmの左右合計を優先し、左右4.5Nm上限の残り0.5NmをYawへ。 */
    input.target_rate_rad_s = 4.0f;
    input.available_delta_nm = 0.5f;
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(out.delta_nm, 0.5f);
    near(state.integral_nm, 0.2f);
    assert(out.saturated && out.integral_frozen);
    BWheelAllocation alloc = {0};
    assert(b_wheel_allocate(8.0f, out.delta_nm, 4.5f, 4.5f, &alloc));
    near(alloc.base_each_nm, 4.0f);
    near(alloc.left_nm, 3.5f);
    near(alloc.right_nm, 4.5f);
    assert(!alloc.saturated);

    /* 旋回要求を反転させると積分値を巻き戻す方向は許可する。 */
    input.target_rate_rad_s = -0.1f;
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(state.integral_nm, 0.19f);
    near(out.delta_nm, 0.09f);
    assert(!out.integral_frozen);
}

/* 0容量・モード解除・通信断で非ゼロ要求や積分が残らないこと。 */
static void test_disable_and_freshness_gate(void)
{
    BYawPiState state = {0};
    BYawPiGains gains = {2.0f, 4.0f, 0.6f, true};
    BYawPiInput input = {
        .target_rate_rad_s = 1.0f, .measured_rate_rad_s = 0.0f,
        .dt_s = 0.1f, .available_delta_nm = 0.0f,
        .balancing = true, .gyro_fresh = true
    };
    BYawPiOutput out = {0};
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(out.delta_nm, 0.0f);
    near(state.integral_nm, 0.0f);
    assert(out.integral_frozen && out.saturated);
    input.available_delta_nm = 2.0f;
    input.target_rate_rad_s = 0.1f;
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    assert(state.integral_nm > 0.0f);

    /* BALANCING以外のステップは積分・出力を消去し、自動再ARMしない。 */
    input.balancing = false;
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(out.delta_nm, 0.0f);
    near(state.integral_nm, 0.0f);
    assert(!state.active);
    input.balancing = true;
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(state.integral_nm, 0.04f);

    input.gyro_fresh = false;
    assert(!b_yaw_pi_step(&state, &gains, &input, &out));
    near(out.delta_nm, 0.0f);
    near(state.integral_nm, 0.0f);
    assert(!state.active);
    input.gyro_fresh = true;
    gains.verified = false;
    assert(!b_yaw_pi_step(&state, &gains, &input, &out));
    gains.verified = true;
    input.measured_rate_rad_s = NAN;
    assert(!b_yaw_pi_step(&state, &gains, &input, &out));
    input.measured_rate_rad_s = 0.0f;
    gains.ki_nm_rad = -1.0f;
    assert(!b_yaw_pi_step(&state, &gains, &input, &out));
    gains.ki_nm_rad = 4.0f;
    input.dt_s = 0.0f;
    assert(!b_yaw_pi_step(&state, &gains, &input, &out));
    assert(!b_yaw_pi_step(&state, &gains, &input, NULL));
    b_yaw_pi_reset(&state);
    near(state.integral_nm, 0.0f);
}

/* 積分単独の上限と符号反転での放電を、左右車輪制限とは独立に評価する。 */
static void test_integral_limit(void)
{
    BYawPiState state = {0};
    const BYawPiGains gains = {0.0f, 1.0f, 0.5f, true};
    BYawPiInput input = {
        .target_rate_rad_s = 0.1f, .measured_rate_rad_s = 0.0f,
        .dt_s = 1.0f, .available_delta_nm = 1.0f,
        .balancing = true, .gyro_fresh = true
    };
    BYawPiOutput out = {0};
    for (int i = 0; i < 20; ++i) {
        assert(b_yaw_pi_step(&state, &gains, &input, &out));
    }
    near(state.integral_nm, 0.5f);
    near(out.delta_nm, 0.5f);
    input.target_rate_rad_s = -0.1f;
    assert(b_yaw_pi_step(&state, &gains, &input, &out));
    near(state.integral_nm, 0.4f);

    /* 失敗値が有効な指令として残らない。 */
    input.available_delta_nm = -0.1f;
    assert(!b_yaw_pi_step(&state, &gains, &input, &out));
    near(state.integral_nm, 0.0f);
    near(out.delta_nm, 0.0f);
}

int main(void)
{
    test_conditional_integration_and_wheel_priority();
    test_integral_limit();
    test_disable_and_freshness_gate();
    puts("PASS: Yaw PI anti-windup, wheel priority, reset and stale gyro gates");
    return 0;
}
