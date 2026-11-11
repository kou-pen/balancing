#include "balancing/control/controller/balancing_leg_pd.h"

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

/* 2脚の不一致を消さず、初回の支持力が重力フィードフォワードだけとなる。 */
static void test_capture_and_initial_support(void)
{
    const BLegPdFeedback measured = {.left_length_m=0.32f, .right_length_m=0.28f};
    BLegPdReference ref = {0};
    BLegPdOutput out = {0};
    const BLegPdGains gains = {100.0f, 10.0f, 20.0f, 2.0f, 5.0f, true};
    assert(b_leg_pd_capture(&ref, &measured, true));
    near(ref.common_length_m, 0.30f);
    near(ref.half_difference_m, 0.02f);
    assert(b_leg_pd_compute(&ref, &gains, &measured, &out));
    near(out.left_force_n, 5.0f);
    near(out.right_force_n, 5.0f);
    near(out.roll_differential_n, 0.0f);

    /* 未校正Roll符号を抽象定義として固定し、右に同じ符号を加えない。 */
    const BLegPdFeedback moving = {
        .left_length_m = 0.31f, .right_length_m = 0.27f,
        .left_velocity_m_s = 0.02f, .right_velocity_m_s = -0.04f,
        .roll_rad = 0.1f, .roll_rate_rad_s = -0.2f
    };
    assert(b_leg_pd_compute(&ref, &gains, &moving, &out));
    near(out.roll_differential_n, -1.6f);
    near(out.left_force_n, 4.2f);
    near(out.right_force_n, 8.0f);
}

/* ARM後の目標の不用意な上書きと、数学的に不正な脚長を拒否する。 */
static void test_capture_denial_clears_reference(void)
{
    BLegPdReference ref = {0};
    const BLegPdFeedback measured = {.left_length_m=0.3f, .right_length_m=0.3f};
    assert(b_leg_pd_capture(&ref, &measured, true));
    assert(!b_leg_pd_capture(&ref, &measured, false));
    assert(!ref.valid);
    BLegPdFeedback invalid = measured;
    invalid.right_length_m = NAN;
    assert(!b_leg_pd_capture(&ref, &invalid, true));
    invalid.right_length_m = 0.0f;
    assert(!b_leg_pd_capture(&ref, &invalid, true));
    assert(!b_leg_pd_capture(NULL, &measured, true));
}

/* 微分項は目標値変更ではなく測定速度だけに反応する。 */
static void test_derivative_and_fail_closed(void)
{
    const BLegPdFeedback measured = {.left_length_m=0.3f, .right_length_m=0.3f};
    BLegPdReference ref = {0};
    BLegPdGains gains = {0.0f, 4.0f, 0.0f, 0.0f, 0.0f, true};
    BLegPdOutput out = {0};
    assert(b_leg_pd_capture(&ref, &measured, true));
    BLegPdFeedback moving = measured;
    moving.left_velocity_m_s = 0.5f;
    moving.right_velocity_m_s = -0.25f;
    assert(b_leg_pd_compute(&ref, &gains, &moving, &out));
    near(out.left_force_n, -2.0f);
    near(out.right_force_n, 1.0f);

    /* 異常入力なら以前の非ゼロ出力を残さず、全てゼロで失敗する。 */
    gains.verified = false;
    assert(!b_leg_pd_compute(&ref, &gains, &moving, &out));
    near(out.left_force_n, 0.0f);
    near(out.right_force_n, 0.0f);
    gains.verified = true;
    gains.roll_kp_n_rad = -1.0f;
    assert(!b_leg_pd_compute(&ref, &gains, &moving, &out));
    gains.roll_kp_n_rad = 0.0f;
    moving.roll_rad = INFINITY;
    assert(!b_leg_pd_compute(&ref, &gains, &moving, &out));
    moving.roll_rad = 0.0f;
    gains.gravity_feedforward_n = INFINITY;
    assert(!b_leg_pd_compute(&ref, &gains, &moving, &out));
    gains.gravity_feedforward_n = 0.0f;
    gains.length_kp_n_m = 3.0e38f;
    moving.left_length_m = 1.0e38f;
    assert(!b_leg_pd_compute(&ref, &gains, &moving, &out));
    near(out.roll_differential_n, 0.0f);
    assert(!b_leg_pd_compute(&ref, &gains, &moving, NULL));
}

int main(void)
{
    test_capture_and_initial_support();
    test_capture_denial_clears_reference();
    test_derivative_and_fail_closed();
    puts("PASS: initial leg targets, PD+Roll force, invalid-input and overflow gates");
    return 0;
}
