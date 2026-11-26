#include "balancing/control/controller/BalanceControl.hpp"
using namespace balancing;
using namespace balancing::balance_math;

#include <assert.h>
#include <cmath>
#include <limits>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define NEAR(actual, expected) assert(std::fabs((actual) - (expected)) < 0.0001f)

static void test_wheel_allocation(void)
{
    WheelAllocation w;
    unsigned i;
    assert(allocateWheels(8.0f, 3.0f, 5.0f, 6.0f, w));
    NEAR(w.base_each_nm, 4.0f);
    NEAR(w.yaw_applied_nm, 1.0f);
    NEAR(w.left_nm, 3.0f);
    NEAR(w.right_nm, 5.0f);
    assert(w.saturated);

    assert(allocateWheels(50.0f, 2.0f, 5.0f, 6.0f, w));
    NEAR(w.base_each_nm, 5.0f);
    NEAR(w.yaw_applied_nm, 0.0f);
    assert(w.saturated);

    assert(!allocateWheels(1.0f, 0.0f, -1.0f, 5.0f, w));
    NEAR(w.left_nm, 0.0f);

    /* 再現可能な入力網羅：左右上限および倒立・Yaw要求が異なっても必ず上限内。 */
    for (i = 1; i <= 10000; ++i) {
        float a = (float)((int)(i % 131u) - 65) / 10.0f;
        float y = (float)((int)((i * 7u) % 103u) - 51) / 10.0f;
        float ll = (float)((i % 37u) + 1u) / 10.0f;
        float rl = (float)((i % 23u) + 1u) / 10.0f;
        assert(allocateWheels(a, y, ll, rl, w));
        assert(std::fabs(w.left_nm) <= ll + 0.0001f);
        assert(std::fabs(w.right_nm) <= rl + 0.0001f);
        NEAR(w.left_nm + w.right_nm, 2.0f * w.base_each_nm);
        assert(std::fabs(w.base_each_nm) <= std::fmin(ll, rl) + 0.0001f);
    }
}

static void test_lqr_no_unverified_gains(void)
{
    LqrConfig c {};
    LqrState state {};
    LqrState target {};
    LqrTorque output = {123.0f, 456.0f};

    c.gain[LqrWheelTotal][LqrTheta] = 2.0f;
    c.gain[LqrHipTotal][LqrPosition] = 3.0f;
    c.max_position_error_m = 0.2f;
    state.theta_rad = 0.1f;
    state.position_m = 1.0f;

    assert(!computeLqr(c, state, target, output));
    NEAR(output.wheel_total_nm, 0.0f);
    NEAR(output.hip_total_nm, 0.0f);

    /* 数学的な試験係数。実機K_300を意味しない。 */
    c.verified = true;
    assert(computeLqr(c, state, target, output));
    NEAR(output.wheel_total_nm, -0.2f);
    NEAR(output.hip_total_nm, -0.6f);

    state.theta_rad = NAN;
    assert(!computeLqr(c, state, target, output));
    NEAR(output.wheel_total_nm, 0.0f);
    NEAR(output.hip_total_nm, 0.0f);
}


static void test_lqr_named_state_mapping(void)
{
    LqrConfig c {};
    LqrState state {};
    LqrState target {};
    LqrTorque result {};

    /* 各フィールドが別の列へ正しく渡ることを独立した係数で検証。 */
    c.gain[LqrWheelTotal][LqrTheta] = 1.0f;
    c.gain[LqrWheelTotal][LqrThetaRate] = 2.0f;
    c.gain[LqrWheelTotal][LqrPosition] = 3.0f;
    c.gain[LqrWheelTotal][LqrVelocity] = 4.0f;
    c.gain[LqrWheelTotal][LqrPitch] = 5.0f;
    c.gain[LqrWheelTotal][LqrPitchRate] = 6.0f;
    c.gain[LqrHipTotal][LqrPitch] = 7.0f;
    c.max_position_error_m = 1.0f;
    c.verified = true;

    state.theta_rad = 0.1f;
    state.theta_rate_rad_s = 0.2f;
    state.position_m = 0.3f;
    state.velocity_m_s = 0.4f;
    state.pitch_rad = 0.5f;
    state.pitch_rate_rad_s = 0.6f;

    assert(computeLqr(c, state, target, result));
    NEAR(result.wheel_total_nm, -9.1f);
    NEAR(result.hip_total_nm, -3.5f);

    /* 目標状態の位置だけを変えても、対応する列だけに反映される。 */
    target.position_m = 0.1f;
    assert(computeLqr(c, state, target, result));
    NEAR(result.wheel_total_nm, -8.8f);
    NEAR(result.hip_total_nm, -3.5f);
}

int main()
{
    test_wheel_allocation();
    test_lqr_no_unverified_gains();
    test_lqr_named_state_mapping();
    LqrConfig c{};
    c.verified=true;c.max_position_error_m=1;
    LqrState x{},r{};
    x.theta_rad=2;
    c.gain[LqrWheelTotal][LqrTheta]=1;
    c.gain[LqrHipTotal][LqrTheta]=std::numeric_limits<float>::max();
    LqrTorque t{3,4};
    assert(!computeLqr(c,x,r,t));
    assert(t.wheel_total_nm==0 && t.hip_total_nm==0);
    c.gain[LqrHipTotal][LqrTheta]=NAN;
    assert(!computeLqr(c,x,r,t));
    assert(t.wheel_total_nm==0 && t.hip_total_nm==0);
    c.gain[LqrHipTotal][LqrTheta]=1;
    x.theta_rad=std::numeric_limits<float>::max();
    r.theta_rad=-x.theta_rad;
    assert(!computeLqr(c,x,r,t));
    puts("PASS: wheel allocation and named LQR math");
    return 0;
}
