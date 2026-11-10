#include "balancing/control/estimator/balancing_leg_kinematics.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f

/* These are artificial lengths for unit tests. NOT dimensions of the robot. */
static BLegFixtureGeometry fixture(void)
{
    const BLegFixtureGeometry g = {
        .base_span_m = 0.2f, .left_crank_m = 0.15f, .right_crank_m = 0.15f,
        .left_rod_m = 0.25f, .right_rod_m = 0.25f,
        .min_link_cross_sine = 0.02f, .min_leg_length_m = 0.01f
    };
    return g;
}

static BLegJointState example_joint(void)
{
    const BLegJointState q = {{-1.25f, -1.85f}, {0.7f, -0.3f}};
    return q;
}

static void near(float observed, float expected, float tolerance)
{
    if (!(isfinite(observed) && isfinite(expected) &&
          fabsf(observed - expected) <= tolerance)) {
        fprintf(stderr, "expected %.9f ~= %.9f (tol %.9f)\n",
                (double)observed, (double)expected, (double)tolerance);
        assert(0);
    }
}

static BLegKinematics solve(BLegFixtureGeometry g, BLegJointState q,
                             BLegAssemblyBranch branch)
{
    BLegKinematics state = {0};
    assert(b_leg_fixture_forward(&g, branch, &q, &state) == B_LEG_OK);
    return state;
}

/* 左右の符号と16:24の理想歯数比を数値上で分離する。
 * モータ側と機構側の瞬時仕事率が一致することも確認するが、
 * 実機の伝達効率・エンコーダ原点は未校正である。 */
static void test_motor_mapping(void)
{
    const BLegMotorMap map = {
        .motor_zero_rad = {0.3f, -0.2f},
        .joint_offset_rad = {0.2f, -0.4f},
        .sign = {1.0f, -1.0f},
        .driving_teeth = 16.0f, .driven_teeth = 24.0f,
        .calibration_valid = true
    };
    const float a[2] = {1.3f, 0.8f};
    const float omega[2] = {3.0f, -6.0f};
    BLegJointState output = {0};
    assert(b_leg_motor_to_joint(&map, a, omega, &output));
    near(output.q_rad[0], 0.2f + 2.0f / 3.0f, 2e-6f);
    near(output.q_rad[1], -0.4f - 2.0f / 3.0f, 2e-6f);
    near(output.qdot_rad_s[0], 2.0f, 1e-6f);
    near(output.qdot_rad_s[1], 4.0f, 1e-6f);

    BLegTorquePair joint = {{3.0f, -6.0f}};
    BLegTorquePair motor = {{0}};
    assert(b_leg_joint_to_ideal_motor_torque(&map, &joint, &motor));
    near(motor.torque_nm[0], 2.0f, 1e-6f);
    near(motor.torque_nm[1], 4.0f, 1e-6f);
    /* Ideal power is invariant under the chain ratio/sign transform. */
    near(motor.torque_nm[0] * omega[0] + motor.torque_nm[1] * omega[1],
         joint.torque_nm[0] * output.qdot_rad_s[0] +
         joint.torque_nm[1] * output.qdot_rad_s[1], 1e-5f);
    BLegMotorMap not_calibrated = map;
    not_calibrated.calibration_valid = false;
    assert(!b_leg_motor_to_joint(&not_calibrated, a, omega, &output));
    near(output.q_rad[0], 0.0f, 0.0f);
    assert(!b_leg_joint_to_ideal_motor_torque(&not_calibrated, &joint, &motor));
    near(motor.torque_nm[1], 0.0f, 0.0f);
}

/* 同じ入力角に対し円交点の組立分岐が2つ存在することを検証する。
 * 分岐の自動ジャンプを許さず、仮想脚長・下向き0の角度を数値照合する。 */
static void test_forward_and_branch(void)
{
    BLegFixtureGeometry g = fixture();
    BLegJointState q = {{-PI / 2.0f, -PI / 2.0f}, {0.0f, 0.0f}};
    const BLegKinematics lower = solve(g, q, B_LEG_BRANCH_NEGATIVE);
    const BLegKinematics upper = solve(g, q, B_LEG_BRANCH_POSITIVE);
    /* The two-circle branches are intentionally distinct. */
    near(lower.foot_x_m, 0.0f, 1e-6f);
    assert(lower.foot_y_m < -0.3f);
    assert(upper.foot_y_m > 0.0f);
    near(lower.length_m, fabsf(lower.foot_y_m), 1e-5f);
    near(lower.theta_rad, 0.0f, 1e-5f);
    near(lower.length_rate_m_s, 0.0f, 1e-6f);

    BLegKinematics invalid = lower;
    assert(b_leg_fixture_forward(&g, (BLegAssemblyBranch)0, &q, &invalid) ==
           B_LEG_INVALID_INPUT);
    near(invalid.length_m, 0.0f, 0.0f);
}

/* 解析微分式に依存しない中央差分でJの各列を検算する。
 * 差分幅は数学fixtureの検証用で、運転時の数値微分には使わない。 */
static void test_jacobian_against_central_difference(void)
{
    const float delta = 0.0005f;
    const BLegFixtureGeometry g = fixture();
    BLegJointState q = example_joint();
    for (int branch = -1; branch <= 1; branch += 2) {
        const BLegAssemblyBranch which = (BLegAssemblyBranch)branch;
        BLegKinematics nominal = solve(g, q, which);
        float derived_length_rate = 0.0f;
        float derived_theta_rate = 0.0f;
        for (unsigned col = 0; col < 2; ++col) {
            BLegJointState plus = q;
            BLegJointState minus = q;
            plus.q_rad[col] += delta;
            minus.q_rad[col] -= delta;
            const BLegKinematics hi = solve(g, plus, which);
            const BLegKinematics lo = solve(g, minus, which);
            near(nominal.jacobian.dlength_dq_m[col],
                 (hi.length_m - lo.length_m) / (2.0f * delta), 1e-4f);
            const float angle_delta = remainderf(hi.theta_rad - lo.theta_rad, 2.0f * PI);
            near(nominal.jacobian.dtheta_dq[col],
                 angle_delta / (2.0f * delta), 3e-4f);
            derived_length_rate += nominal.jacobian.dlength_dq_m[col] * q.qdot_rad_s[col];
            derived_theta_rate += nominal.jacobian.dtheta_dq[col] * q.qdot_rad_s[col];
        }
        near(nominal.length_rate_m_s, derived_length_rate, 1e-6f);
        near(nominal.theta_rate_rad_s, derived_theta_rate, 1e-6f);
    }
}

/* 左右入力軸の反射・交換で脚のxとθの符号が反転する不変条件を検証。
 * また±2πずらしても同じ物理幾何へ写像されることを確認する。 */
static void test_reflection_and_angle_periodicity(void)
{
    const BLegFixtureGeometry g = fixture();
    const BLegJointState q = example_joint();
    const BLegKinematics a = solve(g, q, B_LEG_BRANCH_NEGATIVE);
    /* Reflect and swap the joints; oriented elbow order preserves branch sign. */
    BLegJointState mirrored = {{PI - q.q_rad[1], PI - q.q_rad[0]},
                               {-q.qdot_rad_s[1], -q.qdot_rad_s[0]}};
    const BLegKinematics b = solve(g, mirrored, B_LEG_BRANCH_NEGATIVE);
    near(a.foot_x_m, -b.foot_x_m, 2e-6f);
    near(a.foot_y_m, b.foot_y_m, 2e-6f);
    near(a.length_m, b.length_m, 2e-6f);
    near(a.theta_rad, -b.theta_rad, 2e-5f);
    near(a.length_rate_m_s, b.length_rate_m_s, 2e-5f);
    near(a.theta_rate_rad_s, -b.theta_rate_rad_s, 2e-5f);

    BLegJointState periodic = q;
    periodic.q_rad[0] += 2.0f * PI;
    periodic.q_rad[1] -= 4.0f * PI;
    const BLegKinematics c = solve(g, periodic, B_LEG_BRANCH_NEGATIVE);
    near(a.length_m, c.length_m, 2e-6f);
    near(a.theta_rad, c.theta_rad, 2e-5f);
}

/* J^Tの実装と仮想仕事率の保存を別々の計算経路から照合する。
 * これは理想伝達のみの数学検証であり、実モータのトルク指令ではない。 */
static void test_virtual_work_and_torque_mapping(void)
{
    const BLegKinematics state =
        solve(fixture(), example_joint(), B_LEG_BRANCH_NEGATIVE);
    const BLegVirtualLoad load = {12.0f, -0.5f};
    BLegTorquePair torque = {0};
    assert(b_leg_virtual_to_joint_torque(&state, &load, &torque));
    for (unsigned i = 0; i < 2; ++i) {
        near(torque.torque_nm[i],
             load.axial_force_n * state.jacobian.dlength_dq_m[i] +
             load.pitch_torque_nm * state.jacobian.dtheta_dq[i], 1e-5f);
    }
    /* Generalized power = joint power for arbitrary nonzero qdot. */
    const BLegJointState q = example_joint();
    near(torque.torque_nm[0] * q.qdot_rad_s[0] +
         torque.torque_nm[1] * q.qdot_rad_s[1],
         load.axial_force_n * state.length_rate_m_s +
         load.pitch_torque_nm * state.theta_rate_rad_s, 1e-5f);
}

static void test_unreachable_singular_and_invalid(void)
{
    BLegFixtureGeometry g = fixture();
    const BLegJointState q = example_joint();
    BLegKinematics state = {0};
    g.left_rod_m = 0.02f;
    g.right_rod_m = 0.02f;
    assert(b_leg_fixture_forward(&g, B_LEG_BRANCH_NEGATIVE, &q, &state) ==
           B_LEG_UNREACHABLE);
    near(state.length_m, 0.0f, 0.0f);

    /* Exactly tangent rods: derivative of intersection is singular. */
    g.base_span_m = 0.2f;
    g.left_crank_m = 0.1f;
    g.right_crank_m = 0.1f;
    g.left_rod_m = 0.1f;
    g.right_rod_m = 0.1f;
    const BLegJointState tangent = {{-PI / 2.0f, -PI / 2.0f}, {0.0f, 0.0f}};
    assert(b_leg_fixture_forward(&g, B_LEG_BRANCH_NEGATIVE, &tangent, &state) ==
           B_LEG_SINGULAR);
    near(state.theta_rad, 0.0f, 0.0f);

    g = fixture();
    g.min_link_cross_sine = 0.95f; /* Fail closed at an artificial near-singularity. */
    const BLegStatus high_limit = b_leg_fixture_forward(
        &g, B_LEG_BRANCH_NEGATIVE, &q, &state);
    assert(high_limit == B_LEG_SINGULAR);

    g = fixture();
    g.base_span_m = NAN;
    assert(b_leg_fixture_forward(&g, B_LEG_BRANCH_NEGATIVE, &q, &state) ==
           B_LEG_INVALID_INPUT);
    g = fixture();
    BLegJointState nonfinite = q;
    nonfinite.qdot_rad_s[0] = INFINITY;
    assert(b_leg_fixture_forward(&g, B_LEG_BRANCH_NEGATIVE, &nonfinite, &state) ==
           B_LEG_INVALID_INPUT);
    const BLegVirtualLoad invalid_load = {NAN, 0.0f};
    BLegTorquePair torques = {{1.0f, 1.0f}};
    assert(!b_leg_virtual_to_joint_torque(&state, &invalid_load, &torques));
    near(torques.torque_nm[0], 0.0f, 0.0f);
}

/* 人工的な13×13姿勢格子の両組立分岐を走査し、
 * 有効な点について中央差分と解析Jの一致を確認する。
 * CAD由来の可動域や実機特異姿勢の安全範囲を主張するものではない。 */
static void test_jacobian_grid_and_extreme_inputs(void)
{
    const BLegFixtureGeometry g = fixture();
    unsigned checked = 0;
    const float delta = 0.001f;
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < 13; ++j) {
            BLegJointState q = {{-2.2f + 0.13f * (float)i,
                                 -2.2f + 0.13f * (float)j}, {0.0f, 0.0f}};
            for (int branch = -1; branch <= 1; branch += 2) {
                BLegKinematics state = {0};
                const BLegAssemblyBranch which = (BLegAssemblyBranch)branch;
                if (b_leg_fixture_forward(&g, which, &q, &state) != B_LEG_OK) {
                    continue;
                }
                for (unsigned axis = 0; axis < 2; ++axis) {
                    BLegJointState plus = q;
                    BLegJointState minus = q;
                    plus.q_rad[axis] += delta;
                    minus.q_rad[axis] -= delta;
                    BLegKinematics hi = {0};
                    BLegKinematics lo = {0};
                    if (b_leg_fixture_forward(&g, which, &plus, &hi) != B_LEG_OK ||
                        b_leg_fixture_forward(&g, which, &minus, &lo) != B_LEG_OK) {
                        continue;
                    }
                    near(state.jacobian.dlength_dq_m[axis],
                         (hi.length_m - lo.length_m) / (2.0f * delta), 0.002f);
                    near(state.jacobian.dtheta_dq[axis],
                         remainderf(hi.theta_rad - lo.theta_rad, 2.0f * PI) /
                         (2.0f * delta), 0.003f);
                    ++checked;
                }
            }
        }
    }
    assert(checked > 100u);

    BLegFixtureGeometry limited = fixture();
    limited.min_leg_length_m = 1.0f;
    BLegKinematics invalid = {0};
    const BLegJointState q = example_joint();
    assert(b_leg_fixture_forward(&limited, B_LEG_BRANCH_NEGATIVE, &q, &invalid) ==
           B_LEG_SINGULAR);
    near(invalid.length_m, 0.0f, 0.0f);

    const BLegMotorMap map = {
        .motor_zero_rad = {0.0f, 0.0f},
        .joint_offset_rad = {0.0f, 0.0f},
        .sign = {1.0f, -1.0f},
        .driving_teeth = 16.0f, .driven_teeth = 24.0f,
        .calibration_valid = true
    };
    const float angles[2] = {NAN, 0.0f};
    const float rates[2] = {1.0f, 2.0f};
    BLegJointState joint = {0};
    assert(!b_leg_motor_to_joint(&map, angles, rates, &joint));
    near(joint.q_rad[0], 0.0f, 0.0f);
    BLegMotorMap bad_map = map;
    bad_map.sign[1] = 0.5f;
    const float finite_angles[2] = {0.0f, 0.0f};
    assert(!b_leg_motor_to_joint(&bad_map, finite_angles, rates, &joint));
}

int main(void)
{
    test_motor_mapping();
    test_forward_and_branch();
    test_jacobian_against_central_difference();
    test_reflection_and_angle_periodicity();
    test_virtual_work_and_torque_mapping();
    test_unreachable_singular_and_invalid();
    test_jacobian_grid_and_extreme_inputs();
    puts("PASS: mathematical closed-chain fixture, Jacobian, mirror, limits and virtual work");
    return 0;
}
