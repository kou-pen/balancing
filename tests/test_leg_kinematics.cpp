#include "balancing/control/estimator/LegKinematics.hpp"

#include <assert.h>
#include <cmath>
#include <limits>
using namespace balancing;
#include <stdio.h>

#define PI 3.14159265358979323846f

/* These are artificial lengths for unit tests. NOT dimensions of the robot. */
static LegFixtureGeometry fixture(void)
{
    const LegFixtureGeometry g = {
         0.2f,  0.15f,  0.15f,
         0.25f,  0.25f,
         0.02f,  0.01f
    };
    return g;
}

static LegJointState example_joint(void)
{
    const LegJointState q = {{-1.25f, -1.85f}, {0.7f, -0.3f}};
    return q;
}

static void near(float observed, float expected, float tolerance)
{
    if (!(std::isfinite(observed) && std::isfinite(expected) &&
          std::fabs(observed - expected) <= tolerance)) {
        fprintf(stderr, "expected %.9f ~= %.9f (tol %.9f)\n",
                (double)observed, (double)expected, (double)tolerance);
        assert(0);
    }
}

static LegSolveError forward(LegFixtureGeometry g,int branch,const LegJointState& q,LegEstimate& out)
{
    g.branch=branch;
    return LegKinematics::forward(g,q,out);
}

static LegEstimate solve(LegFixtureGeometry g, LegJointState q,
                             int branch)
{
    LegEstimate state{};
    assert(forward(g,branch,q,state) == LegSolveError::None);
    return state;
}

/* 左右の符号と16:24の理想歯数比を数値上で分離する。
 * モータ側と機構側の瞬時仕事率が一致することも確認するが、
 * 実機の伝達効率・エンコーダ原点は未校正である。 */
static void test_motor_mapping(void)
{
    const LegMotorTransform map = {
         {0.3f, -0.2f},
         {0.2f, -0.4f},
         {1.0f, -1.0f},
         16.0f,  24.0f,
         true
    };
    const std::array<float,2> a = {1.3f, 0.8f};
    const std::array<float,2> omega = {3.0f, -6.0f};
    LegJointState output{};
    assert(LegKinematics::motorToJoint(map,a,omega,output));
    near(output.angleRad[0], 0.2f + 2.0f / 3.0f, 2e-6f);
    near(output.angleRad[1], -0.4f - 2.0f / 3.0f, 2e-6f);
    near(output.velocityRadS[0], 2.0f, 1e-6f);
    near(output.velocityRadS[1], 4.0f, 1e-6f);

    std::array<float,2> joint = {{3.0f, -6.0f}};
    std::array<float,2> motor{};
    assert(LegKinematics::jointToIdealMotorTorque(map,joint,motor));
    near(motor[0], 2.0f, 1e-6f);
    near(motor[1], 4.0f, 1e-6f);
    /* Ideal power is invariant under the chain ratio/sign transform. */
    near(motor[0] * omega[0] + motor[1] * omega[1],
         joint[0] * output.velocityRadS[0] +
         joint[1] * output.velocityRadS[1], 1e-5f);
    LegMotorTransform not_calibrated = map;
    not_calibrated.calibrated = false;
    assert(!LegKinematics::motorToJoint(not_calibrated,a,omega,output));
    near(output.angleRad[0], 0.0f, 0.0f);
    assert(!LegKinematics::jointToIdealMotorTorque(not_calibrated,joint,motor));
    near(motor[1], 0.0f, 0.0f);
}

/* 同じ入力角に対し円交点の組立分岐が2つ存在することを検証する。
 * 分岐の自動ジャンプを許さず、仮想脚長・下向き0の角度を数値照合する。 */
static void test_forward_and_branch(void)
{
    LegFixtureGeometry g = fixture();
    LegJointState q = {{-PI / 2.0f, -PI / 2.0f}, {0.0f, 0.0f}};
    const LegEstimate lower = solve(g, q, -1);
    const LegEstimate upper = solve(g, q, 1);
    /* The two-circle branches are intentionally distinct. */
    near(lower.footXM, 0.0f, 1e-6f);
    assert(lower.footYM < -0.3f);
    assert(upper.footYM > 0.0f);
    near(lower.lengthM, std::fabs(lower.footYM), 1e-5f);
    near(lower.thetaBodyRad, 0.0f, 1e-5f);
    near(lower.lengthRateMps, 0.0f, 1e-6f);

    LegEstimate invalid = lower;
    assert(forward(g,(int)0,q,invalid) ==
           LegSolveError::InvalidGeometry);
    near(invalid.lengthM, 0.0f, 0.0f);
}

/* 解析微分式に依存しない中央差分でJの各列を検算する。
 * 差分幅は数学fixtureの検証用で、運転時の数値微分には使わない。 */
static void test_jacobian_against_central_difference(void)
{
    const float delta = 0.0005f;
    const LegFixtureGeometry g = fixture();
    LegJointState q = example_joint();
    for (int branch = -1; branch <= 1; branch += 2) {
        const int which = (int)branch;
        LegEstimate nominal = solve(g, q, which);
        float derived_length_rate = 0.0f;
        float derived_theta_rate = 0.0f;
        for (unsigned col = 0; col < 2; ++col) {
            LegJointState plus = q;
            LegJointState minus = q;
            plus.angleRad[col] += delta;
            minus.angleRad[col] -= delta;
            const LegEstimate hi = solve(g, plus, which);
            const LegEstimate lo = solve(g, minus, which);
            near(nominal.lengthJacobianM[col],
                 (hi.lengthM - lo.lengthM) / (2.0f * delta), 1e-4f);
            const float angle_delta = std::remainder(hi.thetaBodyRad - lo.thetaBodyRad, 2.0f * PI);
            near(nominal.thetaJacobian[col],
                 angle_delta / (2.0f * delta), 3e-4f);
            derived_length_rate += nominal.lengthJacobianM[col] * q.velocityRadS[col];
            derived_theta_rate += nominal.thetaJacobian[col] * q.velocityRadS[col];
        }
        near(nominal.lengthRateMps, derived_length_rate, 1e-6f);
        near(nominal.thetaRateRadS, derived_theta_rate, 1e-6f);
    }
}

/* 左右入力軸の反射・交換で脚のxとθの符号が反転する不変条件を検証。
 * また±2πずらしても同じ物理幾何へ写像されることを確認する。 */
static void test_reflection_and_angle_periodicity(void)
{
    const LegFixtureGeometry g = fixture();
    const LegJointState q = example_joint();
    const LegEstimate a = solve(g, q, -1);
    /* Reflect and swap the joints; oriented elbow order preserves branch sign. */
    LegJointState mirrored = {{PI - q.angleRad[1], PI - q.angleRad[0]},
                               {-q.velocityRadS[1], -q.velocityRadS[0]}};
    const LegEstimate b = solve(g, mirrored, -1);
    near(a.footXM, -b.footXM, 2e-6f);
    near(a.footYM, b.footYM, 2e-6f);
    near(a.lengthM, b.lengthM, 2e-6f);
    near(a.thetaBodyRad, -b.thetaBodyRad, 2e-5f);
    near(a.lengthRateMps, b.lengthRateMps, 2e-5f);
    near(a.thetaRateRadS, -b.thetaRateRadS, 2e-5f);

    LegJointState periodic = q;
    periodic.angleRad[0] += 2.0f * PI;
    periodic.angleRad[1] -= 4.0f * PI;
    const LegEstimate c = solve(g, periodic, -1);
    near(a.lengthM, c.lengthM, 2e-6f);
    near(a.thetaBodyRad, c.thetaBodyRad, 2e-5f);
}

/* J^Tの実装と仮想仕事率の保存を別々の計算経路から照合する。
 * これは理想伝達のみの数学検証であり、実モータのトルク指令ではない。 */
static void test_virtual_work_and_torque_mapping(void)
{
    const LegEstimate state =
        solve(fixture(), example_joint(), -1);
    const float force=12.0f,pitch=-0.5f;
    std::array<float,2> torque{};
    assert(LegKinematics::virtualToJointTorque(state,force,pitch,torque));
    for (unsigned i = 0; i < 2; ++i) {
        near(torque[i],
             force * state.lengthJacobianM[i] +
             pitch * state.thetaJacobian[i], 1e-5f);
    }
    /* Generalized power = joint power for arbitrary nonzero qdot. */
    const LegJointState q = example_joint();
    near(torque[0] * q.velocityRadS[0] +
         torque[1] * q.velocityRadS[1],
         force * state.lengthRateMps +
         pitch * state.thetaRateRadS, 1e-5f);
}

static void test_unreachable_singular_and_invalid(void)
{
    LegFixtureGeometry g = fixture();
    const LegJointState q = example_joint();
    LegEstimate state{};
    g.firstRodM = 0.02f;
    g.secondRodM = 0.02f;
    assert(forward(g,-1,q,state) ==
           LegSolveError::Unreachable);
    near(state.lengthM, 0.0f, 0.0f);

    /* Exactly tangent rods: derivative of intersection is singular. */
    g.baseSpanM = 0.2f;
    g.firstCrankM = 0.1f;
    g.secondCrankM = 0.1f;
    g.firstRodM = 0.1f;
    g.secondRodM = 0.1f;
    const LegJointState tangent = {{-PI / 2.0f, -PI / 2.0f}, {0.0f, 0.0f}};
    assert(forward(g,-1,tangent,state) ==
           LegSolveError::Singular);
    near(state.thetaBodyRad, 0.0f, 0.0f);

    g = fixture();
    g.minCrossSine = 0.95f; /* Fail closed at an artificial near-singularity. */
    const LegSolveError high_limit = forward(g,-1,q,state);
    assert(high_limit == LegSolveError::Singular);

    g = fixture();
    g.baseSpanM = NAN;
    assert(forward(g,-1,q,state) ==
           LegSolveError::InvalidGeometry);
    g = fixture();
    LegJointState nonfinite = q;
    nonfinite.velocityRadS[0] = INFINITY;
    assert(forward(g,-1,nonfinite,state) ==
           LegSolveError::InvalidGeometry);

    std::array<float,2> torques = {{1.0f, 1.0f}};
    assert(!LegKinematics::virtualToJointTorque(state,NAN,0,torques));
    near(torques[0], 0.0f, 0.0f);
}

/* 人工的な13×13姿勢格子の両組立分岐を走査し、
 * 有効な点について中央差分と解析Jの一致を確認する。
 * CAD由来の可動域や実機特異姿勢の安全範囲を主張するものではない。 */
static void test_jacobian_grid_and_extreme_inputs(void)
{
    const LegFixtureGeometry g = fixture();
    unsigned checked = 0;
    const float delta = 0.001f;
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < 13; ++j) {
            LegJointState q = {{-2.2f + 0.13f * (float)i,
                                 -2.2f + 0.13f * (float)j}, {0.0f, 0.0f}};
            for (int branch = -1; branch <= 1; branch += 2) {
                LegEstimate state{};
                const int which = (int)branch;
                if (forward(g,which,q,state) != LegSolveError::None) {
                    continue;
                }
                for (unsigned axis = 0; axis < 2; ++axis) {
                    LegJointState plus = q;
                    LegJointState minus = q;
                    plus.angleRad[axis] += delta;
                    minus.angleRad[axis] -= delta;
                    LegEstimate hi{};
                    LegEstimate lo{};
                    if (forward(g,which,plus,hi) != LegSolveError::None ||
                        forward(g,which,minus,lo) != LegSolveError::None) {
                        continue;
                    }
                    near(state.lengthJacobianM[axis],
                         (hi.lengthM - lo.lengthM) / (2.0f * delta), 0.002f);
                    near(state.thetaJacobian[axis],
                         std::remainder(hi.thetaBodyRad - lo.thetaBodyRad, 2.0f * PI) /
                         (2.0f * delta), 0.003f);
                    ++checked;
                }
            }
        }
    }
    assert(checked > 100u);

    LegFixtureGeometry limited = fixture();
    limited.minLegLengthM = 1.0f;
    LegEstimate invalid{};
    const LegJointState q = example_joint();
    assert(forward(limited,-1,q,invalid) ==
           LegSolveError::Singular);
    near(invalid.lengthM, 0.0f, 0.0f);

    const LegMotorTransform map = {
         {0.0f, 0.0f},
         {0.0f, 0.0f},
         {1.0f, -1.0f},
         16.0f,  24.0f,
         true
    };
    const std::array<float,2> angles = {NAN, 0.0f};
    const std::array<float,2> rates = {1.0f, 2.0f};
    LegJointState joint{};
    assert(!LegKinematics::motorToJoint(map,angles,rates,joint));
    near(joint.angleRad[0], 0.0f, 0.0f);
    LegMotorTransform bad_map = map;
    bad_map.direction[1] = 0.5f;
    const std::array<float,2> finite_angles = {0.0f, 0.0f};
    assert(!LegKinematics::motorToJoint(bad_map,finite_angles,rates,joint));
}

int main()
{
    test_motor_mapping();
    test_forward_and_branch();
    test_jacobian_against_central_difference();
    test_reflection_and_angle_periodicity();
    test_virtual_work_and_torque_mapping();
    test_unreachable_singular_and_invalid();
    test_jacobian_grid_and_extreme_inputs();
    LegEstimate state{};
    state.lengthJacobianM={1,std::numeric_limits<float>::max()};
    std::array<float,2> torque{1,2};
    assert(!LegKinematics::virtualToJointTorque(state,2,0,torque));
    assert(torque[0]==0 && torque[1]==0);
    LegKinematicsConfig cfg{};
    cfg.fixture=fixture();
    cfg.fixture.branch=-1;
    assert(LegKinematics::solve(cfg,{0,0},{0,0}).error==LegSolveError::InvalidCalibration);
    puts("PASS: mathematical closed-chain fixture, Jacobian, mirror, limits and virtual work");
    return 0;
}
