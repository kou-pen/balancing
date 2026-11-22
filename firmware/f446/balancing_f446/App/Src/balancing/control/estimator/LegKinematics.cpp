#include "balancing/control/estimator/LegKinematics.hpp"
#include "balancing/control/estimator/balancing_leg_kinematics.h"

namespace balancing {

LegSolveResult LegKinematics::solve(
    const LegKinematicsConfig& config, const std::array<float, 2>& angleRad,
    const std::array<float, 2>& velocityRadS) noexcept
{
    // 実機の六節リンクとして未検証のため、C11の幾何fixtureを計算器としてのみ再利用する。
    // 校正・方向符号を付けずにモータ角を関節角として流す近道は採らない。
    const auto& m = config.motor;
    const BLegMotorMap motor{
        {m.motorZeroRad[0], m.motorZeroRad[1]},
        {m.jointOffsetRad[0], m.jointOffsetRad[1]},
        {m.direction[0], m.direction[1]},
        m.drivingTeeth, m.drivenTeeth, m.calibrated
    };
    BLegJointState joints{};
    if (!b_leg_motor_to_joint(&motor, angleRad.data(), velocityRadS.data(), &joints)) {
        return {std::nullopt, LegSolveError::InvalidCalibration};
    }

    const auto& f = config.fixture;
    const BLegFixtureGeometry geometry{
        f.baseSpanM, f.firstCrankM, f.secondCrankM,
        f.firstRodM, f.secondRodM, f.minCrossSine, f.minLegLengthM
    };
    BLegKinematics calculated{};
    // 組立分岐を勝手に近い方へ切り替えると、脚長・角度が不連続になる。
    const auto status = b_leg_fixture_forward(
        &geometry, static_cast<BLegAssemblyBranch>(f.branch), &joints, &calculated);
    if (status != B_LEG_OK) {
        const auto error = status == B_LEG_UNREACHABLE ? LegSolveError::Unreachable :
                           status == B_LEG_SINGULAR ? LegSolveError::Singular :
                           LegSolveError::InvalidGeometry;
        return {std::nullopt, error};
    }
    return {
        LegEstimate{calculated.length_m, calculated.theta_rad,
                    calculated.length_rate_m_s, calculated.theta_rate_rad_s,
                    {calculated.jacobian.dlength_dq_m[0],
                     calculated.jacobian.dlength_dq_m[1]},
                    {calculated.jacobian.dtheta_dq[0],
                     calculated.jacobian.dtheta_dq[1]}},
        LegSolveError::None
    };
}

} // namespace balancing
