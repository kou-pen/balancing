#include "balancing/control/estimator/StateEstimator.hpp"

#include <cmath>

namespace balancing {
namespace {
bool finiteAttitude(const ImuAttitude& imu) noexcept
{
    return std::isfinite(imu.pitchRad) && std::isfinite(imu.rollRad) &&
           std::isfinite(imu.pitchRateRadS) && std::isfinite(imu.rollRateRadS) &&
           std::isfinite(imu.yawRateRadS);
}
} // namespace

StateEstimator::StateEstimator(const StateEstimatorConfig& config) noexcept
    : config_{config}, odometry_{config.wheels}
{
}

std::variant<ControlSnapshot, StateEstimateError> StateEstimator::update(
    const StateEstimatorInput& input, std::chrono::microseconds elapsed) noexcept
{
    const std::uint64_t nowUs = input.motors.sampledAtUs;
    if (elapsed.count() <= 0) {
        // Applicationから実測した正の周期間隔だけを採用する。
        // ここで2ms固定とすると実際のジッタを隠すため、固定値とは比較しない。
        return StateEstimateError::InvalidPeriod;
    }

    // 片方のセンサが欠ける周期も車輪の新しいCAN報告は受理しておく。
    // IMU復旧まで放置すると復旧後に複数周期のロータ変位を失うためである。
    odometry_.update(input.motors);
    const auto wheelSnapshot = odometry_.snapshot();
    const ImuAttitude& imu = input.imu;
    if (!imu.valid || !imu.fresh || imu.receivedAtUs > nowUs) {
        return StateEstimateError::ImuUnavailable;
    }
    if (!finiteAttitude(imu)) {
        return StateEstimateError::InvalidImu;
    }

    // RS06生値の入力順はMotorServiceで左A/B・右A/Bに固定されている。
    // 各軸の有効性と受信時刻は元のCAN報告で判断し、経過時間をここで再計算しない。
    std::array<LegState, 2> legs{};
    std::array<std::uint64_t, 4> rsStamps{};
    for (std::size_t leg = 0; leg < legs.size(); ++leg) {
        std::array<float, 2> angles{};
        std::array<float, 2> rates{};
        for (std::size_t joint = 0; joint < 2; ++joint) {
            const std::size_t axis = leg * 2 + joint;
            const auto& feedback = input.motors.rs06[axis];
            if (!feedback.fresh || !feedback.latest ||
                feedback.latest->receivedAtUs > nowUs) {
                return StateEstimateError::LegFeedbackUnavailable;
            }
            angles[joint] = feedback.latest->report.angleRad;
            rates[joint] = feedback.latest->report.velocityRadPerSec;
            rsStamps[axis] = feedback.latest->receivedAtUs;
        }

        const auto solved = LegKinematics::solve(config_.legs[leg], angles, rates);
        if (!solved.state) {
            return leg == 0 ? StateEstimateError::LeftLegUnsolved :
                              StateEstimateError::RightLegUnsolved;
        }
        // 数学fixtureのθは機体に固定した下向き0。実機取付姿勢の照合前に
        // この角度和を絶対鉛直姿勢や安全限界として用いてはならない。
        legs[leg] = {*solved.state, solved.state->thetaBodyRad + imu.pitchRad};
    }

    std::array<WheelState, 2> wheels{};
    std::array<std::uint64_t, 2> wheelStamps{};
    for (std::size_t axis = 0; axis < wheels.size(); ++axis) {
        const auto& feedback = wheelSnapshot.wheel[axis];
        if (!feedback.fresh || !feedback.latest) {
            return StateEstimateError::WheelFeedbackUnavailable;
        }
        wheels[axis] = {*feedback.latest};
        wheelStamps[axis] = feedback.latest->receivedAtUs;
    }

    // 両車輪の平均は旧C11 LQRの(x_b,dx_b)に相当する数学上の前後状態。
    // 車輪の正方向は起動時の左右校正設定で合わせ済みという前提であり、
    // 物理原点や実タイヤの滑りをここで推測しない。
    const ChassisState chassis{
        0.5f * (wheels[0].motion.positionM + wheels[1].motion.positionM),
        0.5f * (wheels[0].motion.velocityMps + wheels[1].motion.velocityMps)
    };
    return ControlSnapshot{
        nowUs, elapsed, imu.receivedAtUs, rsStamps, wheelStamps,
        {imu.pitchRad, imu.rollRad, imu.pitchRateRadS,
         imu.rollRateRadS, imu.yawRateRadS},
        chassis, legs, wheels, true
    };
}

void StateEstimator::reset() noexcept
{
    // 観測基準を明示的に変更したときだけ相対車輪位置の原点を再設定する。
    odometry_.reset();
}

} // namespace balancing
