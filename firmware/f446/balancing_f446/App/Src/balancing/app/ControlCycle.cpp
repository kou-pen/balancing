#include "balancing/app/ControlCycle.hpp"

#include <chrono>
#include <limits>
#include <variant>

namespace balancing {

ControlCycle::ControlCycle(StateEstimator& estimator, RobotSequence& sequence,
                           BalanceControl& control, MotorCommand& converter) noexcept
    : estimator_{estimator}, sequence_{sequence}, control_{control}, converter_{converter}
{
}

ControlTickResult ControlCycle::step(const ControlTickInput& input) noexcept
{
    using std::chrono::microseconds;
    const auto nowUs = input.observations.motors.sampledAtUs;
    auto observed = input.observedFaults;

    // 初回は上位が実測した時間差だけを使う。TIM7カウントから
    // 2msだったと仮定すると、割り込み滞留によるジッタが隠れる。
    microseconds elapsed = input.firstElapsed;
    if (previousUs_) {
        if (nowUs < *previousUs_) {
            elapsed = microseconds{0};
            observed = observed | FaultFlags::ClockRegression;
        } else if (nowUs == *previousUs_) {
            elapsed = microseconds{0};
            observed = observed | FaultFlags::ControlTiming;
        } else {
            const auto delta = nowUs - *previousUs_;
            // 旧C11の保護条件は「BALANCING中の6ms以上の更新停止」。
            // DISARMEDの保守停止や起動待ちで新しいFAULTを作らない。
            if (sequence_.state() == RobotState::Balancing && delta >= 6000u) {
                observed = observed | FaultFlags::ControlTiming;
            }
            if (delta > static_cast<std::uint64_t>(
                             std::numeric_limits<microseconds::rep>::max())) {
                elapsed = microseconds{0};
                observed = observed | FaultFlags::ControlTiming;
            } else {
                elapsed = microseconds{static_cast<microseconds::rep>(delta)};
            }
        }
    }

    // 時計逆行時に基準時刻を巻き戻すと、以後のサンプルの旧/新が入れ替わる。
    if (!previousUs_ || nowUs > *previousUs_) {
        previousUs_ = nowUs;
    }

    ControlTickResult result{};
    const auto estimated = estimator_.update(input.observations, elapsed);
    if (const auto* snapshot = std::get_if<ControlSnapshot>(&estimated)) {
        result.estimate = *snapshot;
    } else {
        result.estimateError = std::get<StateEstimateError>(estimated);
    }

    RobotSequenceInput decision{};
    decision.operations = input.operations;
    decision.rs06 = input.armFeedback;
    decision.interlocks = input.interlocks;
    decision.observedFaults = observed;

    // 校正未確認・1軸欠損などで推定器が値を返さないなら、
    // 外部がreadyを誤設定しても現在の周期でARMを受理しない。
    const bool hasEstimate = result.estimate.has_value();
    decision.interlocks.imuReady &= hasEstimate;
    decision.interlocks.motorsReady &= hasEstimate;
    decision.interlocks.driveLimitsVerified &= converter_.hasVerifiedConfiguration();

    const auto before = sequence_.state();
    if (before == RobotState::Disarmed && input.operations.arm && hasEstimate) {
        // ARM直前の現左右脚長を1回捕捉する（MOTOR-159）。
        // モータが有効化された後に目標を上書きしてはならない。
        if (!control_.initialize(*result.estimate)) {
            decision.interlocks.modelVerified = false;
        }
    }

    std::optional<BalanceControlOutput> torques{};
    std::optional<MotorCommandPreview> preview{};
    // E-STOPなど既知故障、DISARM指示があれば計算候補を作る必要がない。
    // FAULT原因へ無関係な数値演算失敗を追加しないため、状態機械へ直行する。
    if (before == RobotState::Balancing && hasEstimate &&
        observed == FaultFlags::None && !input.operations.disarm) {
        // RobotSequenceを一周期に2回更新しないよう、数値計算の成否を
        // 先に得て同周期のobservedFaultsに合成する。MotorCommandは送信しない。
        torques = control_.compute(*result.estimate, input.target);
        if (torques) {
            const auto candidate = converter_.evaluate(*torques);
            if (candidate.status == CommandStatus::Ready ||
                candidate.status == CommandStatus::Saturated) {
                preview = candidate;
            }
        }
        if (!torques || !preview) {
            decision.observedFaults = decision.observedFaults | FaultFlags::ExternalFault;
        }
    }

    result.sequence = sequence_.update(decision, nowUs);
    result.observedFaults = decision.observedFaults;
    if (result.sequence.desiredMotorState == DesiredMotorState::DriveAllowed) {
        // コマンドをハードウェアへ公開せず、fixtureOnlyの数学値としてだけ返す。
        result.mathematicalTorques = torques;
        result.motorCommandPreview = preview;
    } else if (result.sequence.state == RobotState::Disarmed ||
               result.sequence.state == RobotState::Fault) {
        // 非駆動状態の古いYaw積分・前回ARMの脚長目標を残さない。
        control_.reset();
    }
    return result;
}

} // namespace balancing
