#include "balancing/app/RobotSequence.hpp"

#include <algorithm>
#include <limits>

namespace balancing {
namespace {

constexpr bool hasFlag(FaultFlags flags, FaultFlags flag) noexcept
{
    return (static_cast<std::uint16_t>(flags) & static_cast<std::uint16_t>(flag)) != 0;
}

FaultFlags primaryCause(FaultFlags observed) noexcept
{
    // E-STOPを最優先し、確定故障、保護条件・時刻異常、期限到達の順で選ぶ。
    // 同じ区分に複数の異常がある場合も、毎回同じ代表原因を選ぶ。
    constexpr FaultFlags priority[] = {
        FaultFlags::EmergencyStop, FaultFlags::BusOff, FaultFlags::MotorFault,
        FaultFlags::ExternalFault, FaultFlags::EnableFailed, FaultFlags::MechanismUnsafe,
        FaultFlags::InterlockLost, FaultFlags::ClockRegression, FaultFlags::ControlTiming,
        FaultFlags::ArmTimeout,
    };
    for (const auto reason : priority) {
        if (hasFlag(observed, reason)) {
            return reason;
        }
    }
    return FaultFlags::None;
}

bool interlocksReady(const SequenceInterlocks& interlocks) noexcept
{
    // 初期姿勢の診断範囲とは別に、校正・モデル・機構の制限値の検証を要求する。
    // 現在の機構安全の判断は非ゼロ駆動へ進む直前にも確認する。
    return interlocks.imuReady && interlocks.motorsReady && interlocks.powerReady &&
           interlocks.calibrationVerified && interlocks.modelVerified &&
           interlocks.driveLimitsVerified;
}

} // namespace

RobotSequence::RobotSequence(ArmTimeoutConfig timeout) noexcept
    : StateMachine{RobotState::Disarmed}, timeout_{timeout}
{
}

RobotSequenceResult RobotSequence::update(const RobotSequenceInput& input,
                                          std::uint64_t nowUs) noexcept
{
    auto observed = input.observedFaults;
    if (previousUpdateUs_ && nowUs < *previousUpdateUs_) {
        // 逆行した時刻を基準にしない。元の時刻に追い付くまで故障が継続する。
        observed = observed | FaultFlags::ClockRegression;
    } else {
        previousUpdateUs_ = nowUs;
    }

    const bool arming = state() == RobotState::Arming;
    // 旧セッションの応答・失敗報告を現在の有効化判定へ混入させない。
    // 現在観測されたバス異常などは、セッションと独立した observedFaults で届く。
    const bool currentFeedback = arming && input.rs06.session == armSession_;
    const bool ready = interlocksReady(input.interlocks);
    const auto mechanism = input.interlocks.mechanism;
    if ((arming || state() == RobotState::Balancing) && (!ready || input.rs06.stopPending)) {
        observed = observed | FaultFlags::InterlockLost;
    }
    if ((arming && mechanism == MechanismSafety::Unsafe) ||
        (state() == RobotState::Balancing && mechanism != MechanismSafety::Safe)) {
        observed = observed | FaultFlags::MechanismUnsafe;
    }
    if (currentFeedback && std::any_of(input.rs06.axes.begin(), input.rs06.axes.end(),
                                      [](auto status) { return status == Rs06ArmStatus::Failed; })) {
        observed = observed | FaultFlags::EnableFailed;
    }
    if (arming && nowUs >= armDeadlineUs_) {
        // 期限と4軸確認が同周期でも停止する。再試行でこの期限は変更しない。
        observed = observed | FaultFlags::ArmTimeout;
    }

    // 明示解除の許可に加え、現在も判定できる発生時の保護条件の回復を確認する。
    // 通信側が報告する継続中の異常は observedFaults から毎周期入力される。
    if (currentFault_) {
        if (hasFlag(currentFault_->triggerFlags, FaultFlags::InterlockLost) &&
            (!ready || input.rs06.stopPending)) {
            observed = observed | FaultFlags::InterlockLost;
        }
        if (hasFlag(currentFault_->triggerFlags, FaultFlags::MechanismUnsafe) &&
            mechanism != MechanismSafety::Safe) {
            observed = observed | FaultFlags::MechanismUnsafe;
        }
    }
    // 先に周期の異常をすべて集める。手動DISARMや4軸確認でFAULTを上書きしない。
    if (observed != FaultFlags::None) {
        latchFault(observed, nowUs);
        return result();
    }

    switch (state()) {
    case RobotState::Disarmed:
        if (input.operations.arm && !input.operations.disarm && !input.rs06.stopPending &&
            ready && mechanism != MechanismSafety::Unsafe && timeout_.verified &&
            timeout_.durationUs != 0 &&
            timeout_.durationUs <= std::numeric_limits<std::uint64_t>::max() - nowUs &&
            armSession_ != std::numeric_limits<ArmSessionId>::max()) {
            // 受理時点だけで期限と世代を決める。送信待ち・応答待ちでも同じ期限を使う。
            armDeadlineUs_ = nowUs + timeout_.durationUs;
            ++armSession_;
            transitionTo(RobotState::Arming);
        }
        break;
    case RobotState::Arming:
        if (input.operations.disarm) {
            transitionTo(RobotState::Disarmed);
        } else if (currentFeedback && mechanism == MechanismSafety::Safe &&
                   std::all_of(input.rs06.axes.begin(), input.rs06.axes.end(), [](auto status) {
                       return status == Rs06ArmStatus::Confirmed;
                   })) {
            // モータの準備と機構安全の双方が成立した周期から駆動を許可する。
            transitionTo(RobotState::Balancing);
        }
        break;
    case RobotState::Balancing:
        if (input.operations.disarm) {
            transitionTo(RobotState::Disarmed);
        }
        break;
    case RobotState::Fault:
        if (input.operations.clearFault && input.operations.clearFaultAuthorized) {
            lastResolvedFault_ = currentFault_;
            currentFault_.reset();
            // 解除とARMが同時でも、この周期の処理はDISARMEDで終了する。
            transitionTo(RobotState::Disarmed);
        }
        break;
    }
    return result();
}

RobotSequenceResult RobotSequence::result() const noexcept
{
    switch (state()) {
    case RobotState::Arming:
        return {state(), DesiredMotorState::EnableRs06, armSession_};
    case RobotState::Balancing:
        return {state(), DesiredMotorState::DriveAllowed, armSession_};
    case RobotState::Disarmed:
    case RobotState::Fault:
        // StopAll は状態に基づく要求。軸別の停止再試行は受け取り側が管理する。
        return {state(), DesiredMotorState::StopAll, armSession_};
    }
    return {RobotState::Fault, DesiredMotorState::StopAll, armSession_};
}

void RobotSequence::latchFault(FaultFlags observed, std::uint64_t nowUs) noexcept
{
    if (!currentFault_) {
        currentFault_ = FaultRecord{primaryCause(observed), observed, FaultFlags::None, nowUs};
        transitionTo(RobotState::Fault);
        return;
    }
    // 初回に記録済みのビットは triggerFlags で表現する。継続中に初めて現れた異常を残す。
    const auto newBits = static_cast<std::uint16_t>(observed) &
                         ~static_cast<std::uint16_t>(currentFault_->triggerFlags);
    currentFault_->subsequentFlags =
        currentFault_->subsequentFlags | static_cast<FaultFlags>(newBits);
}

std::optional<FaultRecord> RobotSequence::currentFault() const noexcept
{
    return currentFault_;
}

std::optional<FaultRecord> RobotSequence::lastResolvedFault() const noexcept
{
    return lastResolvedFault_;
}

} // namespace balancing
