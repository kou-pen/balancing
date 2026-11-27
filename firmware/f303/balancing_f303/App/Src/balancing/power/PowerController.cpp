#include "balancing/power/PowerController.hpp"

#include <cmath>

namespace balancing::power {

PowerController::PowerController(PowerConfig config, ResetCause resetCause) noexcept
    : config_(config),
      stage_(resetCause == ResetCause::BatteryConnection ? Stage::Off : Stage::Fault),
      runRequested_(resetCause == ResetCause::BatteryConnection) {
    // 電源再接続による自動起動も、初期出力はOFF。観測・設定が揃うまで投入しない。
}

bool PowerController::validConfig() const noexcept {
    // 実機閾値を仮定しない。期限が成立時間以下なら正常完了もできないため拒否。
    return config_.verified && std::isfinite(config_.maxVoltageDifferenceV) &&
           config_.maxVoltageDifferenceV > 0.0F && config_.stableUs > 0 &&
           config_.prechargeTimeoutUs > config_.stableUs &&
           config_.overlapTimeoutUs > 0 && config_.observationMaxAgeUs > 0;
}

bool PowerController::validObservation(const PowerInputs& in, std::uint64_t nowUs) const noexcept {
    // 未来値・時刻逆行・期限切れを除外してから差分を取る。NaNの比較によるすり抜けも防ぐ。
    return in.observationValid && in.observedAtUs <= nowUs &&
           (!seenObservation_ || in.observedAtUs >= lastObservationUs_) &&
           nowUs - in.observedAtUs < config_.observationMaxAgeUs &&
           std::isfinite(in.batteryVoltageV) && in.batteryVoltageV > 0.0F &&
           std::isfinite(in.driveBusVoltageV) && in.driveBusVoltageV >= 0.0F;
}

void PowerController::enterFault() noexcept {
    // 異常解除だけでは復帰しない。通常起動の意図も破棄する。
    runRequested_ = false;
    stage_ = Stage::Fault;
    voltageHolding_ = false;
}

void PowerController::enterOff() noexcept {
    stage_ = Stage::Off;
    voltageHolding_ = false;
}

void PowerController::startPrecharge(std::uint64_t nowUs) noexcept {
    stage_ = Stage::Charging;
    stageStartedUs_ = nowUs;
    voltageHolding_ = false;
}

PowerOutput PowerController::result(PowerCommandResult commandResult) const noexcept {
    // 状態から出力を導出し、別の変数として保持しない。FAULT/OFFでは両要求が必ずfalse。
    switch (stage_) {
    case Stage::Off: return {PowerState::Off, false, false, commandResult};
    case Stage::Charging: return {PowerState::Precharge, true, false, commandResult};
    case Stage::Overlap: return {PowerState::Precharge, true, true, commandResult};
    case Stage::On: return {PowerState::On, false, true, commandResult};
    case Stage::Fault: return {PowerState::Fault, false, false, commandResult};
    }
    return {PowerState::Fault, false, false, commandResult};
}

PowerOutput PowerController::output() const noexcept {
    return result(PowerCommandResult::None);
}

PowerOutput PowerController::step(const PowerInputs& in, std::uint64_t nowUs) noexcept {
    const bool onRequest = in.command == PowerCommand::On;
    const auto rejected = onRequest ? PowerCommandResult::SafetyRejected : PowerCommandResult::None;
    if (stepped_ && nowUs < lastStepUs_) {
        enterFault();
        return result(rejected);
    }
    stepped_ = true;
    lastStepUs_ = nowUs;

    const bool configOk = validConfig();
    const bool observed = configOk && validObservation(in, nowUs);
    const bool newObservation = observed &&
        (!seenObservation_ || in.observedAtUs > lastObservationUs_);
    // 成立時間の途中で観測が途切れた場合は、過去の保持時間を流用しない。
    const bool observationGap = newObservation && seenObservation_ &&
        in.observedAtUs - lastObservationUs_ >= config_.observationMaxAgeUs;
    if (newObservation) {
        seenObservation_ = true;
        lastObservationUs_ = in.observedAtUs;
    }

    // CAN非常停止と独立ハード異常はOFF/ON要求より優先し、ラッチする。
    if (in.command == PowerCommand::EmergencyStop || in.hardwareFault ||
        (observed && !in.protectionOk)) {
        enterFault();
        return result(rejected);
    }
    const bool active = stage_ == Stage::Charging || stage_ == Stage::Overlap || stage_ == Stage::On;
    if (active && (!observed || observationGap)) {
        enterFault();
        return result(rejected);
    }

    // 通常遮断は復帰意図も解除する。ただし既存FAULTをOFFで消去しない。
    if (in.command == PowerCommand::Off) {
        runRequested_ = false;
        const bool wasOff = stage_ == Stage::Off;
        if (stage_ != Stage::Fault) enterOff();
        return result(wasOff ? PowerCommandResult::AlreadyInState : PowerCommandResult::Accepted);
    }

    // 物理非常停止は自動復帰の既決方針を維持するが、CAN OFF/FAULTを上書きしない。
    if (!in.estopOk) {
        if (stage_ != Stage::Fault) enterOff();
        return result(rejected);
    }

    if (stage_ == Stage::Fault) {
        // 解除は新規正常観測と明示ONの同時成立だけ。失敗した要求を予約しない。
        if (!onRequest || !newObservation || !in.protectionOk) return result(rejected);
        runRequested_ = true;
        startPrecharge(nowUs);
        return result(PowerCommandResult::Accepted);
    }

    if (stage_ == Stage::Off) {
        const bool canStart = newObservation && in.protectionOk;
        if (onRequest && !canStart) return result(rejected);
        if (onRequest) runRequested_ = true;
        if (runRequested_ && canStart) startPrecharge(nowUs);
        return result(onRequest ? PowerCommandResult::Accepted : PowerCommandResult::None);
    }

    const auto activeResult = onRequest
        ? (stage_ == Stage::On ? PowerCommandResult::AlreadyInState : PowerCommandResult::Busy)
        : PowerCommandResult::None;

    if (stage_ == Stage::Charging) {
        // 期限と許容差の同時成立では期限を優先。開始前の観測を完了根拠にしない。
        if (nowUs - stageStartedUs_ >= config_.prechargeTimeoutUs) {
            enterFault();
            return result(rejected);
        }
        if (newObservation && in.observedAtUs > stageStartedUs_) {
            const float difference = std::fabs(in.batteryVoltageV - in.driveBusVoltageV);
            if (!std::isfinite(difference) || difference > config_.maxVoltageDifferenceV) {
                voltageHolding_ = false;
            } else if (!voltageHolding_) {
                voltageHolding_ = true;
                voltageHoldStartedUs_ = in.observedAtUs;
            } else if (in.observedAtUs - voltageHoldStartedUs_ >= config_.stableUs) {
                // 両要求ONの区間は別内部段階。外部へON報告するのは成立確認後。
                stage_ = Stage::Overlap;
                stageStartedUs_ = nowUs;
                voltageHolding_ = false;
            }
        }
    } else if (stage_ == Stage::Overlap) {
        if (nowUs - stageStartedUs_ >= config_.overlapTimeoutUs) {
            enterFault();
            return result(rejected);
        }
        // 既に充電済みのバス電圧だけでは主電源の導通確認にならない。
        // 回路に基づく独立確認を、投入要求より後の新規観測で受け取る。
        if (newObservation && in.observedAtUs > stageStartedUs_ && in.mainConfirmed) {
            stage_ = Stage::On;
        }
    }
    return result(activeResult);
}

} // namespace balancing::power
