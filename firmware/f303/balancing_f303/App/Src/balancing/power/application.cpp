#include "balancing/power/application.hpp"

namespace balancing::power {

Application::Application(ApplicationConfig config) noexcept
    : controller_(config.controller, config.resetCause), monitor_(config.monitor),
      powerPinsVerified_(config.powerPinsVerified) {}

CycleResult Application::runCycle(const CycleInput& input, ApplyBoardOutput apply,
                                  void* context) noexcept {
    CycleResult result{};
    result.power = controller_.output();
    // タイマ通知なしでは周期を実行せず、CAN候補やIWDG更新許可を生成しない。
    if (input.pendingTicks == 0) return result;
    result.ran = true;
    const bool timingOk = input.pendingTicks == 1 &&
                          (!seenCycle_ || input.nowUs > lastCycleUs_);
    if (!seenCycle_ || input.nowUs > lastCycleUs_) lastCycleUs_ = input.nowUs;
    seenCycle_ = true;

    const auto observation = monitor_.evaluate(input.adc, result.power.state, input.nowUs);
    PowerInputs in{};
    in.estopOk = input.estopOk;
    // 未検証の出力接続や分類不能なIRQ遮断では電源投入を許さない。
    // 物理停止だけならControllerの既存自動復帰契約を維持する。
    const bool knownStop = input.shutdownCommand == PowerCommand::Off ||
                           input.shutdownCommand == PowerCommand::EmergencyStop;
    const bool unknownStop = input.shutdownPending && input.estopOk && !knownStop;
    in.hardwareFault = input.hardwareFault || !powerPinsVerified_ || unknownStop;
    in.command = knownStop ? input.shutdownCommand : PowerCommand::None;
    if (!timingOk || !apply) in.command = PowerCommand::EmergencyStop;
    in.mainConfirmed = input.mainConfirmed;
    in.observationValid = observation.valid;
    in.protectionOk = observation.protectionOk;
    in.batteryVoltageV = observation.batteryVoltageV;
    in.driveBusVoltageV = observation.driveBusVoltageV;
    in.observedAtUs = observation.observedAtUs;

    // 複数通知を追いつくために繰り返さない。停止優先で一度だけ評価する。
    auto tx = can_.step(controller_, in, input.rx, input.nowUs);
    const BoardInterlock gate{powerPinsVerified_, in.estopOk, in.hardwareFault,
                              input.shutdownPending || !timingOk};
    result.board = board_.evaluate(tx.power, gate, input.nowUs);

    // GPIO適用直前に割り込みラッチを再確認する責務は接続先にある。
    // 適用失敗時は接続先が両電源OFF済みという契約で、古い報告を全て捨てる。
    const bool applied = apply && apply(context, result.board);
    if (!applied || !result.board.accepted) {
        PowerInputs stop{};
        stop.command = PowerCommand::EmergencyStop;
        result.power = controller_.step(stop, input.nowUs);
        result.board = board_.evaluate(result.power, {}, input.nowUs);
        can_.invalidateStatus();
        return result;
    }
    result.power = tx.power;
    if (!timingOk) {
        // 周期欠落そのものは正常完了ではない。次の正常周期でFAULTを報告する。
        can_.invalidateStatus();
        return result;
    }
    result.tx = tx;
    result.completed = true;
    // FAULT保持でも周期が正常に完了した場合はウォッチドッグを更新できる。
    // 実際の更新や送信直前の再照合はHAL接続時の責務。
    result.watchdogEligible = true;
    return result;
}

} // namespace balancing::power
