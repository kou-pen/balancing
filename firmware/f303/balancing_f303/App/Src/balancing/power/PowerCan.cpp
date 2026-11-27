#include "balancing/power/PowerCan.hpp"
#include <algorithm>

namespace balancing::power {
namespace {
using namespace protocol;

WirePowerState wireState(PowerState s) noexcept {
    switch (s) {
    case PowerState::Off: return WirePowerState::Off;
    case PowerState::Precharge: return WirePowerState::Precharge;
    case PowerState::On: return WirePowerState::On;
    case PowerState::Fault: return WirePowerState::Fault;
    }
    return WirePowerState::Fault;
}

ResultCode wireResult(PowerCommandResult r) noexcept {
    switch (r) {
    case PowerCommandResult::Accepted: return ResultCode::Accepted;
    case PowerCommandResult::AlreadyInState: return ResultCode::AlreadyInState;
    case PowerCommandResult::Busy: return ResultCode::Busy;
    case PowerCommandResult::SafetyRejected: return ResultCode::SafetyRejected;
    case PowerCommandResult::None: break;
    }
    return ResultCode::SafetyRejected;
}
} // namespace

PowerCanOutput PowerCan::step(PowerController& controller, PowerInputs in,
                              const PowerCanRx& rx, std::uint64_t nowUs) noexcept {
    using namespace protocol;
    PowerCanOutput out{};
    std::array<PowerRequest, PowerCanRx::capacity> requests{};
    std::size_t count = 0;
    bool emergency = in.command == PowerCommand::EmergencyStop;
    bool off = in.command == PowerCommand::Off;
    const bool lost = rx.lost || rx.count > rx.frames.size();
    // 既存CodecでID、IDE、DLC、送信元、操作種別を照合する。
    // 自機ESTOPの受信反射は無視し、受信ESTOPを再送しない。
    for (std::size_t i=0; i<std::min(rx.count, rx.frames.size()); ++i) {
        const auto& frame = rx.frames[i];
        const auto stop = decodeEmergencyStop(frame);
        if (stop && *stop != NodeId::F303) emergency = true;
        if (auto req = decodePowerRequest(frame)) {
            requests[count++] = *req;
            if (req->kind == PowerRequestKind::Off) off = true;
        }
    }
    // 受信順に依存せず、ESTOP > OFF > ON。失われた停止要求を
    // 見落としたまま新規ONを受理しない。一方、CANなしの独立動作は維持する。
    std::size_t selected = count;
    if (!emergency) {
        for (std::size_t i=0; i<count; ++i) {
            if ((off && requests[i].kind == PowerRequestKind::Off) ||
                (!off && !lost && requests[i].kind == PowerRequestKind::On)) {
                selected = i;
                break;
            }
        }
    }
    if (emergency) in.command = PowerCommand::EmergencyStop;
    else if (off) in.command = PowerCommand::Off;
    else if (!lost && selected != count) in.command = PowerCommand::On;
    else if (lost || in.command != PowerCommand::On) in.command = PowerCommand::None;
    out.power = controller.step(in, nowUs);

    // 同一周期の複数要求を逐次実行するとOFF直後にONが走り得る。
    // 選択要求だけ実行し、抑止されたONには拒否、同種の残りにはBusyを返す。
    const auto state = wireState(out.power.state);
    for (std::size_t i=0; i<count; ++i) {
        const auto& req = requests[i];
        auto result = ResultCode::Busy;
        if (i == selected) result = wireResult(out.power.commandResult);
        else if (emergency || (req.kind == PowerRequestKind::On && (off || lost)))
            result = ResultCode::SafetyRejected;
        if (auto frame = encodePowerResult({req.source, req.kind, req.requestId, result, state}))
            out.replies[out.replyCount++] = *frame;
    }
    // 物理非常停止の立下りは自機から一度だけ通知。受信によるエコーは不要。
    if (!in.estopOk && estopWasOk_) out.emergency = encodeEmergencyStop(NodeId::F303);
    estopWasOk_ = in.estopOk;

    // 起動時と100msごとに最新状態を生成する。状態変更も即時通知し、
    // 短いFAULTを周期報告の間に見落としにくくする。遅延分をまとめて送らない。
    if (!statusIssued_ || out.power.state != lastStatusState_ ||
        nowUs < lastStatusUs_ || nowUs - lastStatusUs_ >= 100000u) {
        out.status = encodePowerStatus({state, 0, 0, sequence_});
        if (out.status) {
            ++sequence_; // uint16の周回。生成連番でありTX完了の保証ではない。
            statusIssued_ = true;
            lastStatusUs_ = nowUs;
            lastStatusState_ = out.power.state;
        }
    }
    return out;
}

} // namespace balancing::power
