#include "balancing/power/BoardIO.hpp"

namespace balancing::power {
namespace {

bool consistent(const PowerOutput& request) noexcept {
    // 状態と両出力の組が承認済み状態表にない場合、GPIO適用を拒否する。
    switch (request.state) {
    case PowerState::Off:
    case PowerState::Fault:
        return !request.prechargeRequested && !request.mainRequested;
    case PowerState::Precharge:
        return request.prechargeRequested;
    case PowerState::On:
        return !request.prechargeRequested && request.mainRequested;
    }
    return false;
}

} // namespace

BoardOutput BoardIO::evaluate(const PowerOutput& request, const BoardInterlock& gate,
                             std::uint64_t nowUs) noexcept {
    const bool clockOk = !ledStarted_ || nowUs >= lastUs_;
    const bool shapeOk = consistent(request);
    const bool active = request.prechargeRequested || request.mainRequested;
    // 既定値で全ての投入を拒否する。OFF要求は停止中・未検証でも通す。
    // これは論理境界であり、IRQとの競合を解消する実際のGPIOトランザクションではない。
    const bool safe = gate.powerPinsVerified && gate.estopOk &&
                      !gate.hardwareFault && !gate.shutdownPending;
    BoardOutput out{};
    out.accepted = clockOk && shapeOk && (!active || safe);
    if (out.accepted) {
        out.prechargeOn = request.prechargeRequested;
        out.mainOn = request.mainRequested;
    }

    // 拒否された能動出力をON表示しない。表示状態はControllerの状態を変更しない。
    // 拒否時はFAULT表示。正常OFFとFAULTは電源出力が同じでも表示で区別する。
    const PowerState display = out.accepted ? request.state : PowerState::Fault;
    if (!ledStarted_ || !clockOk || display != ledState_) {
        ledStarted_ = true;
        ledState_ = display;
        ledSinceUs_ = nowUs;
    }
    if (clockOk) lastUs_ = nowUs;
    // 逆行中は減算をしない。次の正常時刻まで拒否を維持する。
    const auto elapsed = nowUs - ledSinceUs_;
    switch (display) {
    case PowerState::Off: out.ledOn = false; break;
    case PowerState::On: out.ledOn = true; break;
    case PowerState::Precharge:
        // 2 Hz、周期500 ms。状態遷移時は点灯から開始し、待機処理は不要。
        out.ledOn = elapsed % 500000u < 250000u;
        break;
    case PowerState::Fault:
        // 5 Hz、周期200 ms。長時間経過も剰余だけで位相を算出する。
        out.ledOn = elapsed % 200000u < 100000u;
        break;
    }
    return out;
}

} // namespace balancing::power
