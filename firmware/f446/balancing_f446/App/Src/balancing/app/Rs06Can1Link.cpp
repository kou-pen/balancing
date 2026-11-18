#include "balancing/app/Rs06Can1Link.hpp"

namespace balancing {

FaultFlags drainRs06Can1(F446Can1Port& port, MotorService& motors) noexcept
{
    Can1Event event{};
    auto faults = FaultFlags::None;
    // 新しい割込みが続いても、制御周期へ戻れるように取り出し回数を制限する。
    for (std::size_t i = 0; i < F446Can1Port::maxEventsPerCycle && port.popEvent(event); ++i) {
        switch (event.kind) {
        case Can1EventKind::Rx:
            motors.onRs06Frame(event.frame, event.atUs);
            break;
        case Can1EventKind::Tx:
            motors.onRs06TxResult(event.token, event.txResult, event.atUs);
            break;
        case Can1EventKind::BusOff:
            motors.setCan1BusOff(true);
            break;
        case Can1EventKind::BusRecovered:
            motors.setCan1BusOff(false);
            break;
        case Can1EventKind::RxLost:
            // 受信キューの欠落を正常応答として扱うとARMの判断に穴が開く。
            // 送信の終端は捨てず、受信欠落を別の安全入力として渡す。
            faults = faults | FaultFlags::ExternalFault;
            break;
        }
    }
    return faults;
}

void submitRs06Can1(F446Can1Port& port, MotorService& motors, std::uint64_t nowUs) noexcept
{
    for (const auto& request : motors.pendingRs06Requests()) {
        if (request) {
            const auto result = port.trySend(*request, nowUs);
            // 割込みはキューに積むだけ。次のdrainより先に受付が確定する。
            motors.onRs06TxResult(request->token, result, nowUs);
        }
    }
}

} // namespace balancing
