#include "balancing/communication/can/CanRouter.hpp"

namespace balancing {

CanRouter::CanRouter(CanEventSink& sink) noexcept : sink_{sink}
{
}

void CanRouter::dispatch(CanBus bus, const CanEvent& event) noexcept
{
    // 受信や送信受付を送信終端として流すと上位のトークン照合が崩れる。
    // 取得時のバスと時刻をそのまま渡し、種別ごとに有効なデータだけを通知する。
    switch (event.kind) {
    case CanEventKind::Rx:
        sink_.onCanReceive(bus, event.frame, event.atUs);
        break;
    case CanEventKind::Tx:
        sink_.onCanTransmit(bus, event.token, event.outcome, event.atUs);
        break;
    case CanEventKind::BusOff:
    case CanEventKind::BusRecovered:
    case CanEventKind::RxLost:
        sink_.onCanPortStatus(bus, event.kind, event.atUs);
        break;
    }
}

} // namespace balancing
