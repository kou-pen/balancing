#include "balancing/communication/can/CanRouter.hpp"

namespace balancing {
namespace {

constexpr std::uint8_t kFeedbackLength = 8;
constexpr std::uint32_t kExtendedMax = 0x1fffffffu;
constexpr std::uint32_t kRs06ReportType = 2;
constexpr std::uint32_t kRs06PeriodicType = 24;
constexpr std::uint32_t kC620LeftId = 0x201;
constexpr std::uint32_t kC620RightId = 0x202;

CanRxTarget classifyReceive(CanBus bus, const CanFrame& frame) noexcept
{
    // RS06とC620の状態報告はともに8バイト。短いフレームを機器の正常受信に数えない。
    if (frame.dlc != kFeedbackLength) {
        return CanRxTarget::Unmatched;
    }
    if (bus == CanBus::Can1 && frame.extended && frame.id <= kExtendedMax) {
        // 29ビットIDの上位5ビットはType。状態ビットと主機・軸IDはCodecに渡す。
        const auto type = frame.id >> 24u;
        if (type == kRs06ReportType || type == kRs06PeriodicType) {
            return CanRxTarget::Rs06Status;
        }
    } else if (bus == CanBus::Can2 && !frame.extended) {
        // 搭載する2軸の受信IDだけを選ぶ。電流指令や未確定のF303/上位制御は含めない。
        switch (frame.id) {
        case kC620LeftId:
            return CanRxTarget::C620LeftFeedback;
        case kC620RightId:
            return CanRxTarget::C620RightFeedback;
        }
    }
    return CanRxTarget::Unmatched;
}

} // namespace

CanRouter::CanRouter(CanEventSink& sink) noexcept : sink_{sink}
{
}

void CanRouter::dispatch(CanBus bus, const CanEvent& event) noexcept
{
    // 受信や送信受付を送信終端として流すと上位のトークン照合が崩れる。
    // 取得時のバスと時刻をそのまま渡し、種別ごとに有効なデータだけを通知する。
    switch (event.kind) {
    case CanEventKind::Rx:
        sink_.onCanReceive(bus, classifyReceive(bus, event.frame), event.frame, event.atUs);
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
