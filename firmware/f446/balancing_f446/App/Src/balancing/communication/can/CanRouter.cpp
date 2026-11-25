#include "balancing/communication/can/CanRouter.hpp"

#include "balancing/communication/can/CanTxScheduler.hpp"
#include "balancing/communication/can/ProtocolId.hpp"

namespace balancing {
namespace {

constexpr std::uint8_t kFeedbackLength = 8;
constexpr std::uint32_t kExtendedMax = 0x1fffffffu;
constexpr std::uint32_t kRs06ReportType = 2;
constexpr std::uint32_t kRs06PeriodicType = 24;
constexpr std::uint32_t kC620LeftId = 0x201;
constexpr std::uint32_t kC620RightId = 0x202;

CanRxTarget classifyProtocol(const CanFrame& frame) noexcept
{
    using namespace protocol;
    // 先に固定ID・IDE・DLCを照合する。CAN IDの送信元は認証ではない。
    // DLC0の非常停止や短い操作要求はモータの8byte条件を適用しない。
    if (!matchesProtocolFrame(frame)) return CanRxTarget::Unmatched;
    const auto fields=decodeId(frame.id);
    if (!fields || fields->source==NodeId::F446 ||
        (fields->destination!=NodeId::F446 && fields->destination!=NodeId::Broadcast)) {
        return CanRxTarget::Unmatched;
    }
    // 宛先だけでなく機能とRegister IDを限定し、将来追加される定義を暗黙に配送しない。
    switch (fields->type) {
    case DataType::Common:
        if (fields->reg==1) return CanRxTarget::EmergencyStop;
        break;
    case DataType::Power:
        if (fields->reg==3) return CanRxTarget::PowerStatus;
        if (fields->reg==4) return CanRxTarget::PowerResult;
        break;
    case DataType::Remote:
        if (fields->reg==1) return CanRxTarget::RemoteMotion;
        if (fields->reg>=2 && fields->reg<=5) return CanRxTarget::RemoteRequest;
        break;
    case DataType::Diagnostics:
        if (fields->reg==0x80 || fields->reg==0x81) return CanRxTarget::DiagnosticRequest;
        break;
    case DataType::Parameters:
        if (fields->reg==1) return CanRxTarget::DiagnosticRequest;
        break;
    }
    return CanRxTarget::Unmatched;
}

CanRxTarget classifyReceive(CanBus bus, const CanFrame& frame) noexcept
{
    // 自作通信はCAN2の拡張データフレームだけを対象にする。
    if (bus == CanBus::Can2 && frame.extended) return classifyProtocol(frame);

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
        // 搭載する2軸の受信IDだけを選ぶ。電流指令や自作通信は含めない。
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

CanRouter::CanRouter(CanEventSink& sink, CanTxScheduler* can1Tx, CanTxScheduler* can2Tx) noexcept
    : sink_{sink}, can1Tx_{can1Tx}, can2Tx_{can2Tx}
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
    case CanEventKind::Tx: {
        // バスとトークンで終端を照合し、停止ではSentだけを完了として扱う。
        // 失敗の通知時点でメールボックスの枠は返却済みなので、要求元は再登録できる。
        auto* const scheduler = bus == CanBus::Can1 ? can1Tx_ : can2Tx_;
        if (!scheduler || scheduler->consumeTerminal(event.token, event.outcome, event.atUs)) {
            sink_.onCanTransmit(bus, event.token, event.outcome, event.atUs);
        }
        break;
    }
    case CanEventKind::BusOff:
    case CanEventKind::BusRecovered:
    case CanEventKind::RxLost:
        sink_.onCanPortStatus(bus, event.kind, event.atUs);
        break;
    }
}

} // namespace balancing
