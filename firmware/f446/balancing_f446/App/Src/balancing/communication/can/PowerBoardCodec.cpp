#include "balancing/communication/can/PowerBoardCodec.hpp"

namespace balancing::protocol {
namespace {

constexpr bool stateValid(WirePowerState state) noexcept
{
    return state == WirePowerState::Off || state == WirePowerState::Precharge
        || state == WirePowerState::On || state == WirePowerState::Fault;
}

constexpr bool kindValid(PowerRequestKind kind) noexcept
{
    return kind == PowerRequestKind::On || kind == PowerRequestKind::Off;
}

constexpr bool resultValid(ResultCode result) noexcept
{
    return static_cast<std::uint8_t>(result) <= 5u;
}

constexpr bool requesterValid(NodeId node) noexcept
{
    return node == NodeId::F446 || node == NodeId::Pi;
}

std::optional<CanFrame> makeFrame(ProtocolId fields) noexcept
{
    // DLCは固定許可リストから取得する。未登録の方向・優先度を生成しない。
    const auto id=encodeId(fields);
    if (!id) return std::nullopt;
    const auto spec=findMessage(*id);
    if (!spec) return std::nullopt;
    return CanFrame{*id,{},spec->dlc,true};
}

std::optional<ProtocolId> checkedFields(const CanFrame& frame) noexcept
{
    // IDの構成要素を読む前にIDEと正確なDLCを検査する。
    if (!matchesProtocolFrame(frame)) return std::nullopt;
    return decodeId(frame.id);
}

bool reservedZero(const CanFrame& frame) noexcept
{
    return frame.data[6]==0 && frame.data[7]==0;
}

} // namespace

std::optional<CanFrame> encodePowerStatus(PowerStatusPayload value) noexcept
{
    if (!stateValid(value.state)) return std::nullopt;
    auto frame=makeFrame({2,DataType::Power,NodeId::F303,NodeId::Broadcast,3});
    if (!frame) return std::nullopt;
    frame->data[0]=static_cast<std::uint8_t>(value.state);
    // 未定義ビットを解釈せず伝送する。予約byteは初期化時の0を維持。
    frame->data[1]=value.flags;
    if (!writeU16BE(*frame,2,value.faultBits) || !writeU16BE(*frame,4,value.sequence))
        return std::nullopt;
    return frame;
}

std::optional<PowerStatusPayload> decodePowerStatus(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->type!=DataType::Power || fields->reg!=3
        || !reservedZero(frame)) return std::nullopt;
    const auto state=static_cast<WirePowerState>(frame.data[0]);
    const auto faults=readU16BE(frame,2);
    const auto seq=readU16BE(frame,4);
    if (!stateValid(state) || !faults || !seq) return std::nullopt;
    // ONや生故障ビットだけからreadyを作らない。鮮度と保護はService側。
    return PowerStatusPayload{state,frame.data[1],*faults,*seq};
}

std::optional<CanFrame> encodePowerRequest(PowerRequest value) noexcept
{
    if (!requesterValid(value.source) || !kindValid(value.kind)) return std::nullopt;
    // OFFの優先度をONより高くするのは固定ID仕様。実行順や遮断は扱わない。
    const std::uint8_t priority=value.kind==PowerRequestKind::Off?1:2;
    auto frame=makeFrame({priority,DataType::Power,value.source,NodeId::F303,
                          static_cast<std::uint16_t>(value.kind)});
    if (!frame || !writeU16BE(*frame,0,value.requestId)) return std::nullopt;
    return frame;
}

std::optional<PowerRequest> decodePowerRequest(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->type!=DataType::Power || !requesterValid(fields->source)
        || fields->destination!=NodeId::F303) return std::nullopt;
    const auto kind=static_cast<PowerRequestKind>(fields->reg);
    const auto requestId=readU16BE(frame,0);
    if (!kindValid(kind) || !requestId) return std::nullopt;
    return PowerRequest{fields->source,kind,*requestId};
}

std::optional<CanFrame> encodePowerResult(PowerResult value) noexcept
{
    if (!requesterValid(value.destination) || !kindValid(value.kind)
        || !resultValid(value.result) || !stateValid(value.state)) return std::nullopt;
    auto frame=makeFrame({2,DataType::Power,NodeId::F303,value.destination,4});
    if (!frame || !writeU16BE(*frame,0,value.requestId)
        || !writeU16BE(*frame,2,static_cast<std::uint16_t>(value.kind))) return std::nullopt;
    frame->data[4]=static_cast<std::uint8_t>(value.result);
    frame->data[5]=static_cast<std::uint8_t>(value.state);
    return frame;
}

std::optional<PowerResult> decodePowerResult(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->type!=DataType::Power || fields->reg!=4
        || !reservedZero(frame)) return std::nullopt;
    const auto requestId=readU16BE(frame,0);
    const auto reg=readU16BE(frame,2);
    if (!requestId || !reg) return std::nullopt;
    const auto kind=static_cast<PowerRequestKind>(*reg);
    const auto result=static_cast<ResultCode>(frame.data[4]);
    const auto state=static_cast<WirePowerState>(frame.data[5]);
    // 応答コードと電源状態は別情報。AcceptedでもONとは限らない。
    if (!kindValid(kind) || !resultValid(result) || !stateValid(state)) return std::nullopt;
    return PowerResult{fields->destination,kind,*requestId,result,state};
}

std::optional<CanFrame> encodeEmergencyStop(NodeId source) noexcept
{
    // 共通の許可リストが送信元3種類とブロードキャストを限定する。
    return makeFrame({0,DataType::Common,source,NodeId::Broadcast,1});
}

std::optional<NodeId> decodeEmergencyStop(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->type!=DataType::Common || fields->reg!=1) return std::nullopt;
    // DLC0なので配列上の未使用バイトには意味を持たせない。
    return fields->source;
}

} // namespace balancing::protocol
