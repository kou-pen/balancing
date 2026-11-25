#include "balancing/communication/can/RemoteControlCodec.hpp"

namespace balancing::protocol {
namespace {

constexpr bool kindValid(RemoteRequestKind kind) noexcept
{
    return kind==RemoteRequestKind::Arm || kind==RemoteRequestKind::Disarm
        || kind==RemoteRequestKind::ClearFault || kind==RemoteRequestKind::SelectSource;
}

constexpr bool sourceValid(WireInputSource source) noexcept
{
    return source==WireInputSource::UsbDualSense || source==WireInputSource::CanRemote;
}

constexpr bool stateValid(WireRobotState state) noexcept
{
    return static_cast<std::uint8_t>(state)<=3u;
}

constexpr bool resultValid(ResultCode result) noexcept
{
    return static_cast<std::uint8_t>(result)<=5u;
}

std::optional<CanFrame> makeFrame(ProtocolId fields) noexcept
{
    // IDの組合せを許可リストと照合し、DLCも仕様上の値を使う。
    const auto id=encodeId(fields);
    if (!id) return std::nullopt;
    const auto spec=findMessage(*id);
    if (!spec) return std::nullopt;
    return CanFrame{*id,{},spec->dlc,true};
}

std::optional<ProtocolId> checkedFields(const CanFrame& frame) noexcept
{
    // 他機能の正常フレームを操作要求と解釈しない。
    if (!matchesProtocolFrame(frame)) return std::nullopt;
    const auto fields=decodeId(frame.id);
    if (!fields || fields->type!=DataType::Remote) return std::nullopt;
    return fields;
}

} // namespace

std::optional<CanFrame> encodeMotionCommand(MotionCommand value) noexcept
{
    auto frame=makeFrame({2,DataType::Remote,NodeId::Pi,NodeId::F446,1});
    // 途中の書込みに失敗してもローカルの候補を破棄し、不完全なフレームを返さない。
    if (!frame || !writeFloat32BE(*frame,0,value.forwardVelocityMps)
        || !writeFloat32BE(*frame,4,value.yawRateRadS)) return std::nullopt;
    return frame;
}

std::optional<MotionCommand> decodeMotionCommand(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->reg!=1) return std::nullopt;
    const auto velocity=readFloat32BE(frame,0);
    const auto yaw=readFloat32BE(frame,4);
    // NaN/Infは共通変換で拒否。物理上限、受信時刻、操作元はServiceで判定する。
    if (!velocity || !yaw) return std::nullopt;
    return MotionCommand{*velocity,*yaw};
}

std::optional<CanFrame> encodeRemoteRequest(RemoteRequest value) noexcept
{
    if (!kindValid(value.kind)) return std::nullopt;
    // 操作元の指定はSELECT_SOURCEだけに許す。余分な指定を黙って捨てない。
    const bool select=value.kind==RemoteRequestKind::SelectSource;
    if (select!=value.source.has_value() || (value.source && !sourceValid(*value.source)))
        return std::nullopt;
    const std::uint8_t priority=value.kind==RemoteRequestKind::Disarm?1:2;
    auto frame=makeFrame({priority,DataType::Remote,NodeId::Pi,NodeId::F446,
                          static_cast<std::uint16_t>(value.kind)});
    if (!frame || !writeU16BE(*frame,0,value.requestId)) return std::nullopt;
    if (select) frame->data[2]=static_cast<std::uint8_t>(*value.source);
    return frame;
}

std::optional<RemoteRequest> decodeRemoteRequest(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->source!=NodeId::Pi || fields->destination!=NodeId::F446)
        return std::nullopt;
    const auto kind=static_cast<RemoteRequestKind>(fields->reg);
    if (!kindValid(kind)) return std::nullopt;
    const auto requestId=readU16BE(frame,0);
    if (!requestId) return std::nullopt;
    RemoteRequest value{kind,*requestId,{}};
    if (kind==RemoteRequestKind::SelectSource) {
        const auto source=static_cast<WireInputSource>(frame.data[2]);
        if (!sourceValid(source)) return std::nullopt;
        value.source=source;
    }
    // 要求番号は応答照合用。ここでARM等の状態変更や鮮度更新を行わない。
    return value;
}

std::optional<CanFrame> encodeRemoteResult(RemoteResult value) noexcept
{
    if (!kindValid(value.kind) || !resultValid(value.result) || !stateValid(value.state))
        return std::nullopt;
    auto frame=makeFrame({2,DataType::Remote,NodeId::F446,NodeId::Pi,6});
    if (!frame || !writeU16BE(*frame,0,value.requestId)
        || !writeU16BE(*frame,2,static_cast<std::uint16_t>(value.kind))) return std::nullopt;
    frame->data[4]=static_cast<std::uint8_t>(value.result);
    frame->data[5]=static_cast<std::uint8_t>(value.state);
    // 予約2byteはゼロ初期化を維持。
    return frame;
}

std::optional<RemoteResult> decodeRemoteResult(const CanFrame& frame) noexcept
{
    const auto fields=checkedFields(frame);
    if (!fields || fields->reg!=6 || frame.data[6]!=0 || frame.data[7]!=0)
        return std::nullopt;
    const auto requestId=readU16BE(frame,0);
    const auto reg=readU16BE(frame,2);
    if (!requestId || !reg) return std::nullopt;
    const auto kind=static_cast<RemoteRequestKind>(*reg);
    const auto result=static_cast<ResultCode>(frame.data[4]);
    const auto state=static_cast<WireRobotState>(frame.data[5]);
    // 受理結果と現在状態は独立。AcceptedをBalancingへ読み替えない。
    if (!kindValid(kind) || !resultValid(result) || !stateValid(state)) return std::nullopt;
    return RemoteResult{kind,*requestId,result,state};
}

} // namespace balancing::protocol
