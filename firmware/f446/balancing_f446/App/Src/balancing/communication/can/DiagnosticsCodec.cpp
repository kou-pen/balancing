#include "balancing/communication/can/DiagnosticsCodec.hpp"

namespace balancing::protocol {
namespace {

constexpr bool stateValid(WireRobotState s) noexcept { return static_cast<std::uint8_t>(s)<=3; }
constexpr bool sourceValid(WireInputSource s) noexcept { return static_cast<std::uint8_t>(s)<=1; }
constexpr bool resultValid(ResultCode r) noexcept { return static_cast<std::uint8_t>(r)<=5; }
constexpr bool faultBitsValid(std::uint16_t bits) noexcept { return (bits & 0xfc00u)==0; }
constexpr bool primaryValid(std::uint16_t bits) noexcept
{
    // 主原因は既知の単一ビット。ゼロ・複数原因・予約ビットを拒否する。
    return bits!=0 && faultBitsValid(bits) && (bits & (bits-1u))==0;
}
constexpr bool streamRequestValid(std::uint16_t reg) noexcept { return reg==0x80 || reg==0x81; }

std::optional<CanFrame> makeFrame(std::uint8_t priority,DataType type,NodeId src,NodeId dst,std::uint16_t reg) noexcept
{
    // DLCと方向を共通許可リストに委ねる。予約バイトはゼロ初期化する。
    const auto id=encodeId({priority,type,src,dst,reg});
    if (!id) return std::nullopt;
    const auto spec=findMessage(*id);
    if (!spec) return std::nullopt;
    return CanFrame{*id,{},spec->dlc,true};
}

} // namespace

std::optional<CanFrame> encodeDiagnostic(RobotStatusPayload value) noexcept
{
    // 状態・操作元・未確定状態フラグ・現在故障・未確定観測有効性。
    if (!(stateValid(value.state) && sourceValid(value.source) && faultBitsValid(value.faultBits))) return std::nullopt;
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x001);
    if (!frame) return std::nullopt;
    frame->data[0]=static_cast<std::uint8_t>(value.state);
    frame->data[1]=static_cast<std::uint8_t>(value.source);
    if (!writeU16BE(*frame,2,value.flags)) return std::nullopt;
    if (!writeU16BE(*frame,4,value.faultBits)) return std::nullopt;
    if (!writeU16BE(*frame,6,value.validity)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(AttitudeAngle value) noexcept
{
    // Pitch/Roll角 [rad]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x002);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.pitchRad)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.rollRad)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(AttitudeRate value) noexcept
{
    // Pitch/Roll角速度 [rad/s]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x003);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.pitchRadS)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.rollRadS)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(YawRate value) noexcept
{
    // Yaw角速度 [rad/s]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x004);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.yawRadS)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(ChassisState value) noexcept
{
    // 前後位置 [m] と速度 [m/s]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x005);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.positionM)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.velocityMps)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(WheelState value) noexcept
{
    // 左右車輪角速度 [rad/s]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x006);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.leftRadS)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.rightRadS)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(LeftLegState value) noexcept
{
    // 左脚長 [m] と角度 [rad]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x007);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.lengthM)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.angleRad)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(RightLegState value) noexcept
{
    // 右脚長 [m] と角度 [rad]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x008);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.lengthM)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.angleRad)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(ControlTarget value) noexcept
{
    // 前後速度目標 [m/s] とYaw角速度目標 [rad/s]。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x009);
    if (!frame) return std::nullopt;
    if (!writeFloat32BE(*frame,0,value.velocityMps)) return std::nullopt;
    if (!writeFloat32BE(*frame,4,value.yawRateRadS)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(ControlTiming value) noexcept
{
    // 実測周期 [us] と超過累計。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x00a);
    if (!frame) return std::nullopt;
    if (!writeU32BE(*frame,0,value.periodUs)) return std::nullopt;
    if (!writeU32BE(*frame,4,value.overrunCount)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(FaultPart1 value) noexcept
{
    // イベント番号・イベント時刻 [ms]・主原因単一ビット。
    if (!(primaryValid(value.primaryCause))) return std::nullopt;
    auto frame=makeFrame(1,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x020);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.eventId)) return std::nullopt;
    if (!writeU32BE(*frame,2,value.eventAtMs)) return std::nullopt;
    if (!writeU16BE(*frame,6,value.primaryCause)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(FaultPart2 value) noexcept
{
    // イベント番号・発生時と追加異常のビット。
    if (!(faultBitsValid(value.triggerFlags) && faultBitsValid(value.subsequentFlags))) return std::nullopt;
    auto frame=makeFrame(1,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x021);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.eventId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.triggerFlags)) return std::nullopt;
    if (!writeU16BE(*frame,4,value.subsequentFlags)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(FaultPart3 value) noexcept
{
    // イベント番号・元FAULT発生時刻 [ms]。
    auto frame=makeFrame(1,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x022);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.eventId)) return std::nullopt;
    if (!writeU32BE(*frame,2,value.originalAtMs)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(StreamSubscribe value) noexcept
{
    // 詳細購読要求。周期・期限 [ms] は上位で検査。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::Pi,NodeId::F446,0x080);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.registerId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.periodMs)) return std::nullopt;
    if (!writeU16BE(*frame,4,value.leaseMs)) return std::nullopt;
    if (!writeU16BE(*frame,6,value.requestId)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(StreamUnsubscribe value) noexcept
{
    // 購読解除要求。
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::Pi,NodeId::F446,0x081);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.registerId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.requestId)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(StreamResult value) noexcept
{
    // 購読操作の応答。
    if (!(streamRequestValid(value.requestRegisterId) && resultValid(value.result))) return std::nullopt;
    auto frame=makeFrame(3,DataType::Diagnostics,NodeId::F446,NodeId::Pi,0x082);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.requestId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.requestRegisterId)) return std::nullopt;
    frame->data[4]=static_cast<std::uint8_t>(value.result);
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(ParamRead value) noexcept
{
    // 設定番号の読み出し要求。
    auto frame=makeFrame(3,DataType::Parameters,NodeId::Pi,NodeId::F446,0x001);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.paramId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.requestId)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(ParamValue value) noexcept
{
    // 設定の生32bit値。型は番号別定義まで未確定。
    auto frame=makeFrame(3,DataType::Parameters,NodeId::F446,NodeId::Pi,0x002);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.paramId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.requestId)) return std::nullopt;
    if (!writeU32BE(*frame,4,value.rawBits)) return std::nullopt;
    return frame;
}

std::optional<CanFrame> encodeDiagnostic(ParamError value) noexcept
{
    // 設定読み出しエラー。
    if (!(resultValid(value.result))) return std::nullopt;
    auto frame=makeFrame(3,DataType::Parameters,NodeId::F446,NodeId::Pi,0x003);
    if (!frame) return std::nullopt;
    if (!writeU16BE(*frame,0,value.paramId)) return std::nullopt;
    if (!writeU16BE(*frame,2,value.requestId)) return std::nullopt;
    frame->data[4]=static_cast<std::uint8_t>(value.result);
    return frame;
}

std::optional<DiagnosticPayload> decodeDiagnostic(const CanFrame& frame) noexcept
{
    // 全IDの許可リスト・IDE・DLCを先に確認する。別機能を同じRegister IDで誤解釈しない。
    if (!matchesProtocolFrame(frame)) return std::nullopt;
    const auto fields=decodeId(frame.id);
    if (!fields) return std::nullopt;
    // Data typeとRegister IDで識別。方向・優先度は固定許可リストで検証済み。
    const auto key=(static_cast<std::uint32_t>(fields->type)<<10) | fields->reg;
    switch (key) {
    case (0x30u<<10)|0x001u: {
        const auto state=static_cast<WireRobotState>(frame.data[0]);
        const auto source=static_cast<WireInputSource>(frame.data[1]);
        const auto flags=readU16BE(frame,2);
        if (!flags) return std::nullopt;
        const auto faultBits=readU16BE(frame,4);
        if (!faultBits) return std::nullopt;
        const auto validity=readU16BE(frame,6);
        if (!validity) return std::nullopt;
        const RobotStatusPayload value{state,source,*flags,*faultBits,*validity};
        if (!(stateValid(value.state) && sourceValid(value.source) && faultBitsValid(value.faultBits))) return std::nullopt;
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x002u: {
        const auto pitchRad=readFloat32BE(frame,0);
        if (!pitchRad) return std::nullopt;
        const auto rollRad=readFloat32BE(frame,4);
        if (!rollRad) return std::nullopt;
        const AttitudeAngle value{*pitchRad,*rollRad};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x003u: {
        const auto pitchRadS=readFloat32BE(frame,0);
        if (!pitchRadS) return std::nullopt;
        const auto rollRadS=readFloat32BE(frame,4);
        if (!rollRadS) return std::nullopt;
        const AttitudeRate value{*pitchRadS,*rollRadS};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x004u: {
        const auto yawRadS=readFloat32BE(frame,0);
        if (!yawRadS) return std::nullopt;
        const YawRate value{*yawRadS};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x005u: {
        const auto positionM=readFloat32BE(frame,0);
        if (!positionM) return std::nullopt;
        const auto velocityMps=readFloat32BE(frame,4);
        if (!velocityMps) return std::nullopt;
        const ChassisState value{*positionM,*velocityMps};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x006u: {
        const auto leftRadS=readFloat32BE(frame,0);
        if (!leftRadS) return std::nullopt;
        const auto rightRadS=readFloat32BE(frame,4);
        if (!rightRadS) return std::nullopt;
        const WheelState value{*leftRadS,*rightRadS};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x007u: {
        const auto lengthM=readFloat32BE(frame,0);
        if (!lengthM) return std::nullopt;
        const auto angleRad=readFloat32BE(frame,4);
        if (!angleRad) return std::nullopt;
        const LeftLegState value{*lengthM,*angleRad};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x008u: {
        const auto lengthM=readFloat32BE(frame,0);
        if (!lengthM) return std::nullopt;
        const auto angleRad=readFloat32BE(frame,4);
        if (!angleRad) return std::nullopt;
        const RightLegState value{*lengthM,*angleRad};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x009u: {
        const auto velocityMps=readFloat32BE(frame,0);
        if (!velocityMps) return std::nullopt;
        const auto yawRateRadS=readFloat32BE(frame,4);
        if (!yawRateRadS) return std::nullopt;
        const ControlTarget value{*velocityMps,*yawRateRadS};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x00au: {
        const auto periodUs=readU32BE(frame,0);
        if (!periodUs) return std::nullopt;
        const auto overrunCount=readU32BE(frame,4);
        if (!overrunCount) return std::nullopt;
        const ControlTiming value{*periodUs,*overrunCount};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x020u: {
        const auto eventId=readU16BE(frame,0);
        if (!eventId) return std::nullopt;
        const auto eventAtMs=readU32BE(frame,2);
        if (!eventAtMs) return std::nullopt;
        const auto primaryCause=readU16BE(frame,6);
        if (!primaryCause) return std::nullopt;
        const FaultPart1 value{*eventId,*eventAtMs,*primaryCause};
        if (!(primaryValid(value.primaryCause))) return std::nullopt;
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x021u: {
        const auto eventId=readU16BE(frame,0);
        if (!eventId) return std::nullopt;
        const auto triggerFlags=readU16BE(frame,2);
        if (!triggerFlags) return std::nullopt;
        const auto subsequentFlags=readU16BE(frame,4);
        if (!subsequentFlags) return std::nullopt;
        if (frame.data[6]!=0 || frame.data[7]!=0) return std::nullopt;
        const FaultPart2 value{*eventId,*triggerFlags,*subsequentFlags};
        if (!(faultBitsValid(value.triggerFlags) && faultBitsValid(value.subsequentFlags))) return std::nullopt;
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x022u: {
        const auto eventId=readU16BE(frame,0);
        if (!eventId) return std::nullopt;
        const auto originalAtMs=readU32BE(frame,2);
        if (!originalAtMs) return std::nullopt;
        if (frame.data[6]!=0 || frame.data[7]!=0) return std::nullopt;
        const FaultPart3 value{*eventId,*originalAtMs};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x080u: {
        const auto registerId=readU16BE(frame,0);
        if (!registerId) return std::nullopt;
        const auto periodMs=readU16BE(frame,2);
        if (!periodMs) return std::nullopt;
        const auto leaseMs=readU16BE(frame,4);
        if (!leaseMs) return std::nullopt;
        const auto requestId=readU16BE(frame,6);
        if (!requestId) return std::nullopt;
        const StreamSubscribe value{*registerId,*periodMs,*leaseMs,*requestId};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x081u: {
        const auto registerId=readU16BE(frame,0);
        if (!registerId) return std::nullopt;
        const auto requestId=readU16BE(frame,2);
        if (!requestId) return std::nullopt;
        const StreamUnsubscribe value{*registerId,*requestId};
        return DiagnosticPayload{value};
    }
    case (0x30u<<10)|0x082u: {
        const auto requestId=readU16BE(frame,0);
        if (!requestId) return std::nullopt;
        const auto requestRegisterId=readU16BE(frame,2);
        if (!requestRegisterId) return std::nullopt;
        const auto result=static_cast<ResultCode>(frame.data[4]);
        if (frame.data[5]!=0) return std::nullopt;
        if (frame.data[6]!=0 || frame.data[7]!=0) return std::nullopt;
        const StreamResult value{*requestId,*requestRegisterId,result};
        if (!(streamRequestValid(value.requestRegisterId) && resultValid(value.result))) return std::nullopt;
        return DiagnosticPayload{value};
    }
    case (0x40u<<10)|0x001u: {
        const auto paramId=readU16BE(frame,0);
        if (!paramId) return std::nullopt;
        const auto requestId=readU16BE(frame,2);
        if (!requestId) return std::nullopt;
        const ParamRead value{*paramId,*requestId};
        return DiagnosticPayload{value};
    }
    case (0x40u<<10)|0x002u: {
        const auto paramId=readU16BE(frame,0);
        if (!paramId) return std::nullopt;
        const auto requestId=readU16BE(frame,2);
        if (!requestId) return std::nullopt;
        const auto rawBits=readU32BE(frame,4);
        if (!rawBits) return std::nullopt;
        const ParamValue value{*paramId,*requestId,*rawBits};
        return DiagnosticPayload{value};
    }
    case (0x40u<<10)|0x003u: {
        const auto paramId=readU16BE(frame,0);
        if (!paramId) return std::nullopt;
        const auto requestId=readU16BE(frame,2);
        if (!requestId) return std::nullopt;
        const auto result=static_cast<ResultCode>(frame.data[4]);
        if (frame.data[5]!=0) return std::nullopt;
        const ParamError value{*paramId,*requestId,result};
        if (!(resultValid(value.result))) return std::nullopt;
        return DiagnosticPayload{value};
    }
    default: return std::nullopt;
    }
}

} // namespace balancing::protocol
