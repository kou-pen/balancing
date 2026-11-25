#include "balancing/communication/can/DiagnosticsCodec.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <limits>
#include <cstdio>

using namespace balancing;
using namespace balancing::protocol;
namespace {
using Bytes=std::array<std::uint8_t,8>;

template<class T>
void golden(T value,std::uint32_t id,std::uint8_t dlc,Bytes bytes)
{
    // 仕様の独立した固定バイト列と照合する。単なる自己往復だけでは誤配置を見逃す。
    const auto encoded=encodeDiagnostic(value);
    assert(encoded && encoded->id==id && encoded->dlc==dlc && encoded->extended && encoded->data==bytes);
    assert(matchesProtocolFrame(*encoded) && !isReservedExtendedId(id));
    const CanFrame input{id,bytes,dlc,true};
    const auto decoded=decodeDiagnostic(input);
    assert(decoded && std::holds_alternative<T>(*decoded));
    const auto roundtrip=encodeDiagnostic(std::get<T>(*decoded));
    assert(roundtrip && roundtrip->data==bytes && roundtrip->id==id && roundtrip->dlc==dlc);
    // IDE、長さ、方向、範囲、予約領域の変異はどの型でも拒否される。
    auto bad=input;bad.extended=false;assert(!decodeDiagnostic(bad));
    for (unsigned n=0;n<=9;++n) {
        if (n==dlc) continue;
        bad=input;bad.dlc=static_cast<std::uint8_t>(n);assert(!decodeDiagnostic(bad));
    }
    for (auto invalid : {0x700u,0x1c000001u,id|0x20000000u,id^0x400u,id^0x4000u,id^0x100u}) {
        bad=input;bad.id=invalid;assert(!decodeDiagnostic(bad));
    }
}

void allMessages()
{
    golden(RobotStatusPayload{WireRobotState::Balancing,WireInputSource::CanRemote,0x1234,0x0201,0xabcd},0xcc04c01u,8,{0x2,0x1,0x12,0x34,0x2,0x1,0xab,0xcd});
    golden(AttitudeAngle{0.5f,-0.25f},0xcc04c02u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(AttitudeRate{0.5f,-0.25f},0xcc04c03u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(YawRate{0.5f},0xcc04c04u,4,{0x3f,0x0,0x0,0x0});
    golden(ChassisState{0.5f,-0.25f},0xcc04c05u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(WheelState{0.5f,-0.25f},0xcc04c06u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(LeftLegState{0.5f,-0.25f},0xcc04c07u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(RightLegState{0.5f,-0.25f},0xcc04c08u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(ControlTarget{0.5f,-0.25f},0xcc04c09u,8,{0x3f,0x0,0x0,0x0,0xbe,0x80,0x0,0x0});
    golden(ControlTiming{0x12345678,0xabcdef01},0xcc04c0au,8,{0x12,0x34,0x56,0x78,0xab,0xcd,0xef,0x1});
    golden(FaultPart1{0xabcd,0x12345678,0x0200},0x4c04c20u,8,{0xab,0xcd,0x12,0x34,0x56,0x78,0x2,0x0});
    golden(FaultPart2{0xabcd,0x0201,0x0008},0x4c04c21u,8,{0xab,0xcd,0x2,0x1,0x0,0x8,0x0,0x0});
    golden(FaultPart3{0xabcd,0x12345678},0x4c04c22u,8,{0xab,0xcd,0x12,0x34,0x56,0x78,0x0,0x0});
    golden(StreamSubscribe{2,20,1000,0xabcd},0xcc0c480u,8,{0x0,0x2,0x0,0x14,0x3,0xe8,0xab,0xcd});
    golden(StreamUnsubscribe{2,0xabcd},0xcc0c481u,4,{0x0,0x2,0xab,0xcd});
    golden(StreamResult{0xabcd,0x80,ResultCode::Accepted},0xcc04c82u,8,{0xab,0xcd,0x0,0x80,0x0,0x0,0x0,0x0});
    golden(ParamRead{0x1234,0xabcd},0xd00c401u,4,{0x12,0x34,0xab,0xcd});
    golden(ParamValue{0x1234,0xabcd,0x7fc00000},0xd004c02u,8,{0x12,0x34,0xab,0xcd,0x7f,0xc0,0x0,0x0});
    golden(ParamError{0x1234,0xabcd,ResultCode::InvalidRequest},0xd004c03u,6,{0x12,0x34,0xab,0xcd,0x4,0x0});
}

void invalidValues()
{
    // float32を含む全フレームについて、各物理量の非有限値を拒否する。
    for (std::uint32_t reg=2;reg<=9;++reg) {
        CanFrame frame{0x0cc04c00u|reg,{},static_cast<std::uint8_t>(reg==4?4:8),true};
        assert(decodeDiagnostic(frame));
        for (unsigned offset=0;offset<frame.dlc;offset+=4)
            for (auto bits : {0x7f800000u,0xff800000u,0x7fc00000u,0x7f800001u}) {
                auto bad=frame;assert(writeU32BE(bad,offset,bits));assert(!decodeDiagnostic(bad));
            }
    }
    const float nan=std::numeric_limits<float>::quiet_NaN(), inf=std::numeric_limits<float>::infinity();
    assert(!encodeDiagnostic(AttitudeAngle{nan,0}) && !encodeDiagnostic(AttitudeAngle{0,inf}));
    assert(!encodeDiagnostic(AttitudeRate{nan,0}) && !encodeDiagnostic(AttitudeRate{0,inf}));
    assert(!encodeDiagnostic(ChassisState{nan,0}) && !encodeDiagnostic(ChassisState{0,inf}));
    assert(!encodeDiagnostic(WheelState{nan,0}) && !encodeDiagnostic(WheelState{0,inf}));
    assert(!encodeDiagnostic(LeftLegState{nan,0}) && !encodeDiagnostic(LeftLegState{0,inf}));
    assert(!encodeDiagnostic(RightLegState{nan,0}) && !encodeDiagnostic(RightLegState{0,inf}));
    assert(!encodeDiagnostic(ControlTarget{nan,0}) && !encodeDiagnostic(ControlTarget{0,inf}));
    assert(!encodeDiagnostic(YawRate{nan}) && !encodeDiagnostic(YawRate{inf}));
    for (unsigned state=0;state<=255;++state) {
        auto f=CanFrame{0x0cc04c01u,{},8,true};
        f.data[0]=static_cast<std::uint8_t>(state);
        assert(bool(decodeDiagnostic(f))==(state<=3));
        assert(bool(encodeDiagnostic(RobotStatusPayload{static_cast<WireRobotState>(state),WireInputSource::CanRemote,0,0,0}))==(state<=3));
        f.data[0]=0;f.data[1]=static_cast<std::uint8_t>(state);
        assert(bool(decodeDiagnostic(f))==(state<=1));
        assert(bool(encodeDiagnostic(RobotStatusPayload{WireRobotState::Disarmed,static_cast<WireInputSource>(state),0,0,0}))==(state<=1));
    }
    // 主原因は単一の既知ビット。その他のFAULTフィールドは既知ビットの組合せ。
    for (unsigned bit=0;bit<16;++bit) {
        const auto mask=static_cast<std::uint16_t>(1u<<bit);
        auto f=CanFrame{0x04c04c20u,{},8,true};assert(writeU16BE(f,6,mask));
        assert(bool(decodeDiagnostic(f))==(bit<10));
        assert(bool(encodeDiagnostic(FaultPart1{0,0,mask}))==(bit<10));
        assert(bool(encodeDiagnostic(FaultPart2{0,mask,0}))==(bit<10));
        assert(bool(encodeDiagnostic(FaultPart2{0,0,mask}))==(bit<10));
        auto f2=CanFrame{0x04c04c21u,{},8,true};assert(writeU16BE(f2,2,mask));
        assert(bool(decodeDiagnostic(f2))==(bit<10));
        f2.data.fill(0);assert(writeU16BE(f2,4,mask));assert(bool(decodeDiagnostic(f2))==(bit<10));
        auto st=CanFrame{0x0cc04c01u,{},8,true};assert(writeU16BE(st,4,mask));
        assert(bool(decodeDiagnostic(st))==(bit<10));
        assert(bool(encodeDiagnostic(RobotStatusPayload{WireRobotState::Disarmed,WireInputSource::CanRemote,0,mask,0}))==(bit<10));
    }
    for (auto bad : {std::uint16_t{0},std::uint16_t{3},std::uint16_t{0x8000}}) {
        assert(!encodeDiagnostic(FaultPart1{0,0,bad}));
        auto f=CanFrame{0x04c04c20u,{},8,true};assert(writeU16BE(f,6,bad));assert(!decodeDiagnostic(f));
    }
    for (unsigned result=0;result<=255;++result) {
        auto f=CanFrame{0x0cc04c82u,{0,0,0,0x80,static_cast<std::uint8_t>(result)},8,true};
        assert(bool(decodeDiagnostic(f))==(result<=5));
        assert(bool(encodeDiagnostic(StreamResult{0,0x80,static_cast<ResultCode>(result)}))==(result<=5));
        auto e=CanFrame{0x0d004c03u,{0,0,0,0,static_cast<std::uint8_t>(result)},6,true};
        assert(bool(decodeDiagnostic(e))==(result<=5));
        assert(bool(encodeDiagnostic(ParamError{0,0,static_cast<ResultCode>(result)}))==(result<=5));
    }
    for (auto reg : {std::uint16_t{0x80},std::uint16_t{0x81},std::uint16_t{0x82},std::uint16_t{0}}) {
        auto f=CanFrame{0x0cc04c82u,{},8,true};assert(writeU16BE(f,2,reg));
        const bool valid=reg==0x80 || reg==0x81;
        assert(bool(decodeDiagnostic(f))==valid);
        assert(bool(encodeDiagnostic(StreamResult{0,reg,ResultCode::Accepted}))==valid);
    }
    // 未確定の購読条件・設定値ビットは意味を仮定せず、上位に渡す。
    golden(StreamSubscribe{0xffff,0,0,0xffff},0x0cc0c480u,8,{0xff,0xff,0,0,0,0,0xff,0xff});
    golden(ParamValue{0xffff,0,0xffffffffu},0x0d004c02u,8,{0xff,0xff,0,0,0xff,0xff,0xff,0xff});
    const CanFrame reserved[]{
        {0x04c04c21u,{},8,true},{0x04c04c22u,{},8,true},
        {0x0cc04c82u,{0,0,0,0x80},8,true},{0x0d004c03u,{},6,true}
    };
    const unsigned offsets[]{6,6,5,5};
    for (unsigned i=0;i<4;++i) {
        assert(decodeDiagnostic(reserved[i]));
        for (unsigned j=offsets[i];j<reserved[i].dlc;++j) {
            auto f=reserved[i];f.data[j]=1;assert(!decodeDiagnostic(f));
        }
    }
    for (const auto f : {CanFrame{0x0880c401u,{},8,true},CanFrame{0x0840bc03u,{},8,true},CanFrame{0x0004fc01u,{},0,true}}) {
        assert(matchesProtocolFrame(f));assert(!decodeDiagnostic(f));
    }
}
}

int main()
{
    allMessages();invalidValues();
    std::puts("PASS: DiagnosticsCodec 19 golden IDs, typed payloads, enum and fault masks");
}
