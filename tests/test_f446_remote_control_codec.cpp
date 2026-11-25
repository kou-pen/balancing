#include "balancing/communication/can/RemoteControlCodec.hpp"
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

void golden(const CanFrame& f,std::uint32_t id,std::uint8_t dlc,Bytes data)
{
    assert(f.extended && f.id==id && f.dlc==dlc && f.data==data);
    assert(matchesProtocolFrame(f) && !isReservedExtendedId(f.id));
}

void motion()
{
    // 仕様書とIEEE754の既知表現から独立に作った人工値。実機駆動値ではない。
    auto f=encodeMotionCommand({0.5f,-0.25f});
    assert(f);
    golden(*f,0x0880c401u,8,{0x3f,0,0,0,0xbe,0x80,0,0});
    auto v=decodeMotionCommand(*f);
    assert(v && v->forwardVelocityMps==0.5f && v->yawRateRadS==-0.25f);
    auto zero=encodeMotionCommand({0,0});
    assert(zero && decodeMotionCommand(*zero)->forwardVelocityMps==0);
    for (float bad : {std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()}) {
        assert(!encodeMotionCommand({bad,0}) && !encodeMotionCommand({0,bad}));
    }
    for (auto bits : {0x7f800000u,0xff800000u,0x7fc00000u,0x7f800001u}) {
        for (auto offset : {0u,4u}) {
            auto bad=*f;
            assert(writeU32BE(bad,offset,bits));
            assert(!decodeMotionCommand(bad));
        }
    }
}

void requests()
{
    const RemoteRequestKind kinds[]{RemoteRequestKind::Arm,RemoteRequestKind::Disarm,
                                    RemoteRequestKind::ClearFault,RemoteRequestKind::SelectSource};
    const std::uint32_t ids[]{0x0880c402u,0x0480c403u,0x0880c404u,0x0880c405u};
    for (unsigned i=0;i<4;++i)
        for (auto number : {std::uint16_t{0},std::uint16_t{0x1234},std::uint16_t{0xffff}}) {
            const bool select=i==3;
            for (unsigned source=0;source<(select?2u:1u);++source) {
                RemoteRequest req{kinds[i],number,{}};
                if (select) req.source=static_cast<WireInputSource>(source);
                const auto f=encodeRemoteRequest(req);assert(f);
                golden(*f,ids[i],select?3:2,
                       {static_cast<std::uint8_t>(number>>8),static_cast<std::uint8_t>(number),
                        static_cast<std::uint8_t>(source),0,0,0,0,0});
                auto v=decodeRemoteRequest(*f);
                assert(v && v->kind==req.kind && v->requestId==number && v->source==req.source);
            }
        }
    // 要求種別ごとの必須フィールドを検査し、余分な操作元指定を受理しない。
    assert(!encodeRemoteRequest({RemoteRequestKind::SelectSource,0,{}}));
    assert(!encodeRemoteRequest({RemoteRequestKind::SelectSource,0,static_cast<WireInputSource>(2)}));
    for (unsigned i=0;i<3;++i)
        assert(!encodeRemoteRequest({kinds[i],0,WireInputSource::UsbDualSense}));
    assert(!encodeRemoteRequest({static_cast<RemoteRequestKind>(1),0,{}}));
    auto f=*encodeRemoteRequest({RemoteRequestKind::SelectSource,0,WireInputSource::CanRemote});
    for (unsigned s=2;s<=255;++s) {
        f.data[2]=static_cast<std::uint8_t>(s);
        assert(!decodeRemoteRequest(f));
    }
}

void results()
{
    for (unsigned k=2;k<=5;++k)
        for (unsigned result=0;result<=5;++result)
            for (unsigned state=0;state<=3;++state)
                for (auto number : {std::uint16_t{0},std::uint16_t{0xffff}}) {
                    const RemoteResult r{static_cast<RemoteRequestKind>(k),number,
                        static_cast<ResultCode>(result),static_cast<WireRobotState>(state)};
                    auto f=encodeRemoteResult(r);assert(f);
                    golden(*f,0x08804c06u,8,
                           {static_cast<std::uint8_t>(number>>8),static_cast<std::uint8_t>(number),
                            0,static_cast<std::uint8_t>(k),static_cast<std::uint8_t>(result),
                            static_cast<std::uint8_t>(state),0,0});
                    const auto v=decodeRemoteResult(*f);
                    assert(v && v->kind==r.kind && v->requestId==number &&
                           v->result==r.result && v->state==r.state);
                }
    RemoteResult r{RemoteRequestKind::Arm,42,ResultCode::Accepted,WireRobotState::Arming};
    const auto f=*encodeRemoteResult(r);
    // AcceptedをBALANCINGへ書き換えず、現在の状態を独立に報告する。
    assert(decodeRemoteResult(f)->state==WireRobotState::Arming);
    for (auto offset : {2u,3u,4u,5u,6u,7u}) {
        auto bad=f;bad.data[offset]=0xff;assert(!decodeRemoteResult(bad));
    }
    r.kind=static_cast<RemoteRequestKind>(6);assert(!encodeRemoteResult(r));
    r.kind=RemoteRequestKind::Arm;r.result=static_cast<ResultCode>(6);assert(!encodeRemoteResult(r));
    r.result=ResultCode::Accepted;r.state=static_cast<WireRobotState>(4);assert(!encodeRemoteResult(r));
}

void invalidFrames()
{
    const CanFrame frames[]{
        *encodeMotionCommand({0,0}),
        *encodeRemoteRequest({RemoteRequestKind::Arm,1,{}}),
        *encodeRemoteRequest({RemoteRequestKind::Disarm,1,{}}),
        *encodeRemoteRequest({RemoteRequestKind::ClearFault,1,{}}),
        *encodeRemoteRequest({RemoteRequestKind::SelectSource,1,WireInputSource::CanRemote}),
        *encodeRemoteResult({RemoteRequestKind::Arm,1,ResultCode::Accepted,WireRobotState::Arming})
    };
    auto rejected=[](const CanFrame& f) {
        assert(!decodeMotionCommand(f) && !decodeRemoteRequest(f) && !decodeRemoteResult(f));
    };
    for (unsigned i=0;i<6;++i) {
        auto f=frames[i];
        assert(bool(decodeMotionCommand(f))==(i==0));
        assert(bool(decodeRemoteRequest(f))==(i>=1 && i<=4));
        assert(bool(decodeRemoteResult(f))==(i==5));
        f.extended=false;rejected(f);
        for (unsigned d=0;d<=9;++d) {
            if (d==frames[i].dlc) continue;
            f=frames[i];f.dlc=static_cast<std::uint8_t>(d);rejected(f);
        }
        for (auto id : {0x700u,0x1c000001u,frames[i].id|0x20000000u,
                        frames[i].id^0x400u,frames[i].id^0x4000u,frames[i].id^0x100u}) {
            f=frames[i];f.id=id;rejected(f);
        }
    }
    // 共通許可リスト上の正常な電源・診断フレームも操作としては拒否。
    const CanFrame other[]{ {0x0840bc03u,{},8,true},{0x0cc04c02u,{},8,true},
                           {0x0004fc01u,{},0,true} };
    for (const auto& f:other) {assert(matchesProtocolFrame(f));rejected(f);}
}
}
int main()
{
    motion();requests();results();invalidFrames();
    std::puts("PASS: RemoteControlCodec 6 golden IDs, finite float32, enums and invalid frames");
}
