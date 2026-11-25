#include "balancing/communication/can/PowerBoardCodec.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
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

// 実機送信はせず、仕様書から独立に転記したID・DATAを比較する。
void status()
{
    for (unsigned i=0;i<4;++i) {
        PowerStatusPayload p{static_cast<WirePowerState>(i),0xa5,0x1234,0xffff};
        auto f=encodePowerStatus(p);
        assert(f);
        golden(*f,0x0840bc03u,8,{static_cast<std::uint8_t>(i),0xa5,0x12,0x34,0xff,0xff,0,0});
        auto v=decodePowerStatus(*f);
        assert(v && v->state==p.state && v->flags==p.flags && v->faultBits==p.faultBits
               && v->sequence==p.sequence);
    }
    assert(!encodePowerStatus({static_cast<WirePowerState>(4),0,0,0}));
    auto f=*encodePowerStatus({WirePowerState::Off,0,0,0});
    // 未定義フラグは保持するが、予約領域と状態コードは厳密に拒否する。
    for (unsigned i=4;i<=255;++i) {
        f.data[0]=static_cast<std::uint8_t>(i);
        assert(!decodePowerStatus(f));
    }
    f.data[0]=0;
    for (auto offset : {6u,7u}) {
        f.data[offset]=1;
        assert(!decodePowerStatus(f));
        f.data[offset]=0;
    }
    assert(decodePowerStatus(f)->sequence==0);
}

void requests()
{
    struct Entry { NodeId node; PowerRequestKind kind; std::uint32_t id; };
    const Entry entries[]{
        {NodeId::F446,PowerRequestKind::On,0x08404801u},
        {NodeId::Pi,PowerRequestKind::On,0x0840c801u},
        {NodeId::F446,PowerRequestKind::Off,0x04404802u},
        {NodeId::Pi,PowerRequestKind::Off,0x0440c802u}
    };
    for (const auto& e:entries) {
        for (auto n : {std::uint16_t{0},std::uint16_t{0x1234},std::uint16_t{0xffff}}) {
            auto f=encodePowerRequest({e.node,e.kind,n});
            assert(f);
            golden(*f,e.id,2,{static_cast<std::uint8_t>(n>>8),static_cast<std::uint8_t>(n),0,0,0,0,0,0});
            auto v=decodePowerRequest(*f);
            assert(v && v->source==e.node && v->kind==e.kind && v->requestId==n);
        }
    }
    assert(!encodePowerRequest({NodeId::F303,PowerRequestKind::On,0}));
    assert(!encodePowerRequest({NodeId::Broadcast,PowerRequestKind::Off,0}));
    assert(!encodePowerRequest({NodeId::Pi,static_cast<PowerRequestKind>(3),0}));
}

void results()
{
    for (auto node : {NodeId::F446,NodeId::Pi})
        for (auto kind : {PowerRequestKind::On,PowerRequestKind::Off})
            for (unsigned state=0;state<4;++state)
                for (unsigned result=0;result<6;++result) {
                    PowerResult p{node,kind,0xabcd,static_cast<ResultCode>(result),static_cast<WirePowerState>(state)};
                    auto f=encodePowerResult(p);
                    assert(f);
                    golden(*f,node==NodeId::F446?0x08408404u:0x08408c04u,8,
                           {0xab,0xcd,0,static_cast<std::uint8_t>(kind),
                            static_cast<std::uint8_t>(result),static_cast<std::uint8_t>(state),0,0});
                    auto v=decodePowerResult(*f);
                    assert(v && v->destination==node && v->kind==kind && v->requestId==p.requestId
                           && v->result==p.result && v->state==p.state);
                }
    PowerResult p{NodeId::Pi,PowerRequestKind::On,0,ResultCode::Accepted,WirePowerState::Precharge};
    auto f=*encodePowerResult(p);
    // 受理と投入完了は別。プリチャージ状態をONに置換しない。
    assert(decodePowerResult(f)->state==WirePowerState::Precharge);
    for (auto offset : {2u,3u,4u,5u,6u,7u}) {
        auto bad=f;
        bad.data[offset]=0xff;
        assert(!decodePowerResult(bad));
    }
    p.destination=NodeId::Broadcast;assert(!encodePowerResult(p));p.destination=NodeId::Pi;
    p.kind=static_cast<PowerRequestKind>(0);assert(!encodePowerResult(p));p.kind=PowerRequestKind::On;
    p.result=static_cast<ResultCode>(6);assert(!encodePowerResult(p));p.result=ResultCode::Accepted;
    p.state=static_cast<WirePowerState>(4);assert(!encodePowerResult(p));
}

void stopAndInvalidFrames()
{
    const NodeId nodes[]{NodeId::F446,NodeId::F303,NodeId::Pi};
    const std::uint32_t ids[]{0x00047c01u,0x0004bc01u,0x0004fc01u};
    for (unsigned i=0;i<3;++i) {
        auto f=encodeEmergencyStop(nodes[i]);assert(f);
        golden(*f,ids[i],0,{});
        assert(decodeEmergencyStop(*f)==nodes[i]);
        // DLC0の未使用配列は意味を持たない。
        f->data.fill(0xff);assert(decodeEmergencyStop(*f)==nodes[i]);
    }
    assert(!encodeEmergencyStop(NodeId::Broadcast));
    assert(!encodeEmergencyStop(static_cast<NodeId>(0)));
    const CanFrame frames[]{
        *encodePowerStatus({WirePowerState::On,0,0,0}),
        *encodePowerRequest({NodeId::Pi,PowerRequestKind::On,1}),
        *encodePowerResult({NodeId::Pi,PowerRequestKind::On,1,ResultCode::Accepted,WirePowerState::On}),
        *encodeEmergencyStop(NodeId::Pi)
    };
    for (unsigned i=0;i<4;++i) {
        auto f=frames[i];
        assert(bool(decodePowerStatus(f))==(i==0));
        assert(bool(decodePowerRequest(f))==(i==1));
        assert(bool(decodePowerResult(f))==(i==2));
        assert(bool(decodeEmergencyStop(f))==(i==3));
        auto rejected=[](const CanFrame& bad) {
            assert(!decodePowerStatus(bad) && !decodePowerRequest(bad)
                   && !decodePowerResult(bad) && !decodeEmergencyStop(bad));
        };
        f.extended=false;rejected(f);f=frames[i];
        for (unsigned d=0;d<=9;++d) {
            if (d==frames[i].dlc) continue;
            f.dlc=static_cast<std::uint8_t>(d);rejected(f);
        }
        f=frames[i];f.id=0x700;rejected(f);f.extended=false;rejected(f);
        f=frames[i];f.id=0x1c000001u;rejected(f);
        f=frames[i];f.id|=0x20000000u;rejected(f);
        f=frames[i];f.id^=0x400u;rejected(f);
        f=frames[i];f.id^=0x100u;rejected(f);
    }
    // 別サービスで形式が正しいフレームも電源として受理しない。
    const CanFrame motion{0x0880c401u,{},8,true};
    assert(matchesProtocolFrame(motion));
    assert(!decodePowerStatus(motion) && !decodePowerRequest(motion)
           && !decodePowerResult(motion) && !decodeEmergencyStop(motion));
}
}
int main()
{
    status();requests();results();stopAndInvalidFrames();
    std::puts("PASS: PowerBoardCodec 10 golden IDs, payloads, enums, invalid frames");
}
