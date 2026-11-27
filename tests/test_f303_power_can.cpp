#include "balancing/power/PowerCan.hpp"
#include "balancing/app/PowerBoardService.hpp"
#include <cstdlib>
#include <iostream>

using namespace balancing;
using namespace balancing::protocol;
using namespace balancing::power;
using State = balancing::power::PowerState;
#define CHECK(x) do { if (!(x)) { std::cerr << "failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while(false)

// 人工fixture。実機電圧や時間の設定ではない。
constexpr PowerConfig cfg{true,1.0F,2000,10000,3000,3000};
PowerInputs input(std::uint64_t t, float bus=47.0F, bool confirmed=false) {
    PowerInputs in{};
    in.estopOk=true; in.observationValid=true; in.protectionOk=true;
    in.batteryVoltageV=48.0F; in.driveBusVoltageV=bus;
    in.observedAtUs=t; in.mainConfirmed=confirmed;
    return in;
}
CanFrame request(NodeId src, PowerRequestKind kind, std::uint16_t id) {
    auto f=encodePowerRequest({src,kind,id}); CHECK(f); return *f;
}
PowerCanRx batch(std::initializer_list<CanFrame> fs) {
    PowerCanRx r{}; for(auto f:fs) { CHECK(r.count<r.capacity); r.frames[r.count++]=f; }
    return r;
}
void reply(const PowerCanOutput& o, std::size_t i, NodeId dst,
           PowerRequestKind kind, std::uint16_t id, ResultCode code, WirePowerState state) {
    CHECK(i<o.replyCount);
    auto r=decodePowerResult(o.replies[i]); CHECK(r);
    CHECK(r->destination==dst && r->kind==kind && r->requestId==id &&
          r->result==code && r->state==state);
}
void report(const PowerCanOutput& o, WirePowerState state, std::uint16_t sequence) {
    CHECK(o.status);
    auto s=decodePowerStatus(*o.status); CHECK(s);
    CHECK(s->state==state && s->sequence==sequence && s->flags==0 && s->faultBits==0);
}
int main() {
    {
        // F446実装の受信サービスへそのままワイヤフレームを渡す。
        PowerController controller(cfg); PowerCan can; PowerBoardService f446;
        auto o=can.step(controller,input(0,0),{},0);
        report(o,WirePowerState::Fault,0);
        CHECK(f446.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*o.status,0,0));
        CHECK(!f446.snapshot(0).ready && f446.snapshot(0).faultObserved);
        f446.takeObservedFaults(0);
        o=can.step(controller,input(1000,0),batch({request(NodeId::F446,PowerRequestKind::On,0x1234)}),1000);
        reply(o,0,NodeId::F446,PowerRequestKind::On,0x1234,ResultCode::Accepted,WirePowerState::Precharge);
        report(o,WirePowerState::Precharge,1);
        CHECK(f446.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*o.status,1000,1000));
        CHECK(!f446.snapshot(1000).ready);
        for(std::uint64_t t=2000;t<=4000;t+=1000) {
            o=can.step(controller,input(t),{},t);
            CHECK(!o.status && o.power.state==State::Precharge);
        }
        CHECK(o.power.mainRequested && o.power.prechargeRequested);
        o=can.step(controller,input(5000,47,true),{},5000);
        report(o,WirePowerState::On,2);
        CHECK(f446.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*o.status,5000,5000));
        CHECK(f446.snapshot(5000).ready);
        o=can.step(controller,input(6000),batch({request(NodeId::Pi,PowerRequestKind::On,7)}),6000);
        reply(o,0,NodeId::Pi,PowerRequestKind::On,7,ResultCode::AlreadyInState,WirePowerState::On);
        for(std::uint64_t t=7000;t<105000;t+=1000) {
            o=can.step(controller,input(t),{},t); CHECK(!o.status);
        }
        o=can.step(controller,input(105000),{},105000);
        report(o,WirePowerState::On,3);
        CHECK(f446.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*o.status,105000,105000));
        CHECK(f446.snapshot(604999).ready && !f446.snapshot(605000).ready);
        o=can.step(controller,input(106000),batch({*encodeEmergencyStop(NodeId::F446)}),106000);
        CHECK(!o.emergency && !o.power.mainRequested && !o.power.prechargeRequested);
        report(o,WirePowerState::Fault,4);
        CHECK(f446.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*o.status,106000,106000));
        CHECK(f446.snapshot(106000).faultObserved && !f446.snapshot(106000).ready);
    }
    for(bool reverse:{false,true}) {
        PowerController c(cfg,ResetCause::BatteryConnection); PowerCan can;
        CHECK(can.step(c,input(0,0),{},0).power.state==State::Precharge);
        auto on=request(NodeId::Pi,PowerRequestKind::On,1);
        auto off=request(NodeId::F446,PowerRequestKind::Off,2);
        auto o=can.step(c,input(1000),reverse?batch({off,on}):batch({on,off}),1000);
        CHECK(o.power.state==State::Off && o.replyCount==2);
        reply(o,reverse?1:0,NodeId::Pi,PowerRequestKind::On,1,ResultCode::SafetyRejected,WirePowerState::Off);
        reply(o,reverse?0:1,NodeId::F446,PowerRequestKind::Off,2,ResultCode::Accepted,WirePowerState::Off);
        CHECK(can.step(c,input(2000),{},2000).power.state==State::Off);
        o=can.step(c,input(3000),batch({on,off,*encodeEmergencyStop(NodeId::Pi)}),3000);
        CHECK(o.power.state==State::Fault && !o.emergency);
        reply(o,0,NodeId::Pi,PowerRequestKind::On,1,ResultCode::SafetyRejected,WirePowerState::Fault);
        reply(o,1,NodeId::F446,PowerRequestKind::Off,2,ResultCode::SafetyRejected,WirePowerState::Fault);
    }
    {
        // 受信不正、自己反射、未割当IDは無視。欠落時のONは保存しない。
        PowerController c(cfg); PowerCan can;
        auto on=request(NodeId::Pi,PowerRequestKind::On,8);
        auto wrong=on; wrong.dlc=1;
        auto standard=on; standard.extended=false;
        auto id=on; id.id=0x200;
        auto o=can.step(c,input(0),batch({wrong,standard,id,*encodeEmergencyStop(NodeId::F303)}),0);
        CHECK(o.power.state==State::Fault && o.replyCount==0);
        auto lost=batch({on}); lost.lost=true;
        o=can.step(c,input(1000),lost,1000);
        reply(o,0,NodeId::Pi,PowerRequestKind::On,8,ResultCode::SafetyRejected,WirePowerState::Fault);
        CHECK(can.step(c,input(2000),{},2000).power.state==State::Fault);
        auto overflow=batch({on}); overflow.count=PowerCanRx::capacity+1;
        o=can.step(c,input(3000),overflow,3000);
        CHECK(o.power.state==State::Fault && o.replyCount==1);
        o=can.step(c,input(4000),batch({on,request(NodeId::F446,PowerRequestKind::On,9)}),4000);
        reply(o,0,NodeId::Pi,PowerRequestKind::On,8,ResultCode::Accepted,WirePowerState::Precharge);
        reply(o,1,NodeId::F446,PowerRequestKind::On,9,ResultCode::Busy,WirePowerState::Precharge);
        auto estop=input(5000); estop.estopOk=false;
        o=can.step(c,estop,{},5000);
        CHECK(o.emergency && decodeEmergencyStop(*o.emergency)==NodeId::F303);
        CHECK(o.power.state==State::Off && !o.power.mainRequested && !o.power.prechargeRequested);
        estop.observedAtUs=6000;
        CHECK(!can.step(c,estop,{},6000).emergency);
        CHECK(can.step(c,input(7000),{},7000).power.state==State::Precharge);
    }
    {
        // CAN通信欠落のみで独立動作を止めない。既知OFFは欠落時にも優先。
        PowerController c(cfg,ResetCause::BatteryConnection); PowerCan can;
        PowerCanRx lost{}; lost.lost=true;
        CHECK(can.step(c,input(0,0),lost,0).power.state==State::Precharge);
        lost=batch({request(NodeId::F446,PowerRequestKind::Off,3)});lost.lost=true;
        auto o=can.step(c,input(1000),lost,1000);
        CHECK(o.power.state==State::Off);
        reply(o,0,NodeId::F446,PowerRequestKind::Off,3,ResultCode::Accepted,WirePowerState::Off);
    }
    {
        // 連番の周回と遅延時に過去の報告をまとめて生成しない。
        PowerController c; PowerCan can;
        for(std::uint64_t i=0;i<=65536;++i) {
            auto o=can.step(c,{}, {},i*100000);
            report(o,WirePowerState::Fault,static_cast<std::uint16_t>(i));
        }
        auto o=can.step(c,{}, {},6600000000ULL);
        report(o,WirePowerState::Fault,1);
        CHECK(!can.step(c,{}, {},6600000001ULL).status);
    }
    std::cout<<"F303 PowerCan request arbitration, periodic reports and F446 peer PASS\n";
}
