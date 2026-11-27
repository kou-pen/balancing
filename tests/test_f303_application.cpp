#include "balancing/power/application.hpp"
#include "balancing/app/PowerBoardService.hpp"
#include <cstdlib>
#include <iostream>

using namespace balancing;
using namespace balancing::power;
using namespace balancing::protocol;
#define CHECK(x) do { if (!(x)) { std::cerr << "failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while(false)

// 人工の校正・電圧・時刻。実機設定を有効化するものではない。
constexpr ApplicationConfig config{
    {true,1.0F,2000,10000,3000,3000},
    {true,{0,4095,0.125F,0},{0,4095,0.125F,0},{0,4095,0.25F,-10},
     40,52,40,54,10,2000,3000},
    ResetCause::BatteryConnection,true};
struct MockBoard {
    bool reject{false};
    bool pre{false};
    bool main{false};
    unsigned calls{0};
};
bool apply(void* context,const BoardOutput& out) noexcept {
    auto& b=*static_cast<MockBoard*>(context);
    ++b.calls;
    // 適用直前のIRQ発生を模擬。失敗時は既に両出力を遮断済み。
    b.pre=!b.reject && out.prechargeOn;
    b.main=!b.reject && out.mainOn;
    return !b.reject;
}
CycleInput input(std::uint64_t t,std::uint16_t bus=376,std::uint16_t current=40) {
    CycleInput i{};
    i.pendingTicks=1; i.nowUs=t;
    i.adc={true,384,bus,current,t};
    i.estopOk=true; i.hardwareFault=false; i.shutdownPending=false;
    return i;
}
void request(CycleInput& i,NodeId source,PowerRequestKind kind,std::uint16_t id) {
    auto frame=encodePowerRequest({source,kind,id}); CHECK(frame);
    i.rx.frames[i.rx.count++]=*frame;
}
void check(const CycleResult& r,const MockBoard& b,balancing::power::PowerState state,bool pre,bool main) {
    CHECK(r.ran && r.completed && r.watchdogEligible);
    CHECK(r.power.state==state && r.board.accepted && r.board.prechargeOn==pre && r.board.mainOn==main);
    CHECK(b.pre==pre && b.main==main && r.tx.power.state==state);
}
void noTx(const CycleResult& r) {
    CHECK(!r.completed && !r.watchdogEligible && !r.tx.status && !r.tx.emergency && r.tx.replyCount==0);
}
void status(const CycleResult& r,WirePowerState expected) {
    CHECK(r.tx.status);
    auto decoded=decodePowerStatus(*r.tx.status); CHECK(decoded && decoded->state==expected);
}
void start(Application& a,MockBoard& b) {
    check(a.runCycle(input(0,0),apply,&b),b,balancing::power::PowerState::Precharge,true,false);
    for(std::uint64_t t=1000;t<3000;t+=1000)
        check(a.runCycle(input(t),apply,&b),b,balancing::power::PowerState::Precharge,true,false);
    check(a.runCycle(input(3000),apply,&b),b,balancing::power::PowerState::Precharge,true,true);
    auto i=input(4000); i.mainConfirmed=true;
    check(a.runCycle(i,apply,&b),b,balancing::power::PowerState::On,false,true);
}
int main() {
    {
        // タイマ通知なしは実行・出力・送信・WDG更新なし。
        Application a(config); MockBoard b; auto i=input(0); i.pendingTicks=0;
        auto r=a.runCycle(i,apply,&b); CHECK(!r.ran && b.calls==0); noTx(r);
    }
    {
        // 既定値は明示ONでも投入不可。正常なFAULT周期は更新可能。
        Application a; MockBoard b; auto i=input(0);
        request(i,NodeId::Pi,PowerRequestKind::On,1);
        auto r=a.runCycle(i,apply,&b); check(r,b,balancing::power::PowerState::Fault,false,false);
        status(r,WirePowerState::Fault);
        CHECK(r.tx.replyCount==1 && decodePowerResult(r.tx.replies[0])->result==ResultCode::SafetyRejected);
    }
    {
        // F446の実受信サービスに、適用完了後の状態候補を接続。
        Application a(config); MockBoard b; PowerBoardService peer;
        auto r=a.runCycle(input(0,0),apply,&b);
        check(r,b,balancing::power::PowerState::Precharge,true,false); status(r,WirePowerState::Precharge);
        CHECK(peer.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*r.tx.status,0,0));
        CHECK(!peer.snapshot(0).ready);
        for(std::uint64_t t=1000;t<=3000;t+=1000) r=a.runCycle(input(t),apply,&b);
        check(r,b,balancing::power::PowerState::Precharge,true,true);
        auto i=input(4000); i.mainConfirmed=true;
        r=a.runCycle(i,apply,&b); check(r,b,balancing::power::PowerState::On,false,true); status(r,WirePowerState::On);
        CHECK(peer.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*r.tx.status,4000,4000));
        CHECK(peer.snapshot(4000).ready);
        for(std::uint64_t t=5000;t<=7000;t+=1000) r=a.runCycle(input(t,376,81),apply,&b);
        check(r,b,balancing::power::PowerState::Fault,false,false); status(r,WirePowerState::Fault);
        CHECK(peer.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,*r.tx.status,7000,7000));
        CHECK(!peer.snapshot(7000).ready);
        i=input(8000,0); request(i,NodeId::F446,PowerRequestKind::On,2);
        check(a.runCycle(i,apply,&b),b,balancing::power::PowerState::Precharge,true,false);
    }
    {
        // 物理停止では従来の自動復帰、CAN OFF/ESTOPは保持する。
        Application a(config); MockBoard b; start(a,b);
        auto i=input(5000);i.estopOk=false;i.shutdownPending=true;
        auto r=a.runCycle(i,apply,&b);check(r,b,balancing::power::PowerState::Off,false,false);
        CHECK(r.tx.emergency);
        check(a.runCycle(input(6000,0),apply,&b),b,balancing::power::PowerState::Precharge,true,false);
        i=input(7000);i.shutdownPending=true;i.shutdownCommand=PowerCommand::Off;
        request(i,NodeId::Pi,PowerRequestKind::On,3);
        r=a.runCycle(i,apply,&b);check(r,b,balancing::power::PowerState::Off,false,false);
        CHECK(decodePowerResult(r.tx.replies[0])->result==ResultCode::SafetyRejected);
        check(a.runCycle(input(8000),apply,&b),b,balancing::power::PowerState::Off,false,false);
        i=input(9000);i.shutdownPending=true;i.shutdownCommand=PowerCommand::EmergencyStop;
        check(a.runCycle(i,apply,&b),b,balancing::power::PowerState::Fault,false,false);
    }
    for(int fault=0;fault<5;++fault) {
        Application a(config); MockBoard b; start(a,b);
        auto i=input(5000);
        if(fault==0) i.pendingTicks=2;
        if(fault==1) i.nowUs=4000;
        if(fault==2) b.reject=true;
        if(fault==3) i.shutdownPending=true;
        if(fault==4) i.hardwareFault=true;
        request(i,NodeId::Pi,PowerRequestKind::On,4);
        auto r=a.runCycle(i,apply,&b);
        CHECK(r.power.state==balancing::power::PowerState::Fault && !b.pre && !b.main);
        if(fault<3) noTx(r); else {CHECK(r.completed && r.watchdogEligible);status(r,WirePowerState::Fault);}
        b.reject=false;
        r=a.runCycle(input(6000),apply,&b);
        check(r,b,balancing::power::PowerState::Fault,false,false);
        if(fault<3) status(r,WirePowerState::Fault);
    }
    {
        // 主電源成立の候補が出た直後に適用拒否。ON報告も応答も外に出ない。
        Application a(config);MockBoard b;
        for(std::uint64_t t=0;t<=3000;t+=1000) a.runCycle(input(t,t?376:0),apply,&b);
        auto i=input(4000);i.mainConfirmed=true;request(i,NodeId::Pi,PowerRequestKind::On,5);
        b.reject=true;
        auto r=a.runCycle(i,apply,&b);noTx(r);CHECK(r.power.state==balancing::power::PowerState::Fault&&!b.pre&&!b.main);
        b.reject=false;r=a.runCycle(input(5000),apply,&b);status(r,WirePowerState::Fault);
    }
    {
        Application a(config);MockBoard b;
        auto r=a.runCycle(input(0),nullptr,nullptr); noTx(r);
        CHECK(r.power.state==balancing::power::PowerState::Fault && b.calls==0);
        r=a.runCycle(input(1000),apply,&b);status(r,WirePowerState::Fault);
    }
    std::cout<<"F303 application full logical cycle and F446 peer PASS\n";
}
