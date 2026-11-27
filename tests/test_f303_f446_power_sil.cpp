#include "balancing/power/application.hpp"
#include "balancing/app/PowerBoardService.hpp"
#include "balancing/app/RobotSequence.hpp"
#include <cstdlib>
#include <iostream>

namespace {
using namespace balancing;
using namespace balancing::protocol;
namespace f303 = balancing::power;
#define CHECK(x) do { if (!(x)) { std::cerr << "failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while(false)

// 独立した二つの周期を単調な仮想時計で接続する。実機定数ではない。
constexpr f303::ApplicationConfig config{
    {true,1.0F,2000,10000,3000,3000},
    {true,{0,4095,0.125F,0},{0,4095,0.125F,0},{0,4095,0.25F,-10},
     40,52,40,54,10,2000,3000},
    f303::ResetCause::BatteryConnection,true};

struct Bench {
    f303::Application power{config};
    PowerBoardService receiver;
    RobotSequence robot{{200000,true}};
    bool pre{},main{},dropStatus{},estopOk{true},hardwareFault{},rejectApply{};
    std::uint16_t current{40};
    std::uint64_t now{};
    std::uint64_t lastStatus{};
    unsigned statusCount{};
    f303::PowerCanRx pending{};
    f303::CycleResult powerResult{};
    RobotSequenceResult robotResult{};
    OperatorEvents events{};
    FaultFlags peerFaults{FaultFlags::None};

    static bool apply(void* ctx,const f303::BoardOutput& out) noexcept {
        auto& b=*static_cast<Bench*>(ctx);
        b.pre=!b.rejectApply&&out.prechargeOn;
        b.main=!b.rejectApply&&out.mainOn;
        return !b.rejectApply;
    }
    void send(const CanFrame& frame) {
        CHECK(pending.count<pending.capacity);
        pending.frames[pending.count++]=frame;
    }
    void request(NodeId source,PowerRequestKind kind,std::uint16_t id) {
        const auto frame=encodePowerRequest({source,kind,id});
        CHECK(frame);send(*frame);
    }
    void tick() {
        // ADCと導通確認は論理fixture。前周期の適用出力から観測を生成し、
        // 主電源成立を要求前に先取りしない。RC時定数を模擬する物理モデルではない。
        f303::CycleInput in{};
        in.pendingTicks=1;in.nowUs=now;
        in.adc={true,384,static_cast<std::uint16_t>(pre||main?376:0),current,now};
        in.estopOk=estopOk;in.hardwareFault=hardwareFault;
        in.shutdownPending=!estopOk||hardwareFault;
        in.mainConfirmed=main;in.rx=pending;pending={};
        powerResult=power.runCycle(in,apply,this);
        if(powerResult.tx.status&&!dropStatus) {
            CHECK(receiver.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,
                    *powerResult.tx.status,now,now));
            lastStatus=now;++statusCount;
        }
        if(powerResult.tx.emergency)
            peerFaults=peerFaults|FaultFlags::EmergencyStop;
        // F446は2msごとに更新。ほかの安全条件は合成値。
        if(now%2000==0) {
            RobotSequenceInput input{};
            input.operations=events;events={};
            input.observedFaults=receiver.takeObservedFaults(now)|peerFaults;
            peerFaults=FaultFlags::None;
            input.interlocks={true,true,receiver.snapshot(now).ready,true,true,true,MechanismSafety::Safe};
            input.rs06.session=robotResult.armSession;
            input.rs06.axes.fill(Rs06ArmStatus::Confirmed);
            robotResult=robot.update(input,now);
            // ここはモータ送信ではなく実RobotSequenceの駆動要求だけを確認。
            if(!input.interlocks.powerReady)
                CHECK(robotResult.desiredMotorState==DesiredMotorState::StopAll);
        }
        now+=1000;
    }
    void until(std::uint64_t inclusive) {
        while(now<=inclusive)tick();
    }
    void arm() {
        CHECK(now%2000==0);events.arm=true;tick();
        CHECK(robotResult.state==RobotState::Arming);
        tick();tick();
        CHECK(robotResult.state==RobotState::Balancing);
        CHECK(robotResult.desiredMotorState==DesiredMotorState::DriveAllowed);
        tick();
    }
    void clear() {
        CHECK(now%2000==0);events.clearFault=true;events.clearFaultAuthorized=true;
        events.arm=true; // 同周期ARMは復旧操作に流用しない。
        tick();
        CHECK(robotResult.state==RobotState::Disarmed);
        CHECK(robotResult.desiredMotorState==DesiredMotorState::StopAll);
        tick();
    }
};
void start(Bench& b) {
    b.events.arm=true;
    b.until(0);
    CHECK(b.robotResult.state==RobotState::Disarmed);
    b.until(5000);
    CHECK(b.main&&!b.pre && b.receiver.snapshot(5000).ready);
    CHECK(b.robotResult.state==RobotState::Disarmed);
    b.arm();
}
void checkFault(const Bench& b) {
    CHECK(b.robotResult.state==RobotState::Fault);
    CHECK(b.robotResult.desiredMotorState==DesiredMotorState::StopAll);
}
void recover(Bench& b) {
    CHECK(b.now%2000==0);
    b.request(NodeId::Pi,PowerRequestKind::On,42);
    b.tick();
    CHECK(b.powerResult.tx.replyCount==1);
    const auto response=decodePowerResult(b.powerResult.tx.replies[0]);
    CHECK(response&&response->result==ResultCode::Accepted&&response->state==WirePowerState::Precharge);
    CHECK(!b.receiver.snapshot(b.now-1000).ready);
    checkFault(b);
    // 充電完了は独立した観測を経由する。電源ONだけではRobotSequenceは復帰しない。
    b.until(b.now+6000);
    CHECK(b.now%2000==0);
    CHECK(b.main && b.receiver.snapshot(b.now-1000).ready);
    checkFault(b);
    b.clear();b.arm();
}
}
int main() {
    {
        Bench b;start(b);
        // 両基板が独立した周期を持つ状態で、実F303の持続過電流→FAULTを接続。
        b.current=81;b.until(b.now+3000);
        CHECK(!b.pre&&!b.main);checkFault(b);
        CHECK(b.robot.currentFault()->primaryCause==FaultFlags::ExternalFault);
        b.events.clearFault=true;b.events.clearFaultAuthorized=true;b.tick();
        checkFault(b);b.tick();
        b.current=40;recover(b);
    }
    {
        Bench b;start(b);
        // Piの通常OFFはF303に保持され、F446は電源ready消失でFAULT。
        b.request(NodeId::Pi,PowerRequestKind::Off,7);b.tick();b.tick();
        CHECK(!b.pre&&!b.main);checkFault(b);
        CHECK(b.robot.currentFault()->primaryCause==FaultFlags::InterlockLost);
        b.until(b.now+199000);CHECK(!b.main);checkFault(b);
        recover(b);
    }
    {
        Bench b;start(b);
        b.estopOk=false;b.tick();b.tick();
        CHECK(!b.main&&!b.pre);checkFault(b);
        CHECK(b.robot.currentFault()->primaryCause==FaultFlags::EmergencyStop);
        b.estopOk=true;b.until(b.now+5000);
        CHECK(b.main&&b.receiver.snapshot(b.now-1000).ready);
        checkFault(b);b.clear();b.arm();
    }
    {
        Bench b;start(b);
        // F303の100ms報告を途中から全て欠落。F303自身は独立でONを維持。
        b.until(105000);const auto last=b.lastStatus;
        CHECK(last==104000 && b.statusCount==3);
        b.dropStatus=true;
        // 最後の報告の正確な500ms境界を受信サービスで確認。
        const auto expires=last+500000;
        b.until(expires-1000);
        CHECK(b.receiver.snapshot(expires-1).ready);
        CHECK(b.robotResult.state==RobotState::Balancing);
        CHECK(!b.receiver.snapshot(expires).ready);
        b.until(expires+(expires%2000?1000:0));
        CHECK(b.main);checkFault(b);
        CHECK(b.robot.currentFault()->primaryCause==FaultFlags::InterlockLost);
        b.dropStatus=false;b.until(b.now+105000);
        CHECK(b.main&&b.receiver.snapshot(b.now-1000).ready);checkFault(b);
        if(b.now%2000)b.tick();
        b.clear();b.arm();
    }
    {
        Bench b;start(b);
        // 出力適用拒否の周期には状態報告が出ない。次の周期のFAULTで停止。
        b.rejectApply=true;b.tick();
        CHECK(!b.powerResult.completed&&!b.powerResult.tx.status&&!b.main);
        b.rejectApply=false;b.tick();b.tick();b.tick();checkFault(b);
        recover(b);
    }
    {
        Bench b;start(b);
        // F446起点のCAN ESTOPは受信反射なし。停止解除だけでは復帰しない。
        const auto stop=encodeEmergencyStop(NodeId::F446);CHECK(stop);
        b.send(*stop);b.tick();
        CHECK(!b.powerResult.tx.emergency&&!b.main&&!b.pre);
        checkFault(b);b.tick();
        b.until(b.now+199000);checkFault(b);CHECK(!b.main);
        recover(b);
    }
    {
        Bench b;
        b.until(5000);
        b.events.arm=true;b.tick();
        CHECK(b.robotResult.state==RobotState::Arming);
        b.request(NodeId::F446,PowerRequestKind::Off,1);
        b.tick();b.tick();
        CHECK(!b.main&&!b.pre);checkFault(b);
    }
    std::cout<<"F303/F446 dual-rate power safety sequence SIL PASS\n";
}
