#include "balancing/app/RemoteControlService.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
using namespace balancing;
int main()
{
    RemoteControlService unverified;
    assert(!unverified.update(RobotState::Disarmed,0));
    // 人工値。実機の操作速度・死活期限ではない。
    OperatorInputConfig config{10000,4000,2,3,0.05f,0.05f,10,20,true};
    RemoteControlService service{config};
    auto accept=[&](std::uint64_t at,float v,float yaw,bool fresh=true) {
        return service.acceptDecoded(OperatorSource::UsbDualSense,{true,at,v,yaw},fresh,at);
    };
    auto tick=[&](RobotState state,std::uint64_t at) {
        const auto result=service.update(state,at);assert(result);return *result;
    };
    assert(accept(1000,0,0));
    assert(!tick(RobotState::Disarmed,1000).operator_ready);
    assert(accept(5000,0,0));
    assert(tick(RobotState::Disarmed,5000).operator_ready);
    assert(!service.selectSource(OperatorSource::CanRemote,RobotState::Arming));
    assert(!service.selectSource(OperatorSource::CanRemote,RobotState::Balancing));
    assert(!service.selectSource(OperatorSource::CanRemote,RobotState::Fault));
    assert(accept(6000,1,2));
    assert(!accept(6000,1,2));
    assert(!accept(5999,1,2));
    assert(!accept(7000,std::numeric_limits<float>::quiet_NaN(),0));
    assert(!accept(7000,3,0));
    assert(!accept(7000,1,1,false));
    assert(tick(RobotState::Arming,6000).speed_m_s==0);
    auto driven=tick(RobotState::Balancing,10000);
    assert(driven.speed_m_s>0 && driven.yaw_rate_rad_s>0);
    auto lost=tick(RobotState::Balancing,16001);
    assert(!lost.source_fresh && !lost.operator_ready && lost.speed_m_s<driven.speed_m_s);
    assert(accept(17000,1,2));
    assert(!tick(RobotState::Balancing,17000).operator_ready);
    assert(accept(18000,0,0));
    assert(accept(22000,0,0));
    assert(tick(RobotState::Balancing,22000).operator_ready);
    assert(tick(RobotState::Fault,23000).speed_m_s==0);
    assert(service.selectSource(OperatorSource::CanRemote,RobotState::Disarmed));
    assert(!service.acceptDecoded(OperatorSource::UsbDualSense,{true,24000,0,0},true,24000));
    auto switched=service.update(RobotState::Disarmed,24000);
    assert(switched && switched->source==OperatorSource::CanRemote && !switched->operator_ready);
    RobotSequence sequence{{100000,true}};
    assert(service.acceptDecoded(OperatorSource::CanRemote,{true,25000,0,0},true,25000));
    assert(service.acceptDecoded(OperatorSource::CanRemote,{true,29000,0,0},true,29000));
    assert(tick(RobotState::Disarmed,29000).operator_ready);
    const auto result=sequence.update({},29000);
    assert(result.state==RobotState::Disarmed && result.desiredMotorState==DesiredMotorState::StopAll);
    assert(tick(result.state,29000).speed_m_s==0);
}
