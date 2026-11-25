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
    BOperatorInputConfig config{10000,4000,2,3,0.05f,0.05f,10,20,true};
    RemoteControlService service{config};
    BOperatorInput c{};
    assert(b_operator_init(&c,&config));
    auto accept=[&](std::uint64_t at,float v,float yaw,bool fresh=true) {
        BOperatorSample sample{true,at,v,yaw};
        const auto result=service.acceptDecoded(B_OPERATOR_USB_DUALSENSE,sample,fresh,at);
        assert(result==b_operator_accept(&c,B_OPERATOR_USB_DUALSENSE,v,yaw,fresh,at,at));
        return result;
    };
    auto tick=[&](RobotState state,BMode mode,std::uint64_t at) {
        BOperatorTarget expected{};
        auto actual=service.update(state,at);
        assert(actual.has_value()==b_operator_tick(&c,mode,at,&expected));
        assert(actual && actual->source_fresh==expected.source_fresh &&
            actual->operator_ready==expected.operator_ready &&
            actual->waiting_neutral==expected.waiting_neutral &&
            actual->speed_m_s==expected.speed_m_s &&
            actual->yaw_rate_rad_s==expected.yaw_rate_rad_s);
        return *actual;
    };
    assert(accept(1000,0,0));
    assert(!tick(RobotState::Disarmed,B_MODE_DISARMED,1000).operator_ready);
    assert(accept(5000,0,0));
    assert(tick(RobotState::Disarmed,B_MODE_DISARMED,5000).operator_ready);
    assert(!service.selectSource(B_OPERATOR_CAN_REMOTE,RobotState::Arming));
    assert(!service.selectSource(B_OPERATOR_CAN_REMOTE,RobotState::Balancing));
    assert(!service.selectSource(B_OPERATOR_CAN_REMOTE,RobotState::Fault));
    assert(accept(6000,1,2));
    assert(!accept(6000,1,2));
    assert(!accept(5999,1,2));
    assert(!accept(7000,std::numeric_limits<float>::quiet_NaN(),0));
    assert(!accept(7000,3,0));
    assert(!accept(7000,1,1,false));
    assert(tick(RobotState::Arming,B_MODE_DISARMED,6000).speed_m_s==0);
    auto driven=tick(RobotState::Balancing,B_MODE_BALANCING,10000);
    assert(driven.speed_m_s>0 && driven.yaw_rate_rad_s>0);
    auto lost=tick(RobotState::Balancing,B_MODE_BALANCING,16001);
    assert(!lost.source_fresh && !lost.operator_ready && lost.speed_m_s<driven.speed_m_s);
    assert(accept(17000,1,2));
    assert(!tick(RobotState::Balancing,B_MODE_BALANCING,17000).operator_ready);
    assert(accept(18000,0,0));
    assert(accept(22000,0,0));
    assert(tick(RobotState::Balancing,B_MODE_BALANCING,22000).operator_ready);
    assert(tick(RobotState::Fault,B_MODE_FAULT,23000).speed_m_s==0);
    assert(service.selectSource(B_OPERATOR_CAN_REMOTE,RobotState::Disarmed));
    assert(!service.acceptDecoded(B_OPERATOR_USB_DUALSENSE,{true,24000,0,0},true,24000));
    auto switched=service.update(RobotState::Disarmed,24000);
    assert(switched && switched->source==B_OPERATOR_CAN_REMOTE && !switched->operator_ready);
}
