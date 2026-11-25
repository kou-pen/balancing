#include "balancing/app/PowerBoardService.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace balancing;
int main()
{
    PowerBoardService power;
    assert(!power.snapshot(0).latest && !power.snapshot(0).ready);
    assert(power.takeObservedFaults(0)==FaultFlags::None);
    assert(power.acceptDecoded({PowerState::Off,0},0));
    assert(power.snapshot(0).fresh && !power.snapshot(0).ready);
    assert(power.acceptDecoded({PowerState::Precharge,100000},100000));
    assert(!power.snapshot(100000).ready);
    assert(power.acceptDecoded({PowerState::On,200000},200000));
    assert(power.snapshot(699999).ready && !power.snapshot(700000).ready);
    assert(!power.snapshot(199999).ready);
    assert(!power.acceptDecoded({PowerState::On,200000},200001));
    assert(!power.acceptDecoded({PowerState::On,199999},200001));
    assert(!power.acceptDecoded({PowerState::On,300000},299999));
    assert(!power.acceptDecoded({static_cast<PowerState>(99),300000},300000));
    assert(!power.acceptDecoded({PowerState::On,300000},800000));
    assert(power.acceptDecoded({PowerState::Fault,300000},300000));
    assert(power.acceptDecoded({PowerState::On,400000},400000));
    assert(!power.snapshot(400000).ready && power.snapshot(400000).faultObserved);
    RobotSequence seq{{100000,true}};
    RobotSequenceInput in{};
    in.observedFaults=power.takeObservedFaults(400000);
    assert(seq.update(in,400000).state==RobotState::Fault);
    assert(power.snapshot(400000).ready);
    in.observedFaults=power.takeObservedFaults(400001);
    assert(seq.update(in,400001).state==RobotState::Fault);
    assert(power.acceptDecoded({PowerState::Fault,500000},500000));
    assert(power.takeObservedFaults(500000)==FaultFlags::ExternalFault);
    assert(power.takeObservedFaults(500001)==FaultFlags::ExternalFault);
}
