#include "balancing/app/PowerBoardService.hpp"
#include "balancing/communication/can/PowerBoardCodec.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace balancing;
namespace {
void testCanStatus()
{
    using namespace protocol;
    PowerBoardService power;
    const auto on = *encodePowerStatus({WirePowerState::On,0x80,0x1234,65535});
    const auto fault = *encodePowerStatus({WirePowerState::Fault,0,0,0});
    // 別バスや別分類からの有効フレームでも電源報告にはならない。
    assert(!power.acceptCanStatus(CanBus::Can1,CanRxTarget::PowerStatus,on,100,100));
    assert(!power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerResult,on,100,100));
    assert(!power.snapshot(100).latest);
    assert(power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,on,100,101));
    const auto first=power.snapshot(101);
    assert(first.ready && first.latest->receivedAtUs==100 && first.latest->wire);
    assert(first.latest->wire->flags==0x80 && first.latest->wire->faultBits==0x1234);
    assert(first.latest->wire->sequence==65535);
    // 外形が合っていても予約領域、列挙値、DLC、IDE、送信元が違えば更新しない。
    for (unsigned kind=0;kind<5;++kind) {
        auto bad=fault;
        switch(kind) {
        case 0: bad.data[7]=1; break;
        case 1: bad.data[0]=4; break;
        case 2: bad.dlc=7; break;
        case 3: bad.extended=false; break;
        case 4: bad.id^=0x4000; break;
        }
        assert(!power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,bad,200,200));
    }
    assert(!power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,fault,100,200));
    assert(!power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,fault,99,200));
    assert(!power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,fault,300,200));
    assert(!power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,fault,200,500200));
    assert(power.snapshot(500099).ready && !power.snapshot(500100).fresh);
    assert(power.snapshot(500100).latest->receivedAtUs==100);
    // 正常なFAULT→ONでも既存の保留FAULTを失わない。連番周回は生値として残す。
    assert(power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,fault,600000,600001));
    assert(power.snapshot(600001).latest->wire->sequence==0);
    assert(power.acceptCanStatus(CanBus::Can2,CanRxTarget::PowerStatus,on,700000,700001));
    assert(!power.snapshot(700001).ready && power.snapshot(700001).faultObserved);
    assert(power.takeObservedFaults(700001)==FaultFlags::ExternalFault);
    assert(power.snapshot(700001).ready);
}
}
int main()
{
    testCanStatus();
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
