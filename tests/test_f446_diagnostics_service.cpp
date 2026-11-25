#include "balancing/app/DiagnosticsService.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <array>
using namespace balancing;
int main()
{
    std::array<DiagnosticSlot,2> important{};
    std::array<DiagnosticSlot,1> normal{};
    DiagnosticsService diag{important.data(),important.size(),normal.data(),normal.size(),71};
    const auto wrap=(std::uint64_t{1}<<32)*1000;
    FaultRecord fault{FaultFlags::BusOff,FaultFlags::BusOff,FaultFlags::None,wrap+1000};
    assert(diag.pushStateTransition(RobotState::Disarmed,RobotState::Fault,wrap));
    assert(!diag.pushStateTransition(RobotState::Fault,RobotState::Disarmed,wrap+1));
    assert(diag.summary().normalLost==1);
    auto n=diag.nextPending();assert(n && n->event.kind==DiagnosticKind::StateTransition);
    assert(diag.onAccepted(*n,{1,71}));
    assert(diag.pushFault(fault,wrap+1000));
    auto f=diag.nextPending();assert(f && f->event.kind==DiagnosticKind::Fault && f->frameIndex==0);
    assert(f->event.fault.occurredAtMs==1 && f->event.fault.eventSequence==f->event.sequence);
    assert(!diag.onAccepted(*f,{2,72}));
    assert(diag.onAccepted(*f,{2,71}));
    assert(!diag.nextPending());
    assert(diag.onTerminal({1,71},CanTxOutcome::Sent)); // 正常の既受付分
    assert(diag.summary().importantPending==1 && diag.summary().normalPending==0);
    assert(!diag.onTerminal({2,72},CanTxOutcome::Sent));
    assert(diag.onTerminal({2,71},CanTxOutcome::TransmitError));
    assert(!diag.onTerminal({2,71},CanTxOutcome::Sent));
    assert(!diag.onAccepted(*f,{2,71}));
    assert(diag.onAccepted(*f,{3,71}));
    assert(diag.onTerminal({3,71},CanTxOutcome::Sent));
    for (std::uint8_t i=1;i<3;++i) {
        auto p=diag.nextPending();assert(p && p->frameIndex==i);
        assert(diag.onAccepted(*p,{std::uint64_t(3+i),71}));
        assert(diag.summary().importantPending==1);
        assert(diag.onTerminal({std::uint64_t(3+i),71},CanTxOutcome::Sent));
    }
    assert(diag.summary().importantPending==0 && !diag.nextPending());
    diag.updateStatus({wrap+2000,RobotState::Fault,fault});
    diag.updateStatus({wrap+3000,RobotState::Fault,fault});
    assert(diag.summary().importantPending==1);
    fault.subsequentFlags=FaultFlags::MotorFault;
    diag.updateStatus({wrap+4000,RobotState::Fault,fault});
    assert(diag.summary().importantPending==2);
    assert(!diag.pushFault(fault,wrap+5000));
    assert(diag.summary().importantLost==1);
    diag.updateStatus({wrap+6000,RobotState::Disarmed,{}});
    assert(diag.summary().latest->state==RobotState::Disarmed);
    diag.updateStatus({wrap+5000,RobotState::Fault,fault});
    assert(diag.summary().latest->state==RobotState::Disarmed);
    DiagnosticsService unset{};
    unset.updateStatus({wrap+7000,RobotState::Fault,fault});
    assert(unset.summary().importantLost==1 && unset.summary().latest->fault);
}
