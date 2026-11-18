#include "balancing/app/Rs06Can1Link.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>

namespace {
std::uint32_t primask{0};
}

std::uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1; }
void __enable_irq() { primask = 0; }

namespace {
using namespace balancing;

constexpr std::uint8_t host = 0x5a;
constexpr std::uint32_t extId(std::uint8_t motor)
{
    return (2u << 24u) | (2u << 22u) | (static_cast<unsigned>(motor) << 8u) | host;
}

struct Fixture {
    CAN_TypeDef registers{};
    CAN_HandleTypeDef hal{&registers, HAL_CAN_STATE_LISTENING};
    F446Can1Port port{hal};
    MotorService motors{{{11, 12, 13, 14}, host}};
    RobotSequence sequence{{1000, true}};

    void occupy(std::size_t box)
    {
        assert(registers.sTxMailBox[box].TIR & CAN_TI0R_TXRQ);
        registers.TSR.bits &= ~(CAN_TSR_TME0 << box);
    }

    void finish(std::size_t box, std::uint32_t result = CAN_TSR_TXOK0)
    {
        assert((registers.TSR.bits & (CAN_TSR_TME0 << box)) == 0u);
        registers.TSR.bits |= (CAN_TSR_TME0 << box) | ((CAN_TSR_RQCP0 | result) << (8u * box));
        registers.sTxMailBox[box].TIR &= ~CAN_TI0R_TXRQ;
    }

    void rx(std::uint8_t motor, std::uint32_t dlc = 8, std::uint32_t control = CAN_RI0R_IDE)
    {
        auto& fifo = registers.RF0R;
        assert(fifo.count < 3u);
        registers.sFIFOMailBox[fifo.count++] = {(extId(motor) << 3u) | control, dlc,
                                              0x44332211u, 0x88776655u};
    }

    RobotState cycle(std::uint64_t nowUs, bool arm = false)
    {
        RobotSequenceInput in{};
        in.operations.arm = arm;
        const auto fault = drainRs06Can1(port, motors);
        in.observedFaults = fault | motors.takeObservedFaults(nowUs);
        in.rs06 = motors.rs06ArmFeedback(nowUs);
        in.interlocks = {true, true, true, true, true, true, MechanismSafety::Safe};
        const auto out = sequence.update(in, nowUs);
        motors.applySequence(out, nowUs);
        submitRs06Can1(port, motors, nowUs);
        return out.state;
    }

    void bootStops()
    {
        assert(cycle(0) == RobotState::Disarmed);
        for (std::size_t box = 0; box < 3; ++box) {
            occupy(box);
            finish(box);
        }
        port.captureInterrupt(1);
        assert(cycle(2) == RobotState::Disarmed);
        occupy(0);
        finish(0);
        port.captureInterrupt(3);
        assert(cycle(4) == RobotState::Disarmed);
        assert(!motors.rs06ArmFeedback(4).stopPending);
    }
};

Rs06TxRequest control(std::uint8_t motor, std::uint64_t serial)
{
    const auto frame = Rs06Codec::encodeControl(Rs06Command::Enable, motor, host);
    assert(frame);
    auto req = Rs06TxRequest{{motor, serial}, Rs06Command::Enable, *frame};
    req.frame.data[0] = 0xde;
    req.frame.data[7] = 0xa5;
    return req;
}

void testMailboxIdentityUntilEventConsumed()
{
    Fixture f;
    primask = 1;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const auto request = control(static_cast<std::uint8_t>(11 + axis), 100 + axis);
        assert(f.port.trySend(request, 7) == Rs06TxResult::Accepted);
        assert(primask == 1);
        const auto& tx = f.registers.sTxMailBox[axis];
        assert(tx.TIR == ((request.frame.id << 3u) | CAN_TI0R_IDE | CAN_TI0R_TXRQ));
        assert(tx.TDTR == 8 && tx.TDLR == 0xde && tx.TDHR == 0xa5000000u);
        f.occupy(axis);
    }
    primask = 0;
    const auto next = control(14, 104);
    f.finish(1);
    assert(f.port.trySend(next, 8) == Rs06TxResult::Busy);
    f.port.captureInterrupt(10);
    f.port.captureInterrupt(11);  // W1C済みの完了を再通知しない。
    assert(f.port.trySend(next, 12) == Rs06TxResult::Busy);
    Can1Event event;
    assert(f.port.popEvent(event) && event.kind == Can1EventKind::Tx);
    assert(event.mailbox == 1 && event.token.motorId == 12 && event.token.serial == 101);
    assert(event.txResult == Rs06TxResult::Completed && event.atUs == 10);
    assert(!f.port.popEvent(event));
    assert(f.port.trySend(next, 13) == Rs06TxResult::Accepted && primask == 0);
}

void testAbortAndBusOffKeepAcceptedTokensUntilTerminal()
{
    Fixture f;
    auto first = control(11, 8);
    assert(f.port.trySend(first, 1) == Rs06TxResult::Accepted);
    f.occupy(0);
    auto second = control(12, 9);
    assert(f.port.trySend(second, 1) == Rs06TxResult::Accepted);
    f.occupy(1);
    assert(!f.port.requestAbort({11, 7}));
    assert(f.port.requestAbort(first.token));
    assert(f.registers.TSR.aborts == CAN_TSR_ABRQ0);
    f.registers.ESR = CAN_ESR_BOFF;
    f.port.captureInterrupt(4);
    assert(f.registers.TSR.aborts == (CAN_TSR_ABRQ0 | (CAN_TSR_ABRQ0 << 8u)));
    assert(f.port.trySend(control(14, 10), 4) == Rs06TxResult::Busy);
    // 復旧通知が先に届いても、上位から見てバスオフの発生自体は消えない。
    f.registers.ESR = 0;
    f.finish(0, 0);
    f.finish(1, CAN_TSR_TERR0);
    f.port.captureInterrupt(5);
    Can1Event e;
    assert(f.port.popEvent(e) && e.kind == Can1EventKind::BusOff && e.atUs == 4);
    assert(f.port.popEvent(e) && e.kind == Can1EventKind::BusRecovered && e.atUs == 5);
    assert(f.port.popEvent(e) && e.token.serial == 8 && e.txResult == Rs06TxResult::RetryableFailure);
    assert(f.port.popEvent(e) && e.token.serial == 9 && e.txResult == Rs06TxResult::Failed);
    assert(!f.port.popEvent(e));
}

void testRxOverflowStillDeliversAllTxTerminals()
{
    Fixture f;
    assert(f.cycle(0) == RobotState::Disarmed);  // 3軸の起動時停止が受付済み。
    for (std::size_t box = 0; box < 3; ++box) f.occupy(box);
    for (std::uint64_t n = 1; n <= 5; ++n) {
        for (unsigned frame = 0; frame < 3; ++frame) f.rx(11);
        f.port.captureInterrupt(n);
    }
    f.registers.RF0R.flags = CAN_RF0R_FOVR0;
    for (std::size_t box = 0; box < 3; ++box) f.finish(box);
    f.port.captureInterrupt(6);
    assert(f.registers.RF0R.flags == 0);
    assert(f.cycle(7) == RobotState::Fault);
    assert(f.sequence.currentFault()->primaryCause == FaultFlags::ExternalFault);
    const auto statuses = f.motors.rs06Status(7);
    assert(!statuses[0].stopPending && !statuses[1].stopPending && !statuses[2].stopPending);
    assert(statuses[3].stopPending);
}

void testArmThroughPortRejectsUnorderedAndMalformedReports()
{
    Fixture f;
    f.bootStops();
    assert(f.cycle(5, true) == RobotState::Arming);
    for (std::size_t box = 0; box < 3; ++box) {
        f.occupy(box);
        f.finish(box);
        f.rx(static_cast<std::uint8_t>(11 + box));
    }
    f.port.captureInterrupt(6);
    assert(f.cycle(7) == RobotState::Arming);
    const auto sameCapture = f.motors.rs06ArmFeedback(7);
    assert(sameCapture.axes[0] == Rs06ArmStatus::Pending);
    f.occupy(0);
    f.finish(0);
    f.port.captureInterrupt(8);
    f.rx(11);
    f.rx(12);
    f.rx(13);
    f.port.captureInterrupt(9);
    f.rx(14, 12);
    f.rx(14, 8, CAN_RI0R_IDE | CAN_RI0R_RTR);
    f.port.captureInterrupt(10);
    assert(f.cycle(11) == RobotState::Arming);
    f.rx(14);
    f.port.captureInterrupt(12);
    assert(f.cycle(13) == RobotState::Balancing);
}

void testTerminalTxErrorStopsArmingImmediately()
{
    Fixture f;
    f.bootStops();
    assert(f.cycle(5, true) == RobotState::Arming);
    f.occupy(0);
    f.finish(0, CAN_TSR_TERR0);
    f.port.captureInterrupt(6);
    assert(f.cycle(7) == RobotState::Fault);
    assert(f.sequence.currentFault()->primaryCause == FaultFlags::EnableFailed);
    assert(f.motors.rs06ArmFeedback(7).stopPending);
}

} // namespace

int main()
{
    testMailboxIdentityUntilEventConsumed();
    testAbortAndBusOffKeepAcceptedTokensUntilTerminal();
    testRxOverflowStillDeliversAllTxTerminals();
    testArmThroughPortRejectsUnorderedAndMalformedReports();
    testTerminalTxErrorStopsArmingImmediately();
    std::puts("PASS: F446 CAN1 mailbox, RX FIFO and RS06 ARM integration");
}
