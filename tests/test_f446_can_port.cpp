#include "balancing/app/MotorService.hpp"
#include "balancing/communication/can/CanPort.hpp"

#include <algorithm>

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

// バスごとに独立したレジスタとHALハンドルを割り当てる。
struct PortFixture {
    CAN_TypeDef registers{};
    CAN_HandleTypeDef hal{&registers, HAL_CAN_STATE_LISTENING};
    CanPort port{hal};

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

    void pushRx(std::uint32_t identifier, std::uint32_t dlc = 8)
    {
        auto& fifo = registers.RF0R;
        assert(fifo.count < 3u);
        registers.sFIFOMailBox[fifo.count++] = {identifier, dlc, 0x44332211u, 0x88776655u};
    }
};

// 旧RS06型との対応は結合試験でのみ使う。製品の配送・調停は後続で接続する。
TxToken encodeToken(Rs06TxToken token)
{
    return {token.serial, token.motorId};
}

CanTxSubmitResult submitRs06(CanPort& port, const Rs06TxRequest& request, std::uint64_t nowUs)
{
    CanFrame frame{};
    frame.id = request.frame.id;
    frame.dlc = request.frame.dlc;
    frame.extended = request.frame.extended;
    frame.data=request.frame.data;
    return port.trySend(frame, encodeToken(request.token), nowUs);
}

Rs06TxResult adaptSubmitResult(CanTxSubmitResult result)
{
    if (result == CanTxSubmitResult::Accepted) {
        return Rs06TxResult::Accepted;
    }
    return result == CanTxSubmitResult::Busy ? Rs06TxResult::Busy : Rs06TxResult::Failed;
}

Rs06TxResult adaptOutcome(CanTxOutcome outcome)
{
    if (outcome == CanTxOutcome::Sent) {
        return Rs06TxResult::Completed;
    }
    return outcome == CanTxOutcome::TransmitError ? Rs06TxResult::Failed : Rs06TxResult::RetryableFailure;
}

struct Fixture : PortFixture {
    MotorService motors{{{11, 12, 13, 14}, host}};
    RobotSequence sequence{{1000, true}};

    void rx(std::uint8_t motor, std::uint32_t dlc = 8, std::uint32_t control = CAN_RI0R_IDE)
    {
        pushRx((extId(motor) << 3u) | control, dlc);
    }

    FaultFlags drain()
    {
        CanEvent event{};
        auto faults = FaultFlags::None;
        for (std::size_t i = 0; i < CanPort::kMaxEventsPerPoll && port.popEvent(event); ++i) {
            switch (event.kind) {
            case CanEventKind::Rx: {
                motors.onRs06Frame(event.frame, event.atUs);
                break;
            }
            case CanEventKind::Tx:
                motors.onRs06TxResult({static_cast<std::uint8_t>(event.token.origin), event.token.serial},
                                     adaptOutcome(event.outcome), event.atUs);
                break;
            case CanEventKind::BusOff:
                motors.setCan1BusOff(true);
                break;
            case CanEventKind::BusRecovered:
                motors.setCan1BusOff(false);
                break;
            case CanEventKind::RxLost:
                faults = faults | FaultFlags::ExternalFault;
                break;
            }
        }
        return faults;
    }

    RobotState cycle(std::uint64_t nowUs, bool arm = false)
    {
        RobotSequenceInput in{};
        in.operations.arm = arm;
        const auto fault = drain();
        in.observedFaults = fault | motors.takeObservedFaults(nowUs);
        in.rs06 = motors.rs06ArmFeedback(nowUs);
        in.interlocks = {true, true, true, true, true, true, MechanismSafety::Safe};
        const auto out = sequence.update(in, nowUs);
        motors.applySequence(out, nowUs);
        for (const auto& request : motors.pendingRs06Requests()) {
            if (request) {
                motors.onRs06TxResult(request->token, adaptSubmitResult(submitRs06(port, *request, nowUs)),
                                     nowUs);
            }
        }
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
    PortFixture f;
    primask = 1;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const auto request = control(static_cast<std::uint8_t>(11 + axis), 100 + axis);
        assert(submitRs06(f.port, request, 7) == CanTxSubmitResult::Accepted);
        assert(primask == 1);
        const auto& tx = f.registers.sTxMailBox[axis];
        assert(tx.TIR == ((request.frame.id << 3u) | CAN_TI0R_IDE | CAN_TI0R_TXRQ));
        assert(tx.TDTR == 8 && tx.TDLR == 0xde && tx.TDHR == 0xa5000000u);
        f.occupy(axis);
    }
    primask = 0;
    const auto next = control(14, 104);
    f.finish(1);
    assert(submitRs06(f.port, next, 8) == CanTxSubmitResult::Busy);
    f.port.captureInterrupt(10);
    f.port.captureInterrupt(11);  // W1C済みの完了を再通知しない。
    assert(submitRs06(f.port, next, 12) == CanTxSubmitResult::Busy);
    CanEvent event;
    assert(f.port.popEvent(event) && event.kind == CanEventKind::Tx);
    assert(event.mailbox == 1 && event.token.origin == 12 && event.token.serial == 101);
    assert(event.outcome == CanTxOutcome::Sent && event.atUs == 10);
    assert(!f.port.popEvent(event));
    assert(submitRs06(f.port, next, 13) == CanTxSubmitResult::Accepted && primask == 0);
}

void testAbortAndBusOffKeepAcceptedTokensUntilTerminal()
{
    PortFixture f;
    auto first = control(11, 8);
    assert(submitRs06(f.port, first, 1) == CanTxSubmitResult::Accepted);
    f.occupy(0);
    auto second = control(12, 9);
    assert(submitRs06(f.port, second, 1) == CanTxSubmitResult::Accepted);
    f.occupy(1);
    assert(!f.port.requestAbort({7, 11}));
    assert(f.port.requestAbort(encodeToken(first.token)));
    assert(f.registers.TSR.aborts == CAN_TSR_ABRQ0);
    f.registers.ESR = CAN_ESR_BOFF;
    f.port.captureInterrupt(4);
    assert(f.registers.TSR.aborts == (CAN_TSR_ABRQ0 | (CAN_TSR_ABRQ0 << 8u)));
    assert(submitRs06(f.port, control(14, 10), 4) == CanTxSubmitResult::Busy);
    // 復旧通知が先に届いても、上位から見てバスオフの発生自体は消えない。
    f.registers.ESR = 0;
    f.finish(0, 0);
    f.finish(1, CAN_TSR_TERR0);
    f.port.captureInterrupt(5);
    CanEvent e;
    assert(f.port.popEvent(e) && e.kind == CanEventKind::BusOff && e.atUs == 4);
    assert(f.port.popEvent(e) && e.kind == CanEventKind::BusRecovered && e.atUs == 5);
    assert(f.port.popEvent(e) && e.token.serial == 8 && e.outcome == CanTxOutcome::NotTransmitted);
    assert(f.port.popEvent(e) && e.token.serial == 9 && e.outcome == CanTxOutcome::TransmitError);
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

void testTwoBusesKeepFramesAndTokensSeparate()
{
    PortFixture can1;
    PortFixture can2;
    const CanFrame extended{0x1fffffffu, {}, 8, true};
    const CanFrame standard{0x7ffu, {}, 2, false};
    // 上位が同じ発行元・シリアルを両バスで使っても、照合範囲は各ポート内。
    const TxToken token{41, 2};
    assert(can1.port.trySend(extended, token, 2) == CanTxSubmitResult::Accepted);
    assert(can2.port.trySend(standard, token, 2) == CanTxSubmitResult::Accepted);
    assert(can1.port.trySend(extended, token, 2) == CanTxSubmitResult::Busy);
    assert(can2.port.trySend(standard, token, 2) == CanTxSubmitResult::Busy);
    assert(can1.registers.sTxMailBox[0].TIR == ((extended.id << 3u) | CAN_TI0R_IDE | CAN_TI0R_TXRQ));
    assert(can2.registers.sTxMailBox[0].TIR == ((standard.id << 21u) | CAN_TI0R_TXRQ));

    can1.occupy(0);
    can2.occupy(0);
    assert(can2.port.requestAbort(token));
    assert(can1.registers.TSR.aborts == 0u && can2.registers.TSR.aborts == CAN_TSR_ABRQ0);

    can1.finish(0, CAN_TSR_ALST0);
    can2.finish(0, 0);
    can1.pushRx((extended.id << 3u) | CAN_RI0R_IDE, 8);
    can2.pushRx(standard.id << 21u, 2);
    can1.port.captureInterrupt(3);
    CanEvent event{};
    assert(!can2.port.popEvent(event));
    can2.port.captureInterrupt(4);

    assert(can1.port.popEvent(event) && event.kind == CanEventKind::Rx && event.atUs == 3);
    assert(event.frame.extended && event.frame.id == extended.id && event.frame.dlc == 8);
    assert(event.frame.data[0] == 0x11 && event.frame.data[7] == 0x88);
    assert(can1.port.popEvent(event) && event.kind == CanEventKind::Tx && event.atUs == 3);
    assert(event.token == token && event.outcome == CanTxOutcome::ArbitrationLost);
    assert(!can1.port.popEvent(event));

    assert(can2.port.popEvent(event) && event.kind == CanEventKind::Rx && event.atUs == 4);
    assert(!event.frame.extended && event.frame.id == standard.id && event.frame.dlc == 2);
    assert(can2.port.popEvent(event) && event.kind == CanEventKind::Tx && event.atUs == 4);
    assert(event.token == token && event.outcome == CanTxOutcome::NotTransmitted);
    assert(!can2.port.popEvent(event));
}

void testOverflowAndBusOffAreLocal()
{
    PortFixture can1;
    PortFixture can2;
    const CanFrame frame{0x123, {}, 0, false};
    for (std::size_t box = 0; box < 3; ++box) {
        const TxToken token{box + 1, 0};
        for (auto* bus : {&can1, &can2}) {
            assert(bus->port.trySend(frame, token, 1) == CanTxSubmitResult::Accepted);
            bus->occupy(box);
        }
    }

    // 21件を7回のIRQに分けて入れ、片側だけでキューをあふれさせる。
    for (std::uint64_t atUs = 2; atUs < 9; ++atUs) {
        for (unsigned packet = 0; packet < 3; ++packet) {
            can1.pushRx(frame.id << 21u, 0);
        }
        can1.port.captureInterrupt(atUs);
    }
    can1.registers.ESR = CAN_ESR_BOFF;
    can1.port.captureInterrupt(9);
    assert(can2.registers.TSR.aborts == 0);
    can1.registers.ESR = 0;
    for (std::size_t box = 0; box < 3; ++box) {
        can1.finish(box, 0);
        can2.finish(box);
    }
    can1.port.captureInterrupt(10);
    can2.port.captureInterrupt(11);
    CanEvent event{};
    assert(can1.port.popEvent(event) && event.kind == CanEventKind::BusOff);
    assert(can1.port.popEvent(event) && event.kind == CanEventKind::BusRecovered);
    assert(can1.port.popEvent(event) && event.kind == CanEventKind::RxLost);

    std::uint64_t terminals = 0;
    unsigned received = 0;
    while (can1.port.popEvent(event)) {
        if (event.kind == CanEventKind::Rx) {
            ++received;
        } else {
            assert(event.kind == CanEventKind::Tx && event.outcome == CanTxOutcome::NotTransmitted);
            assert(event.token.serial == ++terminals);
        }
    }
    assert(received == 13 && terminals == 3);
    for (std::uint64_t serial = 1; serial <= 3; ++serial) {
        assert(can2.port.popEvent(event) && event.kind == CanEventKind::Tx);
        assert(event.token.serial == serial && event.outcome == CanTxOutcome::Sent);
    }
    assert(!can2.port.popEvent(event));
    assert(can2.port.trySend(frame, {4, 0}, 12) == CanTxSubmitResult::Accepted);
}

void testInvalidFramesAndUnstartedPort()
{
    PortFixture can;
    const TxToken token{1, 0};
    assert(can.port.trySend({0x800, {}, 0, false}, token, 1) == CanTxSubmitResult::Invalid);
    assert(can.port.trySend({0x20000000, {}, 0, true}, token, 1) == CanTxSubmitResult::Invalid);
    assert(can.port.trySend({0, {}, 9, false}, token, 1) == CanTxSubmitResult::Invalid);
    assert(can.port.trySend({}, {}, 1) == CanTxSubmitResult::Invalid);
    can.hal.State = HAL_CAN_STATE_READY;
    assert(can.port.trySend({}, token, 1) == CanTxSubmitResult::NotReady);
    for (const auto& mailbox : can.registers.sTxMailBox) {
        assert((mailbox.TIR & CAN_TI0R_TXRQ) == 0u);
    }

    can.hal.Instance = nullptr;
    assert(can.port.trySend({}, token, 1) == CanTxSubmitResult::NotReady);
    assert(!can.port.requestAbort(token));
    can.port.captureInterrupt(1);
    CanEvent event{};
    assert(!can.port.popEvent(event));
}

} // namespace

int main()
{
    testMailboxIdentityUntilEventConsumed();
    testAbortAndBusOffKeepAcceptedTokensUntilTerminal();
    testRxOverflowStillDeliversAllTxTerminals();
    testArmThroughPortRejectsUnorderedAndMalformedReports();
    testTerminalTxErrorStopsArmingImmediately();
    testTwoBusesKeepFramesAndTokensSeparate();
    testOverflowAndBusOffAreLocal();
    testInvalidFramesAndUnstartedPort();
    std::puts("PASS: F446 dual CAN ports and RS06 ARM regression");
}
