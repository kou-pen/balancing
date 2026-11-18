#include "balancing/app/MotorService.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>

namespace {

using namespace balancing;

// 実機設定と分けたCAN IDおよび仮想時刻 [us]。
constexpr std::uint8_t hostId = 0xa6;
constexpr std::uint64_t armTimeoutUs = 1000;
constexpr Rs06BusConfig motorConfig{{11, 12, 13, 14}, hostId};

bool hasFlag(FaultFlags mask, FaultFlags flag)
{
    return (static_cast<unsigned>(mask) & static_cast<unsigned>(flag)) != 0;
}

BMotorCanFrame report(std::uint8_t motorId, unsigned mode = 2, unsigned type = 2,
                      unsigned faults = 0, std::uint8_t host = hostId)
{
    BMotorCanFrame frame{};
    frame.extended = true;
    frame.dlc = 8;
    frame.id = (type << 24) | (mode << 22) | (faults << 16) |
               (static_cast<unsigned>(motorId) << 8) | host;
    return frame;
}

struct Fixture {
    MotorService motors{motorConfig};
    RobotSequence robot{{armTimeoutUs, true}};

    RobotSequenceResult cycle(std::uint64_t nowUs, OperatorEvents operations = {})
    {
        RobotSequenceInput input{};
        input.operations = operations;
        input.interlocks = {true, true, true, true, true, true, MechanismSafety::Safe};
        input.rs06 = motors.rs06ArmFeedback(nowUs);
        input.observedFaults = motors.takeObservedFaults(nowUs);
        const auto output = robot.update(input, nowUs);
        motors.applySequence(output, nowUs);
        return output;
    }

    Rs06TxRequest request(std::size_t index) const
    {
        auto candidate = motors.pendingRs06Requests()[index];
        assert(candidate);
        return *candidate;
    }

    void tx(Rs06TxToken token, Rs06TxResult result, std::uint64_t atUs)
    {
        motors.onRs06TxResult(token, result, atUs);
    }

    void send(const Rs06TxRequest& request, std::uint64_t accepted, std::uint64_t completed)
    {
        tx(request.token, Rs06TxResult::Accepted, accepted);
        tx(request.token, Rs06TxResult::Completed, completed);
    }

    void stopAtStartup()
    {
        for (const auto& candidate : motors.pendingRs06Requests()) {
            assert(candidate && candidate->command == Rs06Command::Stop);
            send(*candidate, 5, 10);
        }
        assert(!motors.rs06ArmFeedback(10).stopPending);
    }

    ArmSessionId startArm()
    {
        stopAtStartup();
        const auto arming = cycle(20, OperatorEvents{true});
        assert(arming.state == RobotState::Arming);
        return arming.armSession;
    }
};

void testStopTransmissionAndResetFeedbackAreSeparate()
{
    Fixture f;
    assert(f.cycle(0, OperatorEvents{true}).state == RobotState::Disarmed);
    const auto stop = f.request(0);
    assert(stop.command == Rs06Command::Stop);
    assert(stop.frame.extended && stop.frame.dlc == 8);
    assert(stop.frame.id == ((4U << 24) | (hostId << 8) | 11U));
    for (auto value : stop.frame.data) {
        assert(value == 0);  // 通常停止は故障解除を含めない。
    }
    f.tx(stop.token, Rs06TxResult::Busy, 2);
    f.tx(stop.token, Rs06TxResult::Completed, 3);
    assert(f.request(0).token.serial == stop.token.serial);
    f.tx(stop.token, Rs06TxResult::Accepted, 4);
    assert(!f.motors.pendingRs06Requests()[0]);
    f.tx(stop.token, Rs06TxResult::Completed, 3);
    assert(f.motors.rs06Status(4)[0].stopPending);
    f.tx(stop.token, Rs06TxResult::Completed, 5);
    f.motors.onRs06Frame(report(11, 0), 5);
    assert(!f.motors.rs06Status(5)[0].stopPending);
    assert(f.motors.rs06ArmFeedback(5).stopPending);  // 残る3軸は独立。
    assert(!f.motors.rs06Status(5)[0].disableConfirmed);
    f.motors.onRs06Frame(report(11, 0, 24), 6);
    assert(!f.motors.rs06Status(6)[0].disableConfirmed);
    f.motors.onRs06Frame(report(11, 0), 7);
    assert(f.motors.rs06Status(7)[0].disableConfirmed);
    assert(f.motors.rs06Status(30006)[0].feedbackFresh);
    assert(!f.motors.rs06Status(30007)[0].feedbackFresh);
    assert(!f.motors.rs06Status(30007)[0].disableConfirmed);
    assert(!f.motors.pendingRs06Requests()[0]);
}

void testEnableRequiresFourNewRunningReports()
{
    Fixture f;
    const auto session = f.startArm();
    const auto lastAxis = f.request(3);
    assert(lastAxis.command == Rs06Command::Enable);
    assert(lastAxis.frame.id == ((3U << 24) | (hostId << 8) | 14U));
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto enable = f.request(axis);
        f.send(enable, 22, 30);
        f.motors.onRs06Frame(report(enable.token.motorId), 31);
    }

    f.tx(lastAxis.token, Rs06TxResult::Accepted, 22);
    f.motors.onRs06Frame(report(14), 29);
    assert(f.cycle(29).state == RobotState::Arming);
    f.tx(lastAxis.token, Rs06TxResult::Completed, 30);
    f.motors.onRs06Frame(report(14), 30);
    f.motors.onRs06Frame(report(14, 2, 24), 31);
    f.motors.onRs06Frame(report(14, 2, 2, 0, 0xff), 32);
    f.motors.onRs06Frame(report(15), 33);
    f.motors.onRs06Frame(report(14, 3), 34);
    auto badDlc = report(14);
    badDlc.dlc = 7;
    f.motors.onRs06Frame(badDlc, 35);
    auto standard = report(14);
    standard.extended = false;
    f.motors.onRs06Frame(standard, 36);
    assert(f.cycle(36).state == RobotState::Arming);

    f.motors.onRs06Frame(report(14), 38);
    assert(f.motors.rs06ArmFeedback(37).axes[3] == Rs06ArmStatus::Pending);
    const auto ready = f.cycle(38);
    assert(ready.state == RobotState::Balancing && ready.armSession == session);
    for (const auto& candidate : f.motors.pendingRs06Requests()) {
        assert(!candidate);  // 有効化の終了後はここから駆動フレームを出さない。
    }
}

void testDisarmRetiresOldEnableBeforeSendingStopForThatAxis()
{
    Fixture f;
    const auto oldSession = f.startArm();
    const auto oldEnable = f.request(0);
    f.tx(oldEnable.token, Rs06TxResult::Accepted, 22);
    assert(f.cycle(23, OperatorEvents{false, true}).state == RobotState::Disarmed);

    const auto partialStops = f.motors.pendingRs06Requests();
    assert(!partialStops[0]);
    for (std::size_t axis = 1; axis < partialStops.size(); ++axis) {
        const auto stop = f.request(axis);
        assert(stop.command == Rs06Command::Stop);
        f.send(stop, 24, 25);
    }
    assert(f.cycle(26, OperatorEvents{true}).state == RobotState::Disarmed);

    f.tx(oldEnable.token, Rs06TxResult::Completed, 27);
    const auto firstStop = f.request(0);
    assert(firstStop.command == Rs06Command::Stop);
    assert(firstStop.token.serial != oldEnable.token.serial);
    f.tx(firstStop.token, Rs06TxResult::Accepted, 28);
    f.tx(firstStop.token, Rs06TxResult::RetryableFailure, 29);
    const auto retryStop = f.request(0);
    assert(retryStop.token.serial != firstStop.token.serial);
    f.tx(retryStop.token, Rs06TxResult::Accepted, 30);
    f.tx(firstStop.token, Rs06TxResult::Completed, 31);
    assert(f.motors.rs06ArmFeedback(31).stopPending);
    f.tx(retryStop.token, Rs06TxResult::Completed, 32);
    f.motors.onRs06Frame(report(11), 33);  // 旧セッションの運転報告が遅れて届く。
    const auto newArm = f.cycle(34, OperatorEvents{true});
    assert(newArm.state == RobotState::Arming && newArm.armSession != oldSession);
    f.tx(oldEnable.token, Rs06TxResult::Completed, 36);
    auto newEnable = f.request(0);
    f.tx(newEnable.token, Rs06TxResult::Accepted, 37);
    f.motors.onRs06Frame(report(11), 39);
    // 主ループで受信を先に処理しても、イベントの記録時刻で順序を判定できる。
    f.tx(newEnable.token, Rs06TxResult::Completed, 38);
    assert(f.motors.rs06ArmFeedback(39).axes[0] == Rs06ArmStatus::Confirmed);
    assert(f.cycle(39).state == RobotState::Arming);
}

void testDeadlineIncludesBackpressureAndRejectsExactBoundary()
{
    Fixture waiting;
    waiting.startArm();
    auto first = waiting.request(0);
    waiting.tx(first.token, Rs06TxResult::Busy, 21);
    waiting.tx(first.token, Rs06TxResult::RetryableFailure, 22);
    assert(waiting.request(0).token.serial != first.token.serial);
    assert(waiting.cycle(20 + armTimeoutUs - 1).state == RobotState::Arming);
    assert(waiting.cycle(20 + armTimeoutUs).state == RobotState::Fault);
    assert(waiting.robot.currentFault()->primaryCause == FaultFlags::ArmTimeout);

    Fixture simultaneous;
    simultaneous.startArm();
    for (const auto& request : simultaneous.motors.pendingRs06Requests()) {
        assert(request);
        simultaneous.send(*request, 23, 24);
        simultaneous.motors.onRs06Frame(report(request->token.motorId), 20 + armTimeoutUs);
    }
    assert(simultaneous.cycle(20 + armTimeoutUs).state == RobotState::Fault);
    assert(simultaneous.robot.currentFault()->primaryCause == FaultFlags::ArmTimeout);
}

void testFourthAxisRetriesWhenMailboxFrees()
{
    Fixture f;
    f.startArm();
    const auto requests = f.motors.pendingRs06Requests();
    // CAN1の3メールボックスを占有させ、4軸目だけ未受付のまま保持する。
    for (std::size_t axis = 0; axis < 3; ++axis) {
        assert(requests[axis]);
        f.tx(requests[axis]->token, Rs06TxResult::Accepted, 21);
    }
    assert(requests[3]);
    auto fourth = requests[3]->token;
    f.tx(fourth, Rs06TxResult::Busy, 22);
    auto remaining = f.motors.pendingRs06Requests();
    assert(!remaining[0] && !remaining[1] && !remaining[2]);
    assert(remaining[3] && remaining[3]->token.serial == fourth.serial);
    assert(f.cycle(22).state == RobotState::Arming);

    f.tx(requests[0]->token, Rs06TxResult::Completed, 23);
    f.tx(fourth, Rs06TxResult::Accepted, 24);
    for (std::size_t axis = 1; axis < 4; ++axis) {
        f.tx(requests[axis]->token, Rs06TxResult::Completed, 25);
    }
    for (const auto& request : requests) {
        f.motors.onRs06Frame(report(request->token.motorId), 26);
    }
    assert(f.cycle(26).state == RobotState::Balancing);
}

void testFailureAndBusOffReachRobotSequence()
{
    Fixture enableFailure;
    enableFailure.startArm();
    auto enable = enableFailure.request(2);
    enableFailure.tx(enable.token, Rs06TxResult::Failed, 21);
    assert(enableFailure.cycle(21).state == RobotState::Fault);
    assert(enableFailure.robot.currentFault()->primaryCause == FaultFlags::EnableFailed);
    for (const auto& request : enableFailure.motors.pendingRs06Requests()) {
        assert(request && request->command == Rs06Command::Stop);
    }
    auto stop = enableFailure.request(2);
    enableFailure.tx(stop.token, Rs06TxResult::Failed, 22);
    enableFailure.send(enableFailure.request(2), 23, 24);
    assert(enableFailure.cycle(25).state == RobotState::Fault);
    assert(hasFlag(enableFailure.robot.currentFault()->subsequentFlags, FaultFlags::ExternalFault));

    Fixture communicationFailure;
    communicationFailure.startArm();
    auto occupied = communicationFailure.request(1);
    communicationFailure.tx(occupied.token, Rs06TxResult::Accepted, 21);
    communicationFailure.motors.setCan1BusOff(true);
    for (const auto& request : communicationFailure.motors.pendingRs06Requests()) {
        assert(!request);
    }
    communicationFailure.motors.setCan1BusOff(false);
    communicationFailure.motors.onRs06Frame(report(11, 2, 24, 1), 22);
    communicationFailure.motors.onRs06Frame(report(11), 23);
    assert(communicationFailure.cycle(24).state == RobotState::Fault);
    auto fault = communicationFailure.robot.currentFault();
    assert(fault && fault->primaryCause == FaultFlags::BusOff);
    assert(hasFlag(fault->triggerFlags, FaultFlags::MotorFault));
    assert(!communicationFailure.motors.pendingRs06Requests()[1]);
    communicationFailure.tx(occupied.token, Rs06TxResult::RetryableFailure, 25);
    assert(communicationFailure.request(1).command == Rs06Command::Stop);
}

void testInvalidAxisConfigurationBlocksSending()
{
    for (const auto& ids : {std::array<std::uint8_t, 4>{11, 12, 0, 14},
                            std::array<std::uint8_t, 4>{11, 12, 11, 14}}) {
        MotorService motors{{ids, hostId}};
        assert(motors.rs06ArmFeedback(0).stopPending);
        assert(motors.takeObservedFaults(0) == FaultFlags::ExternalFault);
        for (const auto& request : motors.pendingRs06Requests()) {
            assert(!request);
        }
    }
}

} // namespace

int main()
{
    testStopTransmissionAndResetFeedbackAreSeparate();
    testEnableRequiresFourNewRunningReports();
    testDisarmRetiresOldEnableBeforeSendingStopForThatAxis();
    testDeadlineIncludesBackpressureAndRejectsExactBoundary();
    testFourthAxisRetriesWhenMailboxFrees();
    testFailureAndBusOffReachRobotSequence();
    testInvalidAxisConfigurationBlocksSending();
    std::puts("PASS: MotorService four-axis RS06 ARM and stop with simulated CAN");
}
