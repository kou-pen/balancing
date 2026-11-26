#include "balancing/app/RobotSequence.hpp"

#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {

using namespace balancing;

// 数値は仮想時計の試験専用。実機用のARM期限設定とは独立している。
constexpr std::uint64_t kStartUs = 1000;
constexpr std::uint64_t kTimeoutUs = 80;
constexpr auto kDeadlineUs = kStartUs + kTimeoutUs;
constexpr ArmTimeoutConfig kVerifiedTimeout{kTimeoutUs, true};

// Releaseビルドでも判定を残し、失敗時に条件と行番号を出す。
#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            std::fprintf(stderr, "line %d: %s\n", __LINE__, #condition);        \
            std::exit(EXIT_FAILURE);                                          \
        }                                                                     \
    } while (false)

RobotSequenceInput readyInput()
{
    RobotSequenceInput input;
    input.interlocks = {true, true, true, true, true, true, MechanismSafety::Safe};
    return input;
}

void expect(RobotSequenceResult result, RobotState state, DesiredMotorState motors)
{
    CHECK(result.state == state);
    CHECK(result.desiredMotorState == motors);
}

void expectCause(const RobotSequence& sequence, FaultFlags cause)
{
    const auto record = sequence.currentFault();
    CHECK(record && record->primaryCause == cause);
}

ArmSessionId arm(RobotSequence& sequence, RobotSequenceInput& input,
                 std::uint64_t nowUs = kStartUs)
{
    // 操作イベントは一周期分だけ入力する。後続周期では状態だけを観測する。
    input.operations.arm = true;
    const auto result = sequence.update(input, nowUs);
    input.operations.arm = false;
    expect(result, RobotState::Arming, DesiredMotorState::EnableRs06);
    CHECK(result.armSession != 0);
    return result.armSession;
}

void confirmAll(RobotSequenceInput& input, ArmSessionId session)
{
    // MotorServiceがTx完了後の応答を照合した結果を4軸とも注入する。
    input.rs06.session = session;
    input.rs06.axes.fill(Rs06ArmStatus::Confirmed);
}

void testArmPreconditions()
{
    for (const auto config : {ArmTimeoutConfig{kTimeoutUs, false}, ArmTimeoutConfig{0, true}}) {
        RobotSequence sequence{config};
        auto input = readyInput();
        input.operations.arm = true;
        const auto result = sequence.update(input, kStartUs);
        expect(result, RobotState::Disarmed, DesiredMotorState::StopAll);
        CHECK(result.armSession == 0);
    }

    // セッションを開始する前に、期限が表現可能かを確認する。
    RobotSequence sequence{kVerifiedTimeout};
    auto input = readyInput();
    input.operations.arm = true;
    expect(sequence.update(input, std::numeric_limits<std::uint64_t>::max() - kTimeoutUs + 1),
           RobotState::Disarmed, DesiredMotorState::StopAll);

    RobotSequence interlocks{kVerifiedTimeout};
    input.interlocks.driveLimitsVerified = false;
    expect(interlocks.update(input, kStartUs), RobotState::Disarmed, DesiredMotorState::StopAll);
    input.interlocks.driveLimitsVerified = true;
    input.interlocks.mechanism = MechanismSafety::Unsafe;
    expect(interlocks.update(input, kStartUs + 1), RobotState::Disarmed, DesiredMotorState::StopAll);
    input.interlocks.mechanism = MechanismSafety::Safe;
    input.rs06.stopPending = true;
    expect(interlocks.update(input, kStartUs + 2), RobotState::Disarmed, DesiredMotorState::StopAll);
}

void testEachInterlockAndRunningLoss()
{
    // 旧全系統鮮度モデルを廃止する代わりに、現行の各インターロックが単独で
    // ARMを拒否し、運転中に失われたらFAULTを要求することを確認する。
    constexpr bool SequenceInterlocks::* checks[]={
        &SequenceInterlocks::imuReady, &SequenceInterlocks::motorsReady,
        &SequenceInterlocks::powerReady, &SequenceInterlocks::calibrationVerified,
        &SequenceInterlocks::modelVerified, &SequenceInterlocks::driveLimitsVerified};
    for (auto check:checks) {
        RobotSequence sequence{kVerifiedTimeout};
        auto input=readyInput();
        input.interlocks.*check=false;
        input.operations.arm=true;
        expect(sequence.update(input,kStartUs),RobotState::Disarmed,DesiredMotorState::StopAll);
        input.interlocks.*check=true;
        const auto session=arm(sequence,input,kStartUs+1);
        confirmAll(input,session);
        expect(sequence.update(input,kStartUs+2),RobotState::Balancing,DesiredMotorState::DriveAllowed);
        input.interlocks.*check=false;
        expect(sequence.update(input,kStartUs+3),RobotState::Fault,DesiredMotorState::StopAll);
        expectCause(sequence,FaultFlags::InterlockLost);
    }
}

void testMechanismProofBeforeDrive()
{
    // 初期姿勢範囲の診断とは独立して、現在の機構安全判断が到着するまで駆動を保留する。
    RobotSequence sequence{kVerifiedTimeout};
    auto input = readyInput();
    input.interlocks.mechanism = MechanismSafety::Unverified;
    confirmAll(input, arm(sequence, input));
    expect(sequence.update(input, kStartUs + 1), RobotState::Arming, DesiredMotorState::EnableRs06);
    input.interlocks.mechanism = MechanismSafety::Safe;
    expect(sequence.update(input, kDeadlineUs - 1), RobotState::Balancing,
           DesiredMotorState::DriveAllowed);

    // 運転中に機構安全判断を失った周期から停止する。
    input.interlocks.mechanism = MechanismSafety::Unverified;
    expect(sequence.update(input, kDeadlineUs), RobotState::Fault, DesiredMotorState::StopAll);
    const auto record = sequence.currentFault();
    CHECK(record && record->primaryCause == FaultFlags::MechanismUnsafe);
}

void testSessionAndFourAxisConfirmation()
{
    RobotSequence sequence{kVerifiedTimeout};
    auto input = readyInput();
    const auto firstSession = arm(sequence, input);
    input.operations.disarm = true;
    expect(sequence.update(input, kStartUs + 1), RobotState::Disarmed, DesiredMotorState::StopAll);
    input.operations.disarm = false;
    const auto session = arm(sequence, input, kStartUs + 2);
    CHECK(session != firstSession);

    // 旧セッションが持つ全軸確認や個別失敗の遅着で遷移しない。
    confirmAll(input, firstSession);
    expect(sequence.update(input, kStartUs + 3), RobotState::Arming, DesiredMotorState::EnableRs06);
    input.rs06.axes.back() = Rs06ArmStatus::Failed;
    expect(sequence.update(input, kStartUs + 4), RobotState::Arming, DesiredMotorState::EnableRs06);

    confirmAll(input, session);
    input.rs06.axes.back() = Rs06ArmStatus::Pending;
    const auto pending = sequence.update(input, kDeadlineUs - 1);
    expect(pending, RobotState::Arming, DesiredMotorState::EnableRs06);
    CHECK(pending.armSession == session);
    input.rs06.axes.back() = Rs06ArmStatus::Confirmed;
    expect(sequence.update(input, kDeadlineUs), RobotState::Balancing,
           DesiredMotorState::DriveAllowed);
}

void testAbsoluteDeadline()
{
    for (const auto nowUs : {kDeadlineUs, kDeadlineUs + 1}) {
        RobotSequence sequence{kVerifiedTimeout};
        auto input = readyInput();
        const auto session = arm(sequence, input);
        // 一時的な送信待ちを繰り返しても、元のセッションの期限は動かない。
        input.rs06.session = session;
        expect(sequence.update(input, kStartUs + 20), RobotState::Arming,
               DesiredMotorState::EnableRs06);
        expect(sequence.update(input, kDeadlineUs - 1), RobotState::Arming,
               DesiredMotorState::EnableRs06);
        confirmAll(input, session);
        input.operations.disarm = true;
        expect(sequence.update(input, nowUs), RobotState::Fault, DesiredMotorState::StopAll);
        const auto record = sequence.currentFault();
        CHECK(record && record->primaryCause == FaultFlags::ArmTimeout);
        CHECK(record->occurredAtUs == nowUs);
    }

    // 4軸無応答のままでも、最初の期限で停止に切り替える。
    RobotSequence sequence{kVerifiedTimeout};
    auto input = readyInput();
    arm(sequence, input);
    expect(sequence.update(input, kDeadlineUs), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(sequence, FaultFlags::ArmTimeout);
}

void testDisarmAndFaultPriority()
{
    RobotSequence cancelled{kVerifiedTimeout};
    auto input = readyInput();
    confirmAll(input, arm(cancelled, input));
    input.operations.disarm = true;
    expect(cancelled.update(input, kStartUs + 1), RobotState::Disarmed,
           DesiredMotorState::StopAll);
    CHECK(!cancelled.currentFault());

    RobotSequence lost{kVerifiedTimeout};
    input = readyInput();
    confirmAll(input, arm(lost, input));
    input.interlocks.imuReady = false;
    input.operations.disarm = true;
    expect(lost.update(input, kStartUs + 1), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(lost, FaultFlags::InterlockLost);

    RobotSequence failed{kVerifiedTimeout};
    input = readyInput();
    confirmAll(input, arm(failed, input));
    input.rs06.axes.back() = Rs06ArmStatus::Failed;
    expect(failed.update(input, kStartUs + 1), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(failed, FaultFlags::EnableFailed);
}

void testFaultRecordAndExplicitRecovery()
{
    RobotSequence sequence{kVerifiedTimeout};
    auto input = readyInput();
    auto session = arm(sequence, input);
    input.observedFaults = FaultFlags::EmergencyStop | FaultFlags::BusOff;
    expect(sequence.update(input, kDeadlineUs), RobotState::Fault, DesiredMotorState::StopAll);
    auto record = sequence.currentFault();
    CHECK(record && record->primaryCause == FaultFlags::EmergencyStop);
    CHECK(record->triggerFlags ==
          (FaultFlags::EmergencyStop | FaultFlags::BusOff | FaultFlags::ArmTimeout));

    input.observedFaults = FaultFlags::BusOff | FaultFlags::ExternalFault;
    input.operations = {false, false, true, true};
    expect(sequence.update(input, kDeadlineUs + 1), RobotState::Fault, DesiredMotorState::StopAll);
    record = sequence.currentFault();
    CHECK(record && record->primaryCause == FaultFlags::EmergencyStop);
    CHECK(record->subsequentFlags == FaultFlags::ExternalFault);
    CHECK(record->occurredAtUs == kDeadlineUs);

    input.observedFaults = FaultFlags::None;
    input.operations.clearFaultAuthorized = false;
    expect(sequence.update(input, kDeadlineUs + 2), RobotState::Fault, DesiredMotorState::StopAll);
    input.operations = {true, false, true, true};
    expect(sequence.update(input, kDeadlineUs + 3), RobotState::Disarmed, DesiredMotorState::StopAll);
    CHECK(!sequence.currentFault());
    const auto resolved = sequence.lastResolvedFault();
    CHECK(resolved && resolved->primaryCause == FaultFlags::EmergencyStop);
    CHECK(resolved->triggerFlags == record->triggerFlags);
    CHECK(resolved->subsequentFlags == record->subsequentFlags);

    input.operations = {};
    input.rs06.stopPending = true;
    input.operations.arm = true;
    expect(sequence.update(input, kDeadlineUs + 4), RobotState::Disarmed, DesiredMotorState::StopAll);
    input.rs06.stopPending = false;
    CHECK(arm(sequence, input, kDeadlineUs + 5) != session);

    // 次のFAULTをラッチしても、解除済みの履歴は次の明示解除まで保持する。
    input.interlocks.imuReady = false;
    expect(sequence.update(input, kDeadlineUs + 6), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(sequence, FaultFlags::InterlockLost);
    const auto previous = sequence.lastResolvedFault();
    CHECK(previous && previous->primaryCause == FaultFlags::EmergencyStop);
    input.operations = {false, false, true, true};
    expect(sequence.update(input, kDeadlineUs + 7), RobotState::Fault, DesiredMotorState::StopAll);
    input.interlocks.imuReady = true;
    expect(sequence.update(input, kDeadlineUs + 8), RobotState::Disarmed, DesiredMotorState::StopAll);
    const auto latest = sequence.lastResolvedFault();
    CHECK(latest && latest->primaryCause == FaultFlags::InterlockLost);
}

void testClockAndRunningSafety()
{
    RobotSequence regression{kVerifiedTimeout};
    auto input = readyInput();
    arm(regression, input);
    expect(regression.update(input, kStartUs - 1), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(regression, FaultFlags::ClockRegression);

    // Applicationが検出した周期異常も、駆動許可より先にFAULTへ反映する。
    RobotSequence timing{kVerifiedTimeout};
    input = readyInput();
    confirmAll(input, arm(timing, input));
    expect(timing.update(input, kStartUs + 1), RobotState::Balancing,
           DesiredMotorState::DriveAllowed);
    input.observedFaults = FaultFlags::ControlTiming;
    expect(timing.update(input, kStartUs + 2), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(timing, FaultFlags::ControlTiming);

    // 有効な4軸結果と停止要求が共存したときも、停止を優先する。
    RobotSequence stopping{kVerifiedTimeout};
    input = readyInput();
    confirmAll(input, arm(stopping, input));
    input.rs06.stopPending = true;
    expect(stopping.update(input, kStartUs + 1), RobotState::Fault, DesiredMotorState::StopAll);
    expectCause(stopping, FaultFlags::InterlockLost);
}

} // namespace

int main()
{
    testArmPreconditions();
    testMechanismProofBeforeDrive();
    testEachInterlockAndRunningLoss();
    testSessionAndFourAxisConfirmation();
    testAbsoluteDeadline();
    testDisarmAndFaultPriority();
    testFaultRecordAndExplicitRecovery();
    testClockAndRunningSafety();
    std::puts("PASS: RobotSequence ARM, deadline, DISARM, FAULT and fault history");
    return EXIT_SUCCESS;
}
