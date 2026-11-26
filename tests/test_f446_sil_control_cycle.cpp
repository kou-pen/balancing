#include "balancing/app/RobotSequence.hpp"
#include "balancing/control/controller/BalanceControl.hpp"
#include "balancing/control/controller/MotorCommand.hpp"
#include "balancing/control/estimator/StateEstimator.hpp"
#include "balancing/app/ControlCycle.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <variant>

using namespace balancing;
using namespace std::chrono_literals;

namespace {

// 全ての値は数値SIL専用。実機の半径・LQR K_300・脚CAD・安全閾値ではない。
constexpr float kPi = 3.14159265358979323846f;

StateEstimatorConfig estimatorFixture()
{
    StateEstimatorConfig c{};
    for (auto& leg : c.legs) {
        leg.motor = {{0, 0}, {0, 0}, {1, 1}, 1, 1, true};
        leg.fixture = {0.2f, 0.15f, 0.15f, 0.25f, 0.25f, 0.02f, 0.01f, -1};
    }
    c.wheels = {{{0.1f, 10.0f, 1.0f, true}, {0.1f, 10.0f, -1.0f, true}}};
    return c;
}

BalanceControlConfig controlFixture()
{
    BalanceControlConfig c{};
    c.lqr.verified = true; // ホスト内の数学fixture検査のみを表す。
    c.lqr.max_position_error_m = 0.2f;
    c.lqr.gain[LqrWheelTotal][LqrPitch] = 2.0f;
    c.lqr.gain[LqrWheelTotal][LqrPosition] = 1.0f;
    c.lqr.gain[LqrHipTotal][LqrPitch] = 3.0f;
    c.legPd = {100.0f, 10.0f, 2.0f, 0.5f, 4.0f, true};
    c.yawPi = {3.0f, 4.0f, 2.0f, true};
    c.wheelTorqueLimitNm = {5.0f, 5.0f};
    return c;
}

MotorCommandConfig motorFixture()
{
    // モータ/車輪の換算比と6軸上限はホスト内の人工値。実機定数ではない。
    MotorCommandConfig config{};
    for (auto& axis : config.rs06) {
        axis = {1.5f, 1.0f, 4.0f, true};
    }
    for (auto& axis : config.c620) {
        axis = {0.4f, 1.0f, 4.0f, true};
    }
    return config;
}

struct ObservationFixture {
    // 人工の静止ロータ報告を1msごとに供給する。制御周期が欠落しても
    // センサ報告自体は継続したケースを独立に再現する。
    std::array<C620Motor, 2> wheels{C620Motor{{4000, true}}, C620Motor{{4000, true}}};
    std::optional<std::uint64_t> last{};
    StateEstimatorInput observation(std::uint64_t atUs)
    {
        StateEstimatorInput input{};
        input.imu = {atUs, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, true, true};
        input.motors.sampledAtUs = atUs;
        for (std::size_t i = 0; i < 4; ++i) {
            input.motors.rs06[i].latest = Rs06FeedbackSample{
                Rs06StateReport{static_cast<std::uint8_t>(i + 1), 0xa6,
                                Rs06Report::Status, Rs06Mode::Running, 0,
                                -kPi / 2.0f, 0.0f, 0.0f, 30.0f},
                atUs};
            input.motors.rs06[i].fresh = true;
        }
        auto feed = [&](std::uint64_t stamp) {
            for (std::size_t i = 0; i < 2; ++i) {
                wheels[i].onFeedback(C620Feedback{static_cast<std::uint8_t>(i + 1), 1000, 0, 0, 30}, stamp);
            }
        };
        if (last && atUs > *last) {
            for (auto stamp = *last + 1000; stamp < atUs; stamp += 1000) feed(stamp);
        }
        feed(atUs);
        last = atUs;
        for (std::size_t i = 0; i < 2; ++i) input.motors.c620[i] = wheels[i].feedback(atUs);
        return input;
    }
};

// SILだけで作る仮想安全状態。物理モデルの実機検証結果を主張しない。
// RobotSequenceそのもののロジックをホストで通すための入力であり、
// どのフレームもMotorServiceやCanPortには送らない。
RobotSequenceInput simulatedInterlocks()
{
    RobotSequenceInput input{};
    input.interlocks = {true, true, true, true, true, true, MechanismSafety::Safe};
    return input;
}

void testControllerMathAndReset()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    auto output = estimator.update(samples.observation(2000), 2000us);
    assert(std::holds_alternative<ControlSnapshot>(output));
    auto snapshot = std::get<ControlSnapshot>(output);
    assert(snapshot.fixtureOnly);
    BalanceControl control{controlFixture()};
    assert(!control.compute(snapshot, {}).has_value()); // ARM前の目標は未取得
    assert(control.initialize(snapshot));

    BalanceReference reference{};
    reference.positionM = snapshot.chassis.positionM;
    reference.yawRateRadS = 1.0f;
    auto computed = control.compute(snapshot, reference);
    assert(computed);
    assert(computed->fixtureOnly);
    // pitch=0.1radなので、T_w=-2*0.1=-0.2Nm、T_p=-3*0.1=-0.3Nm。
    assert(std::fabs(computed->wheelBalanceTotalNm + 0.2f) < 1e-6f);
    assert(std::fabs(computed->jointTorqueNm[0]) > 0.01f);
    assert(std::fabs(computed->wheelTorqueNm[0] +
                     computed->wheelTorqueNm[1] + 0.2f) < 1e-5f);
    // Yawは倒立を確保した左右残余の中だけで割り当てる。
    assert(computed->wheelTorqueNm[0] >= -5.0f);
    assert(computed->wheelTorqueNm[1] <= 5.0f);
    assert(computed->yawAppliedNm > 0.0f);

    reference.yawRateRadS = 10.0f;
    computed = control.compute(snapshot, reference);
    assert(computed && computed->yawIntegralFrozen && computed->wheelSaturated);
    assert(computed->wheelTorqueNm[0] >= -5.0f);
    assert(computed->wheelTorqueNm[1] <= 5.0f);

    control.reset();
    assert(!control.compute(snapshot, reference));
    auto invalid = controlFixture();
    invalid.lqr.verified = false;
    BalanceControl unverified{invalid};
    assert(unverified.initialize(snapshot));
    assert(!unverified.compute(snapshot, reference)); // ゲイン未検証なら数学出力なし
}

void testHostOnly500HzPipeline()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    BalanceControl balance{controlFixture()};
    RobotSequence sequence{{50000, true}}; // 期限50msは人工SIL値。実機の期限ではない。

    // 初周期はセンサ・CANを「同じ時刻のコピー」として確定し、運転停止。
    auto source = samples.observation(2000);
    auto state = estimator.update(source, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(state));
    auto snapshot = std::get<ControlSnapshot>(state);
    auto input = simulatedInterlocks();
    assert(sequence.update(input, source.motors.sampledAtUs).state == RobotState::Disarmed);

    // ARMエッジより前に左右の実測脚長を固定目標として取得する。
    assert(balance.initialize(snapshot));
    input.operations.arm = true;
    auto result = sequence.update(input, 4000);
    assert(result.state == RobotState::Arming);
    assert(result.desiredMotorState == DesiredMotorState::EnableRs06);
    input.operations.arm = false;

    // 4軸のTx完了+新規応答を「模擬済み」の入力で渡す。
    input.rs06.session = result.armSession;
    input.rs06.axes.fill(Rs06ArmStatus::Confirmed);
    result = sequence.update(input, 6000);
    assert(result.state == RobotState::Balancing);
    assert(result.desiredMotorState == DesiredMotorState::DriveAllowed);

    // 制御計算は駆動命令ではない。SILではMotorCommand/CanPortを持たない。
    source = samples.observation(8000);
    state = estimator.update(source, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(state));
    snapshot = std::get<ControlSnapshot>(state);
    assert(sequence.update(input, 8000).state == RobotState::Balancing);
    const auto torque = balance.compute(snapshot, {0.0f, 0.0f, 0.0f});
    assert(torque && torque->fixtureOnly);
    assert(torque->wheelBalanceTotalNm < 0.0f);

    // RS06 1軸の鮮度が落ちた周期は推定結果を欠損扱いにし、
    // RobotSequenceが同周期にInterlockLostをラッチする。
    source = samples.observation(10000);
    source.motors.rs06[3].fresh = false;
    state = estimator.update(source, 2000us);
    assert(std::holds_alternative<StateEstimateError>(state));
    input.interlocks.motorsReady = false;
    result = sequence.update(input, 10000);
    assert(result.state == RobotState::Fault);
    assert(result.desiredMotorState == DesiredMotorState::StopAll);
    balance.reset();
    assert(!balance.compute(snapshot, {}));
    assert(sequence.currentFault()->primaryCause == FaultFlags::InterlockLost);

    // 受信が戻っても自動再ARMしない。CLEAR_FAULTには明示許可が必要。
    input.interlocks.motorsReady = true;
    source = samples.observation(12000);
    input.operations.clearFault = true;
    result = sequence.update(input, 12000);
    assert(result.state == RobotState::Fault);
    input.operations.clearFaultAuthorized = true;
    result = sequence.update(input, 14000);
    assert(result.state == RobotState::Disarmed);
    assert(result.desiredMotorState == DesiredMotorState::StopAll);
}

void testEstimatorFailurePreventsControl()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    BalanceControl balance{controlFixture()};
    auto source = samples.observation(2000);
    source.imu.fresh = false;
    auto result = estimator.update(source, 2000us);
    assert(std::holds_alternative<StateEstimateError>(result));
    assert(!std::get_if<ControlSnapshot>(&result)); // 古い制御値にfallbackしない
    // 運用コードはStateEstimateErrorだけを受け取り、物理トルクを生成しない。
    assert(!balance.compute(ControlSnapshot{}, {}));
}


void testEndToEnd500HzSilRunner()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}}; // 2sはSILだけの期限。実機値ではない。
    MotorCommand converter{motorFixture()};
    ControlCycle cycle{estimator, sequence, control, converter};
    ControlTickInput tick{};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    assert(!result.mathematicalTorques);
    tick.operations.arm = false;

    // CAN完了と応答の成立をSIL入力として別途模擬し、ARMセッションを保つ。
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = samples.observation(4000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Balancing);
    assert(!result.mathematicalTorques); // 初回の遷移周期に非ゼロ指令は作らない

    // 仮想時計の2ms周期で100回、同じ順番で推定→状態→数学トルクを実行。
    for (std::uint64_t nowUs = 6000; nowUs <= 204000; nowUs += 2000) {
        tick.observations = samples.observation(nowUs);
        result = cycle.step(tick);
        assert(result.sequence.state == RobotState::Balancing);
        assert(result.estimate && result.estimate->elapsed == 2000us);
        assert(result.mathematicalTorques && result.mathematicalTorques->fixtureOnly);
        // 実数学制御の値をモータ軸[Nm]・車輪電流[A]まで変換する。
        // どれもfixtureOnly=trueで、CAN送信可能なフレームにはならない。
        assert(result.motorCommandPreview && result.motorCommandPreview->fixtureOnly);
        const auto command = *result.motorCommandPreview;
        assert(command.status == CommandStatus::Ready ||
               command.status == CommandStatus::Saturated);
        assert(std::isfinite(command.rs06[0].limitedMotorTorqueNm));
        assert(std::isfinite(command.c620[0].limitedCurrentA));
    }

    // 3周期分（6ms）の欠落でControlTimingをラッチし、その周期の数値出力を捨てる。
    tick.observations = samples.observation(210000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(result.sequence.desiredMotorState == DesiredMotorState::StopAll);
    assert(!result.mathematicalTorques);
    assert(!result.motorCommandPreview);
    assert(sequence.currentFault()->primaryCause == FaultFlags::ControlTiming);

    // 故障原因が解消しても自動復帰しない。
    tick.observations = samples.observation(212000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(!result.mathematicalTorques);
}

void testSilSensorFaultAndClockRegression()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    MotorCommand converter{motorFixture()};
    ControlCycle cycle{estimator, sequence, control, converter};
    ControlTickInput tick{};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    tick.operations.arm = false;
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = samples.observation(4000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Balancing);

    // 現周期の推定に欠損があるなら、外部のready=trueを鵜呑みにしない。
    tick.observations = samples.observation(6000);
    tick.observations.motors.rs06[3].fresh = false;
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(!result.estimate && result.estimateError);
    assert(!result.mathematicalTorques);
    assert(!result.motorCommandPreview);
    assert(sequence.currentFault()->primaryCause == FaultFlags::InterlockLost);

    StateEstimator secondEstimator{estimatorFixture()};
    BalanceControl secondControl{controlFixture()};
    RobotSequence secondSequence{{2000000, true}};
    MotorCommand secondConverter{motorFixture()};
    ControlCycle second{secondEstimator, secondSequence, secondControl, secondConverter};
    ObservationFixture secondSamples;
    tick = {};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.observations = secondSamples.observation(4000);
    assert(second.step(tick).sequence.state == RobotState::Disarmed);
    tick.observations = secondSamples.observation(2000); // 時計逆行を試験
    result = second.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(sequence.currentFault()); // 別インスタンスのFAULTを消していない
    assert(secondSequence.currentFault()->primaryCause == FaultFlags::ClockRegression);
}

void testMotorConversionFailureTriggersSameCycleFault()
{
    ObservationFixture samples;
    // 係数自体は有限だが変換先floatの範囲を超える人工値を注入する。
    // 数学ControlOutputが成功しても、この周期の機構指令候補は捨てる。
    auto conversion = motorFixture();
    conversion.c620[0].wheelTorqueNmPerAmp =
        std::numeric_limits<float>::denorm_min();
    MotorCommand converter{conversion};
    assert(converter.hasVerifiedConfiguration());
    StateEstimator estimator{estimatorFixture()};
    BalanceControl balance{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    ControlCycle cycle{estimator, sequence, balance, converter};
    ControlTickInput tick{};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    tick.operations.arm = false;
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = samples.observation(4000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Balancing);
    tick.observations = samples.observation(6000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(result.sequence.desiredMotorState == DesiredMotorState::StopAll);
    assert(!result.mathematicalTorques && !result.motorCommandPreview);
    assert(sequence.currentFault()->primaryCause == FaultFlags::ExternalFault);
}

void testUnverifiedMotorConfigPreventsArm()
{
    ObservationFixture samples;
    auto conversion = motorFixture();
    conversion.c620[1].verified = false;
    MotorCommand converter{conversion};
    StateEstimator estimator{estimatorFixture()};
    BalanceControl balance{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    ControlCycle cycle{estimator, sequence, balance, converter};
    ControlTickInput tick{};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(2000);
    const auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Disarmed);
    assert(result.sequence.desiredMotorState == DesiredMotorState::StopAll);
    assert(!result.motorCommandPreview);
}


void testFirstTickNeedsMeasuredElapsedAndDisarmedPause()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    MotorCommand converter{motorFixture()};
    ControlCycle cycle{estimator, sequence, control, converter};
    ControlTickInput tick{};
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(1000);

    // 初回の時間差が未供給なら2msとみなさず、ARMも受理しない。
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Disarmed);
    assert(result.estimateError == StateEstimateError::InvalidPeriod);
    assert(!result.motorCommandPreview);

    // 次の実測周期で初めて推定が成立し、明示ARMを受け付ける。
    tick.observations = samples.observation(3000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    assert(result.estimate && result.estimate->elapsed == 2000us);

    // 別のセッションではDISARMED中の長い保守待機を
    // 「稼働中6ms停止」と混同してFAULTにしない。
    StateEstimator secondEstimator{estimatorFixture()};
    BalanceControl secondControl{controlFixture()};
    RobotSequence secondSequence{{2000000, true}};
    MotorCommand secondConverter{motorFixture()};
    ControlCycle second{secondEstimator, secondSequence, secondControl, secondConverter};
    ObservationFixture secondSamples;
    tick.operations.arm = false;
    tick.observations = secondSamples.observation(10000);
    tick.firstElapsed = 2000us;
    assert(second.step(tick).sequence.state == RobotState::Disarmed);
    tick.observations = secondSamples.observation(110000);
    const auto paused = second.step(tick);
    assert(paused.sequence.state == RobotState::Disarmed);
    assert(paused.observedFaults == FaultFlags::None);
    assert(paused.estimate && paused.estimate->elapsed == 100000us);
}

void testKnownEmergencyStopDoesNotAppendUnrelatedArithmeticFault()
{
    ObservationFixture samples;
    auto invalid = motorFixture();
    invalid.c620[0].wheelTorqueNmPerAmp =
        std::numeric_limits<float>::denorm_min();
    MotorCommand converter{invalid};
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    ControlCycle cycle{estimator, sequence, control, converter};
    ControlTickInput tick{};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    tick.operations.arm = false;
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = samples.observation(4000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Balancing);

    // 故障が先に判明しているなら、変換オーバーフロー試験用の
    // 不適切な係数で数学トルクを計算せず、主因E-STOPだけを保持。
    tick.observations = samples.observation(6000);
    tick.observedFaults = FaultFlags::EmergencyStop;
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(result.sequence.desiredMotorState == DesiredMotorState::StopAll);
    assert(!result.mathematicalTorques && !result.motorCommandPreview);
    assert(sequence.currentFault()->triggerFlags == FaultFlags::EmergencyStop);
}

void testWheelContinuityLossStopsSil()
{
    ObservationFixture samples;
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    MotorCommand converter{motorFixture()};
    ControlCycle cycle{estimator, sequence, control, converter};
    ControlTickInput tick{};
    tick.firstElapsed = 2000us;
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = samples.observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    tick.operations.arm = false;
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = samples.observation(4000);
    assert(cycle.step(tick).sequence.state == RobotState::Balancing);
    // 通信鮮度が有効でも巻数を喪失した位置は制御に使えない。
    samples.wheels[0].invalidateContinuity();
    tick.observations = samples.observation(6000);
    assert(tick.observations.motors.c620[0].fresh);
    result = cycle.step(tick);
    assert(result.estimateError == StateEstimateError::WheelFeedbackUnavailable);
    assert(result.sequence.state == RobotState::Fault);
    assert(result.sequence.desiredMotorState == DesiredMotorState::StopAll);
    assert(!result.mathematicalTorques && !result.motorCommandPreview);
    tick.observations = samples.observation(8000);
    assert(cycle.step(tick).sequence.state == RobotState::Fault);
    assert(!tick.observations.motors.c620[0].position);
}

} // namespace

int main()
{
    testControllerMathAndReset();
    testHostOnly500HzPipeline();
    testEstimatorFailurePreventsControl();
    testEndToEnd500HzSilRunner();
    testSilSensorFaultAndClockRegression();
    testMotorConversionFailureTriggersSameCycleFault();
    testUnverifiedMotorConfigPreventsArm();
    testFirstTickNeedsMeasuredElapsedAndDisarmedPause();
    testKnownEmergencyStopDoesNotAppendUnrelatedArithmeticFault();
    testWheelContinuityLossStopsSil();
    std::puts("PASS: SIL 500 Hz estimator→sequence→math-only control, failure/stop");
}
