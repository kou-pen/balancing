#include "balancing/app/RobotSequence.hpp"
#include "balancing/control/controller/BalanceControl.hpp"
#include "balancing/control/estimator/StateEstimator.hpp"
#include "support/SilControlCycle.hpp"

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
    c.lqr.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_PITCH] = 2.0f;
    c.lqr.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_POSITION] = 1.0f;
    c.lqr.gain[B_OUTPUT_HIP_TOTAL][B_STATE_PITCH] = 3.0f;
    c.legPd = {100.0f, 10.0f, 2.0f, 0.5f, 4.0f, true};
    c.yawPi = {3.0f, 4.0f, 2.0f, true};
    c.wheelTorqueLimitNm = {5.0f, 5.0f};
    return c;
}

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
    for (std::size_t i = 0; i < 2; ++i) {
        input.motors.c620[i].latest = C620FeedbackSample{
            BC620Feedback{static_cast<std::uint8_t>(i + 1), 1000, 0, 0, 30},
            atUs};
        input.motors.c620[i].fresh = true;
    }
    return input;
}

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
    StateEstimator estimator{estimatorFixture()};
    auto output = estimator.update(observation(2000), 2000us);
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
    StateEstimator estimator{estimatorFixture()};
    BalanceControl balance{controlFixture()};
    RobotSequence sequence{{50000, true}}; // 期限50msは人工SIL値。実機の期限ではない。

    // 初周期はセンサ・CANを「同じ時刻のコピー」として確定し、運転停止。
    auto source = observation(2000);
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
    source = observation(8000);
    state = estimator.update(source, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(state));
    snapshot = std::get<ControlSnapshot>(state);
    assert(sequence.update(input, 8000).state == RobotState::Balancing);
    const auto torque = balance.compute(snapshot, {0.0f, 0.0f, 0.0f});
    assert(torque && torque->fixtureOnly);
    assert(torque->wheelBalanceTotalNm < 0.0f);

    // RS06 1軸の鮮度が落ちた周期は推定結果を欠損扱いにし、
    // RobotSequenceが同周期にInterlockLostをラッチする。
    source = observation(10000);
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
    source = observation(12000);
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
    StateEstimator estimator{estimatorFixture()};
    BalanceControl balance{controlFixture()};
    auto source = observation(2000);
    source.imu.fresh = false;
    auto result = estimator.update(source, 2000us);
    assert(std::holds_alternative<StateEstimateError>(result));
    assert(!std::get_if<ControlSnapshot>(&result)); // 古い制御値にfallbackしない
    // 運用コードはStateEstimateErrorだけを受け取り、物理トルクを生成しない。
    assert(!balance.compute(ControlSnapshot{}, {}));
}


void testEndToEnd500HzSilRunner()
{
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}}; // 2sはSILだけの期限。実機値ではない。
    sil::SilControlCycle cycle{estimator, sequence, control};
    sil::TickInput tick{};
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    assert(!result.mathematicalTorques);
    tick.operations.arm = false;

    // CAN完了と応答の成立をSIL入力として別途模擬し、ARMセッションを保つ。
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = observation(4000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Balancing);
    assert(!result.mathematicalTorques); // 初回の遷移周期に非ゼロ指令は作らない

    // 仮想時計の2ms周期で100回、同じ順番で推定→状態→数学トルクを実行。
    for (std::uint64_t nowUs = 6000; nowUs <= 204000; nowUs += 2000) {
        tick.observations = observation(nowUs);
        result = cycle.step(tick);
        assert(result.sequence.state == RobotState::Balancing);
        assert(result.estimate && result.estimate->elapsed == 2000us);
        assert(result.mathematicalTorques && result.mathematicalTorques->fixtureOnly);
    }

    // 3周期分（6ms）の欠落でControlTimingをラッチし、その周期の数値出力を捨てる。
    tick.observations = observation(210000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(result.sequence.desiredMotorState == DesiredMotorState::StopAll);
    assert(!result.mathematicalTorques);
    assert(sequence.currentFault()->primaryCause == FaultFlags::ControlTiming);

    // 故障原因が解消しても自動復帰しない。
    tick.observations = observation(212000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(!result.mathematicalTorques);
}

void testSilSensorFaultAndClockRegression()
{
    StateEstimator estimator{estimatorFixture()};
    BalanceControl control{controlFixture()};
    RobotSequence sequence{{2000000, true}};
    sil::SilControlCycle cycle{estimator, sequence, control};
    sil::TickInput tick{};
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.operations.arm = true;
    tick.observations = observation(2000);
    auto result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Arming);
    tick.operations.arm = false;
    tick.armFeedback.session = result.sequence.armSession;
    tick.armFeedback.axes.fill(Rs06ArmStatus::Confirmed);
    tick.observations = observation(4000);
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Balancing);

    // 現周期の推定に欠損があるなら、外部のready=trueを鵜呑みにしない。
    tick.observations = observation(6000);
    tick.observations.motors.rs06[3].fresh = false;
    result = cycle.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(!result.estimate && result.estimateError);
    assert(!result.mathematicalTorques);
    assert(sequence.currentFault()->primaryCause == FaultFlags::InterlockLost);

    StateEstimator secondEstimator{estimatorFixture()};
    BalanceControl secondControl{controlFixture()};
    RobotSequence secondSequence{{2000000, true}};
    sil::SilControlCycle second{secondEstimator, secondSequence, secondControl};
    tick = {};
    tick.interlocks = simulatedInterlocks().interlocks;
    tick.observations = observation(4000);
    assert(second.step(tick).sequence.state == RobotState::Disarmed);
    tick.observations = observation(2000); // 時計逆行を試験
    result = second.step(tick);
    assert(result.sequence.state == RobotState::Fault);
    assert(sequence.currentFault()); // 別インスタンスのFAULTを消していない
    assert(secondSequence.currentFault()->primaryCause == FaultFlags::ClockRegression);
}

} // namespace

int main()
{
    testControllerMathAndReset();
    testHostOnly500HzPipeline();
    testEstimatorFailurePreventsControl();
    testEndToEnd500HzSilRunner();
    testSilSensorFaultAndClockRegression();
    std::puts("PASS: SIL 500 Hz estimator→sequence→math-only control, failure/stop");
}
