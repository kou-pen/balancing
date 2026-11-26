#include "balancing/control/estimator/StateEstimator.hpp"
#include "balancing/control/filter/balancing_attitude_filter.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace balancing;
using namespace std::chrono_literals;

namespace {
constexpr float kPi = 3.14159265358979323846f;

StateEstimatorConfig fixtureConfig()
{
    StateEstimatorConfig config{};
    for (auto& leg : config.legs) {
        // 数値試験専用：実機CADの原点・組立分岐・軸向きを表していない。
        leg.motor.motorZeroRad = {0.0f, 0.0f};
        leg.motor.jointOffsetRad = {0.0f, 0.0f};
        leg.motor.direction = {1.0f, 1.0f};
        leg.motor.drivingTeeth = 1.0f;
        leg.motor.drivenTeeth = 1.0f;
        leg.motor.calibrated = true;
        leg.fixture = {0.2f, 0.15f, 0.15f, 0.25f, 0.25f, 0.02f, 0.01f, -1};
    }
    config.wheels = {{{0.1f, 10.0f, 1.0f, true}, {0.1f, 10.0f, -1.0f, true}}};
    return config;
}

void near(float actual, float expected, float tolerance = 1.0e-5f)
{
    assert(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance);
}

ImuAttitude filteredAttitude(BAttitudeFilter& filter, std::uint64_t atUs)
{
    // 旧C11の実際のバイアス学習と相補フィルタを通してからC++へ渡す。
    const BImuSample level{{0.0f, 0.0f, 9.81f}, {0.1f, -0.2f, 0.05f}};
    BAttitudeEstimate output{};
    assert(b_attitude_update(&filter, &level, true, atUs, atUs, &output));
    assert(output.valid);
    return {output.stamp_us, output.pitch_rad, output.roll_rad,
            output.pitch_rate_rad_s, output.roll_rate_rad_s, output.yaw_rate_rad_s,
            output.valid, true};
}

BAttitudeFilter calibratedFilter()
{
    BAttitudeFilter filter{};
    const BAttitudeFilterConfig config{
        {{0u, 1u, 2u}, {1, 1, 1}, true},
        2u, 8.0f, 11.0f, 0.9f, 0.01f, true
    };
    const BImuSample level{{0.0f, 0.0f, 9.81f}, {0.1f, -0.2f, 0.05f}};
    assert(b_attitude_init(&filter, &config));
    assert(b_attitude_add_static_sample(&filter, &level, true, true, 1000, 1000));
    assert(b_attitude_add_static_sample(&filter, &level, true, true, 2000, 2000));
    return filter;
}

void fillLegs(MotorFeedbackSnapshot& motors, std::uint64_t atUs)
{
    // 4軸の姿勢は外部からRS06のモータ軸角として注入。実機フレームは未接続。
    for (std::size_t axis = 0; axis < motors.rs06.size(); ++axis) {
        motors.rs06[axis].latest = Rs06FeedbackSample{
            Rs06StateReport{static_cast<std::uint8_t>(axis + 1), 0xa6,
                            Rs06Report::Status, Rs06Mode::Running, 0,
                            -kPi / 2.0f, 0.0f, 0.0f, 30.0f},
            atUs
        };
        motors.rs06[axis].fresh = true;
    }
}

const CanFrame kLeft{0x201u, {0x1fu, 0xfeu, 0x00u, 0x3cu,
                                    0x00u, 0x00u, 30u, 0u}, 8u, false};
const CanFrame kRight{0x202u, {0x00u, 0x02u, 0x00u, 0x3cu,
                                     0x00u, 0x00u, 30u, 0u}, 8u, false};
const CanFrame kLeft2{0x201u, {0x00u, 0x02u, 0x00u, 0x3cu,
                                     0x00u, 0x00u, 30u, 0u}, 8u, false};
const CanFrame kRight2{0x202u, {0x1fu, 0xfeu, 0x00u, 0x3cu,
                                      0x00u, 0x00u, 30u, 0u}, 8u, false};

template<class T>
bool isError(const std::variant<ControlSnapshot, StateEstimateError>& result, T expected)
{
    return std::holds_alternative<StateEstimateError>(result) &&
           std::get<StateEstimateError>(result) == expected;
}

void testSixAxisAndImuOneTick()
{
    MotorService motors{{{1, 2, 3, 4}, 0xa6}, {{{4000, true}, {4000, true}}}};
    motors.onC620Frame(kLeft, 7000);
    motors.onC620Frame(kRight, 7001);

    BAttitudeFilter filter = calibratedFilter();
    StateEstimator estimator{fixtureConfig()};
    StateEstimatorInput input{};
    input.imu = filteredAttitude(filter, 7000);
    input.motors = motors.feedback(8000);
    fillLegs(input.motors, 7000);

    const auto result = estimator.update(input, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(result));
    const auto& snapshot = std::get<ControlSnapshot>(result);
    assert(snapshot.sampledAtUs == 8000);
    assert(snapshot.elapsed == 2000us);
    assert(snapshot.imuReceivedAtUs == 7000);
    assert(snapshot.rs06ReceivedAtUs[0] == 7000);
    assert(snapshot.rs06ReceivedAtUs[3] == 7000);
    assert(snapshot.wheelReceivedAtUs[0] == 7000);
    assert(snapshot.wheelReceivedAtUs[1] == 7001);
    assert(snapshot.fixtureOnly); // 数学fixtureは実機モデルの検証にはならない。
    near(snapshot.body.pitchRad, 0.0f);
    near(snapshot.legs[0].mechanism.thetaBodyRad, 0.0f);
    near(snapshot.legs[1].mechanism.thetaBodyRad, 0.0f);
    assert(snapshot.legs[0].mechanism.lengthM > 0.3f);
    near(snapshot.chassis.positionM, 0.0f);
    near(snapshot.chassis.velocityMps, 0.0f);

    // 500Hzで同じCAN報告を再読しただけでは車輪位置を二重加算しない。
    input.motors = motors.feedback(9000);
    fillLegs(input.motors, 7000);
    const auto same = estimator.update(input, 1000us);
    assert(std::holds_alternative<ControlSnapshot>(same));
    near(std::get<ControlSnapshot>(same).chassis.positionM, 0.0f);

    motors.onC620Frame(kLeft2, 10000);
    motors.onC620Frame(kRight2, 10001);
    input.imu = filteredAttitude(filter, 10000);
    input.motors = motors.feedback(11000);
    fillLegs(input.motors, 10000);
    const auto moved = estimator.update(input, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(moved));
    const auto& updated = std::get<ControlSnapshot>(moved);
    const float deltaM = 6.2831853071795864769f * 0.1f * 4.0f / (10.0f * 8192.0f);
    near(updated.chassis.positionM, deltaM, 1e-7f);
    near(updated.wheels[0].motion.positionM, deltaM, 1e-7f);
    near(updated.wheels[1].motion.positionM, deltaM, 1e-7f);
    assert(updated.wheelReceivedAtUs[0] == 10000);
    assert(updated.wheelReceivedAtUs[1] == 10001);

    // resetは新原点を開始するための明示操作。通常ARM/DISARMでは行わない。
    assert(motors.resetWheelReference());
    estimator.reset();
    assert(isError(estimator.update(input, 2000us), StateEstimateError::WheelFeedbackUnavailable));
    motors.onC620Frame(kLeft2, 12000);
    motors.onC620Frame(kRight2, 12001);
    input.motors = motors.feedback(13000);
    fillLegs(input.motors, 12000);
    const auto restarted = estimator.update(input, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(restarted));
    near(std::get<ControlSnapshot>(restarted).chassis.positionM, 0.0f);
}

void testInvalidInputDoesNotProducePartialSnapshot()
{
    StateEstimator estimator{fixtureConfig()};
    StateEstimatorInput input{};
    input.imu = {7000, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, true, true};
    input.motors.sampledAtUs = 8000;
    fillLegs(input.motors, 7000);
    assert(isError(estimator.update(input, 2000us),
                   StateEstimateError::WheelFeedbackUnavailable));
    assert(isError(estimator.update(input, 0us),
                   StateEstimateError::InvalidPeriod));
    input.imu.fresh = false;
    assert(isError(estimator.update(input, 2000us),
                   StateEstimateError::ImuUnavailable));
    input.imu.fresh = true;
    input.imu.pitchRad = std::numeric_limits<float>::quiet_NaN();
    assert(isError(estimator.update(input, 2000us),
                   StateEstimateError::InvalidImu));
    input.imu.pitchRad = 0;
    input.motors.rs06[3].fresh = false;
    assert(isError(estimator.update(input, 2000us),
                   StateEstimateError::LegFeedbackUnavailable));

    // 左右脚のいずれが解けないかを区別し、片側だけの出力は公開しない。
    auto config = fixtureConfig();
    config.legs[1].motor.calibrated = false;
    StateEstimator invalidRight{config};
    input.motors.rs06[3].fresh = true;
    assert(isError(invalidRight.update(input, 2000us),
                   StateEstimateError::RightLegUnsolved));
    config = fixtureConfig();
    config.legs[0].fixture.branch = 0;
    StateEstimator invalidLeft{config};
    assert(isError(invalidLeft.update(input, 2000us),
                   StateEstimateError::LeftLegUnsolved));
}

void testStaleWheelBoundaryAndImuRecovery()
{
    MotorService motors{{{1, 2, 3, 4}, 0xa6}, {{{4000, true}, {4000, true}}}};
    motors.onC620Frame(kLeft, 7000);
    motors.onC620Frame(kRight, 7001);
    StateEstimator estimator{fixtureConfig()};
    StateEstimatorInput input{};
    input.imu = {27000, 0, 0, 0, 0, 0, true, true};
    // 新C++17 MotorServiceの20msちょうどでは、左車輪だけfresh=false。
    input.motors = motors.feedback(27000);
    fillLegs(input.motors, 7000);
    assert(isError(estimator.update(input, 2000us),
                   StateEstimateError::WheelFeedbackUnavailable));

    // 21msの受信空白では新しい報告だけで位置を復旧させない。
    motors.onC620Frame(kLeft2, 28000);
    motors.onC620Frame(kRight2, 28001);
    input.motors = motors.feedback(29000);
    fillLegs(input.motors, 28000);
    assert(isError(estimator.update(input, 2000us), StateEstimateError::WheelFeedbackUnavailable));
    assert(motors.resetWheelReference());
    estimator.reset();
    input.imu.fresh = false;
    motors.onC620Frame(kLeft2, 30000);
    motors.onC620Frame(kRight2, 30001);
    input.motors = motors.feedback(31000);
    fillLegs(input.motors, 30000);
    assert(isError(estimator.update(input, 2000us),
                   StateEstimateError::ImuUnavailable));
    input.imu.fresh = true;
    input.imu.receivedAtUs = 31000;
    // IMU回復周期には、欠損中の新規CAN報告を二重加算せずに使える。
    const auto recovered = estimator.update(input, 2000us);
    assert(std::holds_alternative<ControlSnapshot>(recovered));
    assert(std::get<ControlSnapshot>(recovered).wheelReceivedAtUs[0] == 30000);
}
} // namespace

int main()
{
    testSixAxisAndImuOneTick();
    testInvalidInputDoesNotProducePartialSnapshot();
    testStaleWheelBoundaryAndImuRecovery();
    std::puts("PASS: StateEstimator IMU + 6 motors, C11 math, time stamps and failure isolation");
}
