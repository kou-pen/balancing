#include "balancing/control/estimator/WheelOdometry.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace balancing;

namespace {
constexpr float kTau = 6.2831853071795864769f;
// 人工の取得間隔上限。実機速度・CAN遅延を検証した値ではない。
constexpr std::array<C620ContinuityConfig, 2> continuity{{{4000, true}, {4000, true}}};
std::array<WheelOdometryConfig, 2> config()
{
    return {{{0.1f, 10.0f, 1.0f, true}, {0.1f, 10.0f, -1.0f, true}}};
}
void near(float actual, float expected)
{
    assert(std::isfinite(actual) && std::fabs(actual - expected) < 1e-6f);
}
float distance(int counts)
{
    return kTau * 0.1f * static_cast<float>(counts) / (10.0f * 8192.0f);
}
BMotorCanFrame frame(unsigned axis, unsigned raw, std::int16_t rpm = 60)
{
    const auto speed = static_cast<std::uint16_t>(rpm);
    return {0x201u + axis, {static_cast<std::uint8_t>(raw >> 8),
        static_cast<std::uint8_t>(raw), static_cast<std::uint8_t>(speed >> 8),
        static_cast<std::uint8_t>(speed), 0, 0, 30, 0}, 8, false};
}

void testMultipleReportsAndSameTimestamp()
{
    MotorService motors{{{11, 12, 13, 14}, 0xa6}, continuity};
    WheelOdometry odom{config()};
    motors.onC620Frame(frame(0, 0), 1000);
    motors.onC620Frame(frame(1, 2), 1000);
    auto initial = motors.feedback(1000);
    assert(odom.update(initial)[0] == WheelUpdateResult::Updated);
    near(odom.snapshot().wheel[0].latest->positionM, 0);

    // +3000,+3000の2報告を1周期に取得。最新値だけの比較では-2192になる。
    motors.onC620Frame(frame(0, 3000), 2000);
    motors.onC620Frame(frame(0, 6000), 2000);
    motors.onC620Frame(frame(1, 8190), 2000);
    auto next = motors.feedback(2000);
    assert(next.c620[0].position->counts == 6000);
    assert(next.c620[0].latest->sequence == 3);
    assert(next.c620[1].position->counts == -4);
    assert(odom.update(next)[0] == WheelUpdateResult::Updated);
    near(odom.snapshot().wheel[0].latest->positionM, distance(6000));
    near(odom.snapshot().wheel[1].latest->positionM, distance(4));
    near(odom.snapshot().wheel[1].latest->velocityMps, -kTau * 0.1f / 10);
    assert(odom.update(next)[0] == WheelUpdateResult::NoNewSample);
    // 同時刻の別報告も受理番号で識別し、正転周回も失わない。
    motors.onC620Frame(frame(0, 1), 2000);
    next = motors.feedback(2000);
    assert(next.c620[0].position->counts == 8193);
    assert(odom.update(next)[0] == WheelUpdateResult::Updated);
    near(odom.snapshot().wheel[0].latest->positionM, distance(8193));
    assert(odom.update(initial)[0] == WheelUpdateResult::InvalidSequence);
    assert(!odom.snapshot().wheel[0].fresh);
}

void testLossAndExplicitReference()
{
    MotorService motors{{{11, 12, 13, 14}, 0xa6}, continuity};
    WheelOdometry odom{config()};
    motors.onC620Frame(frame(0, 100), 1000);
    motors.onC620Frame(frame(1, 100), 1000);
    const auto old = motors.feedback(1000);
    odom.update(old);
    motors.onC620Frame(frame(0, 4196), 2000);
    motors.onC620Frame(frame(1, 99), 2000);
    auto broken = motors.feedback(2000);
    assert(broken.c620[0].continuity == C620ContinuityStatus::AmbiguousDelta);
    assert(broken.c620[0].fresh && !broken.c620[0].position);
    assert(odom.update(broken)[0] == WheelUpdateResult::ContinuityUnavailable);
    assert(!odom.snapshot().wheel[0].fresh && odom.snapshot().wheel[1].fresh);
    motors.onC620Frame(frame(0, 4200), 3000);
    assert(!motors.feedback(3000).c620[0].position);
    assert(odom.update(old)[0] == WheelUpdateResult::ContinuityUnavailable);
    motors.setCan2BusOff(true);
    assert(!motors.resetWheelReference());
    motors.setCan2BusOff(false);
    assert(!motors.feedback(3000).c620[1].position);
    // 旧キュー排除済みとするホストfixture。片側だけresetしても位置は成立しない。
    assert(motors.resetWheelReference());
    motors.onC620Frame(frame(0, 4200), 3000); // 旧時刻の報告では原点を作らない
    assert(!motors.feedback(3000).c620[0].latest);
    motors.onC620Frame(frame(0, 4210), 4000);
    motors.onC620Frame(frame(1, 105), 4000);
    auto rebased = motors.feedback(4000);
    assert(odom.update(rebased)[0] == WheelUpdateResult::ReferenceMismatch);
    odom.reset();
    assert(odom.update(old)[0] == WheelUpdateResult::ReferenceMismatch);
    assert(odom.update(rebased)[0] == WheelUpdateResult::Updated);
    near(odom.snapshot().wheel[0].latest->positionM, 0);
    assert(rebased.c620[0].generation == 2 && rebased.c620[0].latest->sequence > old.c620[0].latest->sequence);
    motors.reportCan2RxLost();
    motors.onC620Frame(frame(0, 4211), 5000);
    assert(!motors.feedback(5000).c620[0].position);
}

void testConfigurationGapAndInvalidInput()
{
    C620Motor absent{};
    BC620Feedback raw{1, 100, 0, 0, 30};
    absent.onFeedback(raw, 1000);
    assert(absent.feedback(1000).fresh && !absent.feedback(1000).position);
    for (const auto bad : {C620ContinuityConfig{0,true}, C620ContinuityConfig{20001,true}}) {
        C620Motor motor{bad};
        motor.onFeedback(raw, 1000);
        assert(motor.feedback(1000).continuity == C620ContinuityStatus::Unverified);
    }
    C620Motor gap{{4000,true}};
    gap.onFeedback(raw, 1000);
    gap.onFeedback(raw, 5000);
    assert(gap.feedback(5000).continuity == C620ContinuityStatus::GapExceeded);
    gap.onFeedback(raw, 5001);
    assert(!gap.feedback(5001).position);
    C620Motor regression{{4000,true}};
    regression.onFeedback(raw, 1000);
    regression.onFeedback(raw, 999);
    assert(regression.feedback(1000).continuity == C620ContinuityStatus::ClockRegression);
    assert(regression.feedback(1000).latest->receivedAtUs == 1000);
    C620Motor badRaw{{4000,true}};
    badRaw.onFeedback(raw, 1000);
    raw.encoder_raw = 8192;
    badRaw.onFeedback(raw, 2000);
    assert(badRaw.feedback(2000).continuity == C620ContinuityStatus::InvalidValue);

    MotorService motors{{{11,12,13,14},0xa6},continuity};
    motors.onC620Frame(frame(0, 1), 1000);
    motors.onC620Frame(frame(1, 1), 1000);
    auto invalidConfig = config();
    invalidConfig[1].verified = false;
    WheelOdometry odom{invalidConfig};
    auto valid = motors.feedback(1000);
    assert(odom.update(valid)[1] == WheelUpdateResult::InvalidConfiguration);
    auto future = valid;
    future.sampledAtUs = 999;
    assert(odom.update(future)[0] == WheelUpdateResult::InvalidTimestamp);
    auto mismatch = valid;
    ++mismatch.c620[0].position->sequence;
    assert(odom.update(mismatch)[0] == WheelUpdateResult::InvalidSequence);
    assert(!odom.snapshot().wheel[0].fresh);
    // 既存20ms鮮度期限は変更しない。
    assert(motors.feedback(20999).c620[0].fresh);
    assert(!motors.feedback(21000).c620[0].fresh);
    odom.update(motors.feedback(21000));
    assert(!odom.snapshot().wheel[0].fresh);
    motors.onC620Frame(frame(0, 8192), 22000);
    assert(motors.feedback(22000).c620[0].continuity == C620ContinuityStatus::TransportLost);
    assert(motors.feedback(22000).c620[0].latest->receivedAtUs == 1000);
    assert(motors.feedback(22000).c620[1].position);
}
} // namespace

int main()
{
    testMultipleReportsAndSameTimestamp();
    testLossAndExplicitReference();
    testConfigurationGapAndInvalidInput();
    std::puts("PASS: C620 receive-order accumulation, continuity, generations and wheel conversion");
}
