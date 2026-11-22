#include "balancing/control/estimator/WheelOdometry.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace balancing;

namespace {
constexpr float kTolerance = 1.0e-6f;
constexpr float kTau = 6.2831853071795864769f;

std::array<WheelOdometryConfig, 2> testConfig()
{
    // 数学fixture：半径0.1m、10:1、左右で逆符号。実機で検証された値ではない。
    return {{{0.1f, 10.0f, 1.0f, true}, {0.1f, 10.0f, -1.0f, true}}};
}

MotorFeedbackSnapshot sample(std::uint64_t nowUs)
{
    MotorFeedbackSnapshot motors{};
    motors.sampledAtUs = nowUs;
    return motors;
}

void setWheel(MotorFeedbackSnapshot& motors, std::size_t axis,
              std::uint16_t encoder, std::int16_t rpm, std::uint64_t atUs,
              bool fresh = true)
{
    motors.c620[axis].latest = C620FeedbackSample{
        BC620Feedback{static_cast<std::uint8_t>(axis + 1), encoder, rpm, 0, 30},
        atUs};
    motors.c620[axis].fresh = fresh;
}

void near(float actual, float expected)
{
    assert(std::isfinite(actual));
    assert(std::fabs(actual - expected) < kTolerance);
}

void testWrapAndIndependentWheels()
{
    WheelOdometry odom{testConfig()};
    auto first = sample(1000);
    setWheel(first, 0, 8190, 60, 1000);
    setWheel(first, 1, 2, 60, 1000);
    assert((odom.update(first) ==
            std::array<WheelUpdateResult, 2>{WheelUpdateResult::Updated, WheelUpdateResult::Updated}));
    auto state = odom.snapshot();
    assert(state.sampledAtUs == 1000);
    near(state.wheel[0].latest->positionM, 0.0f);
    near(state.wheel[1].latest->positionM, 0.0f);

    // 8190→2は+4、右の2→8190は-4。方向符号の逆転で両方+の移動になる。
    auto next = sample(2000);
    setWheel(next, 0, 2, 60, 2000);
    setWheel(next, 1, 8190, 60, 2000);
    const auto results = odom.update(next);
    assert(results[0] == WheelUpdateResult::Updated);
    assert(results[1] == WheelUpdateResult::Updated);
    state = odom.snapshot();
    near(state.wheel[0].latest->positionM, kTau * 0.1f * 4.0f / (10.0f * 8192.0f));
    near(state.wheel[1].latest->positionM, kTau * 0.1f * 4.0f / (10.0f * 8192.0f));
    near(state.wheel[0].latest->velocityMps, kTau * 0.1f / 10.0f);
    near(state.wheel[1].latest->velocityMps, -kTau * 0.1f / 10.0f);
    assert(state.wheel[0].fresh && state.wheel[1].fresh);
    assert(state.wheel[0].latest->receivedAtUs == 2000);

    // 500Hzで同じ受信値を再読しても積算位置・受信時刻は進めない。
    next.sampledAtUs = 4000;
    assert(odom.update(next)[0] == WheelUpdateResult::NoNewSample);
    near(odom.snapshot().wheel[0].latest->positionM, state.wheel[0].latest->positionM);
    assert(odom.snapshot().wheel[0].latest->receivedAtUs == 2000);

    // 左車輪だけ新しいフレームを受けても右はそのまま。鮮度は入力元で確定する。
    auto leftOnly = sample(5000);
    setWheel(leftOnly, 0, 3, 30, 5000);
    setWheel(leftOnly, 1, 8190, 60, 2000, false);
    assert(odom.update(leftOnly)[0] == WheelUpdateResult::Updated);
    state = odom.snapshot();
    assert(state.wheel[0].fresh && !state.wheel[1].fresh);
    assert(state.wheel[1].latest->receivedAtUs == 2000);
    assert(state.wheel[0].latest->receivedAtUs == 5000);
}

void testInvalidSampleLeavesPreviousValue()
{
    WheelOdometry odom{testConfig()};
    auto initial = sample(1000);
    setWheel(initial, 0, 0, 0, 1000);
    assert(odom.update(initial)[0] == WheelUpdateResult::Updated);

    // 4096カウントは正逆の符号を決められない。新規のCAN時刻に飛び付かない。
    auto invalid = sample(2000);
    setWheel(invalid, 0, 4096, 10, 2000);
    assert(odom.update(invalid)[0] == WheelUpdateResult::AmbiguousDelta);
    assert(!odom.snapshot().wheel[0].fresh);
    assert(odom.snapshot().wheel[0].latest->receivedAtUs == 1000);
    near(odom.snapshot().wheel[0].latest->positionM, 0.0f);

    // CAN受信時刻の未来・逆行や範囲外のロータ角も位置・基準時刻を変更しない。
    invalid.sampledAtUs = 2500;
    setWheel(invalid, 0, 10, 0, 3000);
    assert(odom.update(invalid)[0] == WheelUpdateResult::InvalidTimestamp);
    setWheel(invalid, 0, 10, 0, 999);
    assert(odom.update(invalid)[0] == WheelUpdateResult::InvalidTimestamp);
    setWheel(invalid, 0, 8192, 0, 2000);
    assert(odom.update(invalid)[0] == WheelUpdateResult::InvalidValue);
    assert(odom.snapshot().wheel[0].latest->receivedAtUs == 1000);

    // 不正サンプル後に有効な新規受信が来れば、最後に受理した角度を起点にする。
    setWheel(invalid, 0, 1, 0, 2001);
    assert(odom.update(invalid)[0] == WheelUpdateResult::Updated);
    near(odom.snapshot().wheel[0].latest->positionM, kTau * 0.1f / (10.0f * 8192.0f));
}

void testUnverifiedAndReset()
{
    auto config = testConfig();
    config[1].verified = false;
    WheelOdometry odom{config};
    auto first = sample(1000);
    setWheel(first, 0, 100, 100, 1000);
    setWheel(first, 1, 100, 100, 1000);
    const auto outcome = odom.update(first);
    assert(outcome[0] == WheelUpdateResult::Updated);
    assert(outcome[1] == WheelUpdateResult::InvalidConfiguration);
    assert(!odom.snapshot().wheel[1].latest && !odom.snapshot().wheel[1].fresh);

    odom.reset();
    assert(!odom.snapshot().wheel[0].latest);
    first.sampledAtUs = 2000;
    assert(odom.update(first)[0] == WheelUpdateResult::Updated);
    near(odom.snapshot().wheel[0].latest->positionM, 0.0f);

    // 無効なギヤ比もコンストラクタ時だけ棄却し、周期毎の設定変更手段は持たない。
    config = testConfig();
    config[0].motorRevsPerWheelRev = std::numeric_limits<float>::quiet_NaN();
    WheelOdometry invalidConfig{config};
    assert(invalidConfig.update(first)[0] == WheelUpdateResult::InvalidConfiguration);
}

void testRealCanDecoderToMotorServiceAndOdometry()
{
    // 実C11 C620デコーダ → MotorService → C++車輪変換のホスト結合。
    MotorService motors{{{11, 12, 13, 14}, 0xa6}};
    WheelOdometry odom{testConfig()};
    const BMotorCanFrame left{0x201u, {0x1fu, 0xfeu, 0x00u, 0x3cu,
                                       0x00u, 0x00u, 30u, 0u}, 8u, false};
    const BMotorCanFrame right{0x202u, {0x00u, 0x02u, 0x00u, 0x3cu,
                                        0x00u, 0x00u, 30u, 0u}, 8u, false};
    motors.onC620Frame(left, 1000);
    motors.onC620Frame(right, 1001);
    assert(odom.update(motors.feedback(2000))[0] == WheelUpdateResult::Updated);
    auto observed = odom.snapshot();
    assert(observed.wheel[0].latest->encoderRaw == 8190);
    assert(observed.wheel[0].latest->receivedAtUs == 1000);
    assert(observed.wheel[1].latest->receivedAtUs == 1001);
    // 期限の意味は#44に従う：20msちょうどでfresh=false。
    odom.update(motors.feedback(21000));
    observed = odom.snapshot();
    assert(!observed.wheel[0].fresh && observed.wheel[1].fresh);
    assert(observed.wheel[0].latest->receivedAtUs == 1000);
    // 旧C11 b_check_freshnessは境界ちょうど有効なので、差分は変更せず記録する。
}
} // namespace

int main()
{
    testWrapAndIndependentWheels();
    testInvalidSampleLeavesPreviousValue();
    testUnverifiedAndReset();
    testRealCanDecoderToMotorServiceAndOdometry();
    std::puts("PASS: C620 wheel odometry C++17, wrap, freshness, decode, invalid timestamps");
}
