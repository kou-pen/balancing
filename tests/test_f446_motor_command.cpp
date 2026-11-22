#include "balancing/control/controller/MotorCommand.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace balancing;

namespace {
#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::fprintf(stderr, "line %d: %s\n", __LINE__, #condition);    \
            std::exit(EXIT_FAILURE);                                        \
        }                                                                   \
    } while (false)

void near(float actual, float expected)
{
    CHECK(std::isfinite(actual));
    CHECK(std::fabs(actual - expected) < 2.0e-6f);
}

MotorCommandConfig fixture()
{
    // 数学テスト専用、実機のトルク限界・車輪効率・ギヤ比ではない。
    // すべての軸で独立した設定を持ち、符号と飽和の取り違えを検出する。
    MotorCommandConfig cfg{};
    cfg.rs06 = {{
        {1.5f, 1.0f, 3.0f, true},
        {1.5f, -1.0f, 1.0f, true},
        {1.5f, 1.0f, 2.5f, true},
        {1.5f, -1.0f, 5.0f, true}
    }};
    cfg.c620 = {{
        {0.4f, 1.0f, 2.0f, true},
        {0.6f, -1.0f, 4.0f, true}
    }};
    return cfg;
}

BalanceControlOutput request()
{
    BalanceControlOutput input{};
    input.jointTorqueNm = {3.0f, -3.0f, 9.0f, -9.0f};
    input.wheelTorqueNm = {1.2f, -1.2f};
    return input;
}

void testSixAxesAndIndependentLimits()
{
    MotorCommand convert{fixture()};
    const auto result = convert.evaluate(request());
    CHECK(result.status == CommandStatus::Saturated);
    CHECK(result.fixtureOnly); // 数学結果は実機への駆動許可ではない
    // 仮想仕事: τ_motor = τ_joint * sign / (motorRevsPerJointRev)。
    near(result.rs06[0].requestedMotorTorqueNm, 2.0f);
    near(result.rs06[1].requestedMotorTorqueNm, 2.0f);
    near(result.rs06[2].requestedMotorTorqueNm, 6.0f);
    near(result.rs06[3].requestedMotorTorqueNm, 6.0f);
    near(result.rs06[0].limitedMotorTorqueNm, 2.0f);
    near(result.rs06[1].limitedMotorTorqueNm, 1.0f);
    near(result.rs06[2].limitedMotorTorqueNm, 2.5f);
    near(result.rs06[3].limitedMotorTorqueNm, 5.0f);
    CHECK(!result.rs06[0].saturated && result.rs06[1].saturated &&
          result.rs06[2].saturated && result.rs06[3].saturated);

    // 車輪ごとのNm/Aには減速機+モータ定数+効率が既に含まれている。
    // 左正電流、右は反転方向の負トルクを正電流で実現するfixture。
    near(result.c620[0].requestedCurrentA, 3.0f);
    near(result.c620[1].requestedCurrentA, 2.0f);
    near(result.c620[0].limitedCurrentA, 2.0f);
    near(result.c620[1].limitedCurrentA, 2.0f);
    CHECK(result.c620[0].saturated && !result.c620[1].saturated);
    CHECK(!result.upstreamWheelSaturated);
}

void testNoSaturationAndPreserveUpstreamDiagnostic()
{
    MotorCommand convert{fixture()};
    auto input = request();
    input.jointTorqueNm = {0.3f, -0.3f, 0.3f, -0.3f};
    input.wheelTorqueNm = {0.2f, -0.2f};
    input.wheelSaturated = true; // 上流のYaw飽和と個別軸の最終飽和は異なる
    const auto result = convert.evaluate(input);
    CHECK(result.status == CommandStatus::Ready);
    CHECK(result.upstreamWheelSaturated);
    for (const auto& axis : result.rs06) {
        CHECK(!axis.saturated);
    }
    for (const auto& axis : result.c620) {
        CHECK(!axis.saturated);
    }

    // 0 Nmであっても未検証の物理係数を勝手に許可してよいわけではない。
    input.jointTorqueNm.fill(0.0f);
    input.wheelTorqueNm.fill(0.0f);
    CHECK(convert.evaluate(input).status == CommandStatus::Ready);
}

void checkAllZero(const MotorCommandPreview& result)
{
    for (const auto& axis : result.rs06) {
        near(axis.limitedMotorTorqueNm, 0.0f);
        near(axis.requestedMotorTorqueNm, 0.0f);
        CHECK(!axis.saturated);
    }
    for (const auto& axis : result.c620) {
        near(axis.limitedCurrentA, 0.0f);
        near(axis.requestedCurrentA, 0.0f);
        CHECK(!axis.saturated);
    }
}

void testUnverifiedConfigurationsAndInput()
{
    auto cfg = fixture();
    cfg.rs06[3].verified = false;
    MotorCommand unverified{cfg};
    auto result = unverified.evaluate(request());
    CHECK(result.status == CommandStatus::UnverifiedConfiguration);
    checkAllZero(result);

    cfg = fixture();
    cfg.rs06[0].motorRevsPerJointRev = 0.0f;
    MotorCommand badRatio{cfg};
    CHECK(badRatio.evaluate(request()).status == CommandStatus::UnverifiedConfiguration);

    cfg = fixture();
    cfg.c620[0].currentDirection = 0.0f;
    MotorCommand noDirection{cfg};
    CHECK(noDirection.evaluate(request()).status == CommandStatus::UnverifiedConfiguration);

    cfg = fixture();
    cfg.c620[0].maxAbsCurrentA = 20.1f; // CAN表現幅≠物理安全上限
    MotorCommand beyondCodec{cfg};
    CHECK(beyondCodec.evaluate(request()).status == CommandStatus::UnverifiedConfiguration);

    MotorCommand convert{fixture()};
    auto input = request();
    input.jointTorqueNm[3] = std::numeric_limits<float>::quiet_NaN();
    result = convert.evaluate(input);
    CHECK(result.status == CommandStatus::InvalidInput);
    checkAllZero(result);

    input = request();
    input.wheelTorqueNm[1] = std::numeric_limits<float>::infinity();
    result = convert.evaluate(input);
    CHECK(result.status == CommandStatus::InvalidInput);
    checkAllZero(result);

    input = request();
    input.fixtureOnly = false; // 実機向け指令への偽装を許可しない数学段階
    result = convert.evaluate(input);
    CHECK(result.status == CommandStatus::InvalidInput);
    checkAllZero(result);
}

void testConversionFailureIsAtomic()
{
    // ratioは有限でも小さすぎてfloatの表現可能範囲を超える換算があり得る。
    // 先行する軸が成功しても部分的な指令を出力してはいけない。
    auto cfg = fixture();
    cfg.rs06[3].motorRevsPerJointRev = std::numeric_limits<float>::min();
    MotorCommand convert{cfg};
    auto input = request();
    input.jointTorqueNm[3] = std::numeric_limits<float>::max();
    const auto result = convert.evaluate(input);
    CHECK(result.status == CommandStatus::ConversionFailed);
    checkAllZero(result);

    cfg = fixture();
    cfg.c620[1].wheelTorqueNmPerAmp = std::numeric_limits<float>::min();
    MotorCommand wheelOverflow{cfg};
    input = request();
    input.wheelTorqueNm[1] = std::numeric_limits<float>::max();
    const auto wheelResult = wheelOverflow.evaluate(input);
    CHECK(wheelResult.status == CommandStatus::ConversionFailed);
    checkAllZero(wheelResult);
}
} // namespace

int main()
{
    testSixAxesAndIndependentLimits();
    testNoSaturationAndPreserveUpstreamDiagnostic();
    testUnverifiedConfigurationsAndInput();
    testConversionFailureIsAtomic();
    std::puts("PASS: MotorCommand 6-axis conversion, limits, error atomicity and SIL boundary");
}
