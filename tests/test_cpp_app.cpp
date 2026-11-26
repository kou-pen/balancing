#include "balancing_app_bridge.h"
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/device/ImuObservation.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace {
balancing::MotorObservationSnapshot observed{};
bool available{};
bool sampled{};
unsigned acknowledgements{};
struct Gpio { unsigned calls{}; bool high{},fail{}; };
bool gpioWrite(void* ctx,bool high) noexcept {
    auto& p=*static_cast<Gpio*>(ctx); ++p.calls;
    if (p.fail) return false;
    p.high=high; return true;
}

}
std::optional<balancing::MotorObservationSnapshot> balancing::motorObservation() noexcept
{
    if (!available || !sampled) return std::nullopt;
    return observed;
}
bool balancing::acknowledgeMotorObservation(std::uint64_t generation) noexcept
{
    assert(generation == observed.generation);
    ++acknowledgements;
    return true;
}
balancing::ImuAttitude balancing::imuAttitude(std::uint64_t nowUs) noexcept
{
    return {nowUs,0,0,0,0,0,false,false};
}

void balancing::serviceCan2ApplicationTx() noexcept {}
std::optional<std::uint64_t> balancing::observationClockUs() noexcept
{ return available ? std::optional<std::uint64_t>{observed.motors.sampledAtUs} : std::nullopt; }

// CANポートはこの試験で起動しない。リンクに必要なIRQマスクだけ模擬。
std::uint32_t __get_PRIMASK() { return 0u; }
void __disable_irq() {}
void __enable_irq() {}

int main()
{
    BF446AppTimingSnapshot timing{};
    assert(!balancing_app_timing_snapshot(&timing));
    assert(!balancing::applicationObservation());
    // 呼び出し前も非駆動かつ未初期化で安全。
    assert(balancing_app_control_tick_count() == 0u);
    balancing_app_control_tick();
    assert(balancing_app_control_tick_count() == 0u);

    balancing_app_init();
    for (std::uint32_t i = 0; i < 500u; ++i) {
        balancing_app_control_tick();
        balancing_app_poll();
    }
    assert(balancing_app_control_tick_count() == 500u);
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid);
    assert(timing.control_ticks == 500u);
    assert(!balancing_app_timing_snapshot(nullptr));

    // CAN1起動直後の未採取と時刻0の有効な初回採取は別状態。
    available = true;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid);
    sampled = true;
    observed.motors.sampledAtUs = 0;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && !timing.elapsed_valid && timing.sampled_at_us == 0);
    auto report = balancing::applicationObservation();
    assert(report && report->motors && report->cycle);
    assert(report->cycle->estimateError == balancing::StateEstimateError::InvalidPeriod);
    assert(report->cycle->sequence.state == balancing::RobotState::Disarmed);
    observed.motors.sampledAtUs = 2000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && timing.elapsed_valid && timing.elapsed_us == 2000);
    report = balancing::applicationObservation();
    assert(report->cycle->estimateError == balancing::StateEstimateError::ImuUnavailable);
    assert(!report->cycle->motorCommandPreview && !report->cycle->mathematicalTorques);
    assert(report->cycle->sequence.desiredMotorState == balancing::DesiredMotorState::StopAll);
    observed.motors.sampledAtUs = 8000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 6000);
    // 遅延は測定結果に残すだけで、駆動やFAULTの判断はしない。
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid && timing.elapsed_us == 0);
    assert(timing.duplicate_samples == 1);
    assert(!balancing::applicationObservation()->cycle);
    observed.motors.sampledAtUs = 7000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid && timing.clock_regressions == 1);
    observed.motors.sampledAtUs = 10000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 2000); // 逆行で基準を変えない
    available = false;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid && timing.elapsed_us == 0);
    available = true;
    observed.motors.sampledAtUs = 20000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && !timing.elapsed_valid); // 欠損区間の差分を公表しない
    assert(balancing::applicationObservation()->cycle->estimateError == balancing::StateEstimateError::InvalidPeriod);
    assert(acknowledgements == 5);
    observed.observedFaults = balancing::FaultFlags::BusOff;
    observed.motors.sampledAtUs = 22000;
    ++observed.generation;
    balancing_app_control_tick();
    report = balancing::applicationObservation();
    assert(report->cycle->sequence.state == balancing::RobotState::Fault);
    assert(report->fault && report->fault->primaryCause == balancing::FaultFlags::BusOff);
    assert(!report->cycle->motorCommandPreview);
    observed.observedFaults = balancing::FaultFlags::None;
    observed.motors.sampledAtUs = 24000;
    ++observed.generation;
    balancing_app_control_tick();
    assert(balancing::applicationObservation()->cycle->sequence.state == balancing::RobotState::Fault);


    balancing_app_init();
    assert(balancing_app_control_tick_count() == 0u);
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid);
    assert(timing.duplicate_samples == 0 && timing.clock_regressions == 0);
    // 非駆動Applicationに明示された模擬GPIOだけを接続。
    using namespace balancing;
    Can2ObservationProfile profile{};
    profile.communication={200000,100000,20000,20,1000,4,0xCA2u,false};
    std::array<Gpio,4> gpio{};
    profile.indicator.verified=true;
    profile.indicator.patterns[0]={false,true,false,0,0};
    profile.indicator.patterns[3]={true,false,false,4,1000};
    profile.indicatorOutput.verified=true;
    for (unsigned i=0;i<4;++i)
        profile.indicatorOutput.pins[i]={nullptr,&gpio[i],true,true,gpioWrite};
    assert(initializeCan2Observation(profile));
    for (const auto& p:gpio) assert(!p.calls);
    observed.observedFaults=FaultFlags::None;
    observed.motors.sampledAtUs=30000;
    balancing_app_control_tick();
    assert(gpio[1].high && !gpio[0].high && !gpio[3].high);
    assert(applicationObservation()->indicatorOutput.applied);
    observed.observedFaults=FaultFlags::BusOff;
    observed.motors.sampledAtUs=32000;
    balancing_app_control_tick();
    assert(gpio[0].high && !gpio[1].high && gpio[3].high);
    observed.observedFaults=FaultFlags::None;
    observed.motors.sampledAtUs=33000;
    balancing_app_control_tick(); assert(!gpio[0].high && gpio[3].high);
    // 出力故障はFAULTの主因を変更しない。古いパターンを継続出力しない。
    gpio[0].fail=true;
    observed.motors.sampledAtUs=34000;
    balancing_app_control_tick();
    auto display=applicationObservation();
    assert(display->indicatorOutput.failed && !display->indicatorOutput.applied);
    assert(display->fault->primaryCause==FaultFlags::BusOff);
    assert(!gpio[3].high);
    auto n=gpio[0].calls;
    observed.motors.sampledAtUs=36000;
    balancing_app_control_tick(); assert(gpio[0].calls==n);
    assert(applicationObservation()->diagnostics.latest->state==RobotState::Fault);
    std::puts("合格：CubeMX C -> C++ application ABI bridge");
    return 0;
}
