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
    std::puts("合格：CubeMX C -> C++ application ABI bridge");
    return 0;
}
