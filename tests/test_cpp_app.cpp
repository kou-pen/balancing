#include "balancing_app_bridge.h"
#include "balancing/device/Can1Observation.h"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace {
BF446Can1Snapshot observed{};
bool available{};
}
extern "C" bool balancing_f446_can1_snapshot(BF446Can1Snapshot* out)
{
    if (!available || out == nullptr) return false;
    *out = observed;
    return true;
}

int main()
{
    BF446AppTimingSnapshot timing{};
    assert(!balancing_app_timing_snapshot(&timing));
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
    observed.sampled = true;
    observed.sampled_at_us = 0;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && !timing.elapsed_valid && timing.sampled_at_us == 0);
    observed.sampled_at_us = 2000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && timing.elapsed_valid && timing.elapsed_us == 2000);
    observed.sampled_at_us = 8000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 6000);
    // 遅延は測定結果に残すだけで、駆動やFAULTの判断はしない。
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid && timing.elapsed_us == 0);
    assert(timing.duplicate_samples == 1);
    observed.sampled_at_us = 7000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid && timing.clock_regressions == 1);
    observed.sampled_at_us = 10000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 2000); // 逆行で基準を変えない
    available = false;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid && timing.elapsed_us == 0);
    available = true;
    observed.sampled_at_us = 20000;
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && !timing.elapsed_valid); // 欠損区間の差分を公表しない


    balancing_app_init();
    assert(balancing_app_control_tick_count() == 0u);
    assert(balancing_app_timing_snapshot(&timing));
    assert(!timing.sample_valid && !timing.elapsed_valid);
    assert(timing.duplicate_samples == 0 && timing.clock_regressions == 0);
    std::puts("合格：CubeMX C -> C++ application ABI bridge");
    return 0;
}
