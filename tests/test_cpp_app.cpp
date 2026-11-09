#include "balancing_app_bridge.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

int main()
{
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

    balancing_app_init();
    assert(balancing_app_control_tick_count() == 0u);
    std::puts("合格：CubeMX C -> C++ application ABI bridge");
    return 0;
}
