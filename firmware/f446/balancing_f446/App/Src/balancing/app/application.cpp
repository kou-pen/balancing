#include "../../../Inc/balancing_app_bridge.h"

#include <cstdint>

namespace balancing {

class Application final {
public:
    void init() noexcept
    {
        control_ticks_ = 0;
        initialized_ = true;
    }

    void controlTick() noexcept
    {
        if (!initialized_) {
            return;
        }

        ++control_ticks_;
        // TODO(T-F446-11): 既存のC11制御コア、CAN1/2アダプタと結合する。
        // 明示的な安全確認とモータID/換算値の確定までは出力を行わない。
    }

    void poll() noexcept
    {
        if (!initialized_) {
            return;
        }
        // TODO(T-F446-05): 非ブロッキング通信・診断処理を置く。
    }

    std::uint32_t controlTickCount() const noexcept
    {
        return control_ticks_;
    }

private:
    std::uint32_t control_ticks_{0};
    bool initialized_{false};
};

// 静的に一つだけ保持。malloc/newも静的初期化時のHAL利用も行わない。
Application application;

} // namespace balancing

// main.cはCとしてコンパイルするので、公開関数だけCリンケージにする。
extern "C" {

void balancing_app_init(void)
{
    balancing::application.init();
}

void balancing_app_control_tick(void)
{
    balancing::application.controlTick();
}

void balancing_app_poll(void)
{
    balancing::application.poll();
}

uint32_t balancing_app_control_tick_count(void)
{
    return balancing::application.controlTickCount();
}

} // extern "C"
