#pragma once
#include "balancing/device/Led.hpp"
#include <cstdint>
#include <optional>

namespace balancing {
/** @brief 時刻差で単発鳴動を管理する。HAL_Delayを呼ばない。
 * @spec{BOARD-F446-001}
 */
class Buzzer final {
public:
    explicit Buzzer(IndicatorPin pin={}) noexcept : output_{pin} {}
    /** @brief 確認済み出力の単発鳴動。期限は絶対時刻加算でなく差分を用いる。
     * @spec{BOARD-F446-001}
     */
    bool beep(std::uint32_t durationMs,std::uint64_t nowUs) noexcept
    {
        if (lastUs_ && nowUs<*lastUs_) { stop(); return false; }
        if (!durationMs) return stop();
        if (!output_.on()) return false;
        lastUs_=nowUs; startedAtUs_=nowUs;
        durationUs_=std::uint64_t(durationMs)*1000u; active_=true;
        return true;
    }
    /** @brief 鳴動終了。未検証ピンなら書込みなし。
     * @spec{BOARD-F446-001}
     */
    bool stop() noexcept { active_=false; return output_.off(); }
    /** @brief 主ループの時刻で期限を評価する。逆行は停止。
     * @spec{BOARD-F446-001}
     */
    void update(std::uint64_t nowUs) noexcept
    {
        if (lastUs_ && nowUs<*lastUs_) { stop(); return; }
        lastUs_=nowUs;
        if (active_ && nowUs-startedAtUs_>=durationUs_) stop();
    }
    bool active() const noexcept { return active_; }
private:
    Led output_;
    std::optional<std::uint64_t> lastUs_{};
    std::uint64_t startedAtUs_{0},durationUs_{0};
    bool active_{false};
};
}
