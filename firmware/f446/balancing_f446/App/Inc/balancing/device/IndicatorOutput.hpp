#pragma once
#include <cstdint>
#include <optional>
namespace balancing {
/** @brief 明示検証済み極性を持つ論理出力。実GPIOの所有権はアダプタに残す。
 * @spec{BOARD-F446-001}
 */
struct IndicatorPin {
    using Write = void (*)(void*,bool) noexcept;
    Write write{nullptr};
    void* context{nullptr};
    bool activeHigh{false};
    bool verified{false};
    // 既存void GPIO通知と、失敗を報告できるアダプタを併存させる。
    using TryWrite = bool (*)(void*,bool) noexcept;
    TryWrite tryWrite{nullptr};
    bool configured() const noexcept { return verified && (write || tryWrite); }
};
class Led final {
public:
    /** @brief 構築では物理出力を変更しない。
     * @spec{BOARD-F446-001}
     */
    explicit Led(IndicatorPin pin={}) noexcept : pin_{pin} {}
    /** @brief 論理ON/OFFを極性に従って出力する。未設定なら拒否。
     * @spec{BOARD-F446-001}
     */
    bool set(bool on) noexcept
    {
        if (!pin_.configured()) return false;
        if (!known_ || on!=on_) {
            if (pin_.tryWrite) {
                // 失敗した出力をキャッシュすると再試行時に未出力のままになる。
                if (!pin_.tryWrite(pin_.context,on==pin_.activeHigh)) {
                    known_=false;
                    return false;
                }
            } else pin_.write(pin_.context,on==pin_.activeHigh);
        }
        known_=true; on_=on;
        return true;
    }
    bool on() noexcept { return set(true); }
    bool off() noexcept { return set(false); }
    bool toggle() noexcept { return known_ && set(!on_); }
private:
    IndicatorPin pin_;
    bool known_{false}, on_{false};
};
class RgbLed final {
public:
    RgbLed(IndicatorPin r={},IndicatorPin g={},IndicatorPin b={}) noexcept
        : r_{r},g_{g},b_{b},configured_{r.configured() && g.configured() && b.configured()} {}
    /** @brief 三色すべてが確認済みの場合のみ出力。部分設定で誤表示しない。
     * @spec{BOARD-F446-001}
     */
    bool setRgb(bool red,bool green,bool blue) noexcept
    {
        if (!configured_) return false;
        return r_.set(red) && g_.set(green) && b_.set(blue);
    }
private:
    Led r_,g_,b_;
    bool configured_;
};

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
