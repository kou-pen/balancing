#pragma once
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
        if (!pin_.verified || !pin_.write) return false;
        if (!known_ || on!=on_) pin_.write(pin_.context,on==pin_.activeHigh);
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
        : r_{r},g_{g},b_{b},configured_{r.verified && r.write && g.verified && g.write && b.verified && b.write} {}
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
}
