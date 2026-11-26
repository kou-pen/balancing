#include "balancing/device/StatusIndicatorPort.hpp"
#include <algorithm>
#include <limits>

namespace balancing {
StatusIndicatorPort::StatusIndicatorPort(const IndicatorOutputConfig& c) noexcept
    : outputs_{Led{c.pins[0]},Led{c.pins[1]},Led{c.pins[2]},Led{c.pins[3]}} {
    // 全出力が揃わない限り部分出力しない。構築は一切のGPIO書込みを行わない。
    summary_.configured=c.verified && std::all_of(c.pins.begin(),c.pins.end(),
        [](const IndicatorPin& p){return p.configured() && p.tryWrite;});
}
bool StatusIndicatorPort::setAll(const std::array<bool,4>& levels) noexcept {
    bool ok=true;
    // 1チャネル失敗で他チャネルのOFF要求を短絡しない。
    for (std::size_t i=0;i<outputs_.size();++i) {
        if (!outputs_[i].set(levels[i])) {
            ok=false;
            if (summary_.writeFailures!=std::numeric_limits<std::uint32_t>::max())
                ++summary_.writeFailures;
        }
    }
    return ok;
}
bool StatusIndicatorPort::apply(const IndicatorState& s) noexcept {
    if (!summary_.configured || summary_.failed) return false;
    if (!s.configured) { disable(); return false; }
    if (!setAll({s.red,s.green,s.blue,s.buzzer})) {
        // 状態表示の失敗で駆動許可やFAULT原因を変更しない。出力の再開は再初期化のみ。
        summary_.failed=true; summary_.applied.reset();
        (void)setAll({false,false,false,false});
        return false;
    }
    summary_.applied=s;
    return true;
}
void StatusIndicatorPort::disable() noexcept {
    if (!summary_.configured || summary_.failed) return;
    if (!setAll({false,false,false,false})) summary_.failed=true;
    summary_.applied.reset();
}
}
