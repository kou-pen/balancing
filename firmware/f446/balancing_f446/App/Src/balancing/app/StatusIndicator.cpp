#include "balancing/app/StatusIndicator.hpp"

namespace balancing {
IndicatorState StatusIndicator::update(RobotState state,const std::optional<FaultRecord>& fault,
                                      std::uint64_t nowUs) noexcept
{
    // 状態とFAULT記録のどちらかがFAULTなら表示もFAULT。これは駆動許可ではない。
    if (fault) state=RobotState::Fault;
    IndicatorState out{};
    out.state=state;
    const auto index=static_cast<std::size_t>(state);
    if (!config_.verified || index>=config_.patterns.size()) return out;
    // 時計逆行後は点滅・鳴動を再開せず、再初期化まで無効表示。
    if (lastUs_ && nowUs<*lastUs_) clockInvalid_=true;
    if (clockInvalid_) return out;
    lastUs_=nowUs;
    if (!previous_ || *previous_!=state) {
        previous_=state;
        enteredAtUs_=nowUs;
    }
    const auto& p=config_.patterns[index];
    const auto age=nowUs-enteredAtUs_;
    const bool phase=p.halfPeriodUs==0 || (age/p.halfPeriodUs)%2==0;
    out.configured=true;
    out.red=phase && p.red;
    out.green=phase && p.green;
    out.blue=phase && p.blue;
    out.buzzer=age<std::uint64_t(p.beepMs)*1000u;
    return out;
}
}
