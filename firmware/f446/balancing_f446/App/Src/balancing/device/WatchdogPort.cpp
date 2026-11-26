#include "balancing/device/WatchdogPort.hpp"

namespace balancing {
bool WatchdogPort::refresh(const CycleCompletion& c) noexcept
{
    if (!config_.verified || !config_.refresh) return false;
    if (c.completedAtUs<c.ticket.startedAtUs ||
        (last_ && (static_cast<std::uint32_t>(c.ticket.eventSequence-
                   last_->ticket.eventSequence)==0u ||
                   static_cast<std::uint32_t>(c.ticket.eventSequence-
                   last_->ticket.eventSequence)>=0x80000000u ||
                   c.ticket.startedAtUs<last_->completedAtUs))) {
        ++status_.rejected;
        return false;
    }
    // 失敗しても同一証跡を再送しない。次の正常完了だけを受理する。
    last_=c;
    ++status_.attempts;
    const bool ok=config_.refresh(config_.context);
    if (ok) ++status_.successes;
    else ++status_.failures;
    return ok;
}

WatchdogSummary WatchdogPort::summary() const noexcept
{
    auto result=status_;
    result.configured=config_.verified && config_.refresh!=nullptr;
    return result;
}
}
