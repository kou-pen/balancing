#include "balancing/app/CycleHealthMonitor.hpp"

namespace balancing {
bool CycleHealthMonitor::begin(CycleTicket ticket) noexcept
{
    const bool unfinished=status_.inProgress;
    // 新しい開始が来ても前回の未完了を成功に読み替えない。
    abandon();
    if (clockHighWater_ && ticket.startedAtUs<*clockHighWater_) {
        ++status_.clockRegressions;
        eligible_=false;
        // 逆行した時刻や番号を新たな基準にせず、復帰を待つ。
        return false;
    }
    if (status_.lastStarted) {
        const auto delta=static_cast<std::uint32_t>(ticket.eventSequence-
                                                    status_.lastStarted->eventSequence);
        if (delta==0u) { ++status_.duplicateEvents; return false; }
        // 半周以上の差は逆順とみなす。0xffffffff→0 は差1として受理。
        if (delta>=0x80000000u) { ++status_.oldEvents; return false; }
        status_.missedEvents+=delta-1u;
        eligible_=delta==1u && !unfinished;
    } else {
        eligible_=!unfinished;
    }
    clockHighWater_=ticket.startedAtUs;
    status_.lastStarted=ticket;
    status_.inProgress=true;
    return true;
}

std::optional<CycleCompletion> CycleHealthMonitor::complete(CycleTicket ticket,
    std::uint64_t completedAtUs,bool processingCompleted) noexcept
{
    // 旧完了通知は現在進行中の処理に影響させない。
    if (!status_.inProgress || !status_.lastStarted ||
        ticket.eventSequence!=status_.lastStarted->eventSequence ||
        ticket.startedAtUs!=status_.lastStarted->startedAtUs) return std::nullopt;
    status_.inProgress=false;
    status_.refreshEligible=false;
    if (completedAtUs<ticket.startedAtUs) {
        ++status_.clockRegressions;
        ++status_.incompleteCycles;
        return std::nullopt;
    }
    clockHighWater_=completedAtUs;
    if (!processingCompleted) {
        ++status_.incompleteCycles;
        return std::nullopt;
    }
    const CycleCompletion completion{ticket,completedAtUs};
    status_.lastCompleted=completion;
    ++status_.completedCycles;
    // 欠落は診断に残す。6msのFAULT判断は既存ControlCycleに委ねる。
    status_.refreshEligible=eligible_;
    return eligible_ ? std::optional<CycleCompletion>{completion}:std::nullopt;
}

void CycleHealthMonitor::abandon() noexcept
{
    if (status_.inProgress) ++status_.incompleteCycles;
    status_.inProgress=false;
    status_.refreshEligible=false;
    eligible_=false;
}

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
