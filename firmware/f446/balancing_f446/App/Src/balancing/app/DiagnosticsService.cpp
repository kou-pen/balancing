#include "balancing/app/DiagnosticsService.hpp"
#include <limits>

namespace balancing {
namespace {
std::uint32_t millis(std::uint64_t us) noexcept
{
    // 通信契約は32bitミリ秒。周回は定義された切り捨て変換で表す。
    return static_cast<std::uint32_t>(us / 1000u);
}
}
DiagnosticsService::DiagnosticsService(DiagnosticSlot* important,std::size_t importantSize,
                                       DiagnosticSlot* normal,std::size_t normalSize,
                                       std::uint32_t tokenOrigin) noexcept
    : important_{important,important ? importantSize : 0},
      normal_{normal,normal ? normalSize : 0},origin_{tokenOrigin}
{}
bool DiagnosticsService::push(Queue& q,DiagnosticEvent event) noexcept
{
    // 満杯なら未完了イベントを上書きしない。欠落数は周回させない。
    if (q.count==q.capacity || eventSequence_==std::numeric_limits<std::uint64_t>::max()) {
        if (q.lost!=std::numeric_limits<std::uint32_t>::max()) ++q.lost;
        return false;
    }
    event.sequence=++eventSequence_;
    if (event.kind==DiagnosticKind::Fault) event.fault.eventSequence=event.sequence;
    const auto tail=(q.head+q.count)%q.capacity;
    q.slots[tail]=DiagnosticSlot{event,0,{}};
    ++q.count;
    return true;
}
bool DiagnosticsService::pushFault(const FaultRecord& record,std::uint64_t eventAtUs) noexcept
{
    DiagnosticEvent e{};
    e.kind=DiagnosticKind::Fault;
    e.occurredAtMs=millis(eventAtUs);
    e.fault={0,e.occurredAtMs,record};
    return push(important_,e);
}
bool DiagnosticsService::pushStateTransition(RobotState from,RobotState to,std::uint64_t nowUs) noexcept
{
    if (from==to) return false;
    DiagnosticEvent e{};
    e.from=from; e.to=to; e.occurredAtMs=millis(nowUs);
    return push(normal_,e);
}
void DiagnosticsService::updateStatus(const DiagnosticStatus& status) noexcept
{
    if (latest_ && status.sampledAtUs<latest_->sampledAtUs) return;
    if (status.fault) {
        const auto& f=*status.fault;
        const bool newFault=!latest_ || !latest_->fault ||
            latest_->fault->occurredAtUs!=f.occurredAtUs ||
            latest_->fault->triggerFlags!=f.triggerFlags;
        const auto previous=latest_ && latest_->fault
            ? static_cast<std::uint16_t>(latest_->fault->subsequentFlags) : 0;
        const bool added=(static_cast<std::uint16_t>(f.subsequentFlags)&~previous)!=0;
        if (newFault || added) (void)pushFault(f,newFault ? f.occurredAtUs : status.sampledAtUs);
    }
    if (latest_ && latest_->state!=status.state)
        (void)pushStateTransition(latest_->state,status.state,status.sampledAtUs);
    latest_=status;
}
std::optional<DiagnosticPart> DiagnosticsService::nextPending() const noexcept
{
    // 重要イベントの全3部分を順に完了させる。通常の既受付分は取り消さない。
    const auto* slot=important_.front();
    if (!slot) slot=normal_.front();
    if (!slot || slot->accepted) return std::nullopt;
    return DiagnosticPart{slot->event,slot->nextIndex};
}
bool DiagnosticsService::onAccepted(const DiagnosticPart& part,TxToken token) noexcept
{
    const auto next=nextPending();
    if (!next || next->event.sequence!=part.event.sequence || next->frameIndex!=part.frameIndex ||
        token.origin!=origin_ || token.serial<=lastSerial_) return false;
    auto* slot=important_.front();
    if (!slot) slot=normal_.front();
    slot->accepted=token;
    lastSerial_=token.serial;
    return true;
}
bool DiagnosticsService::onTerminal(TxToken token,CanTxOutcome outcome) noexcept
{
    for (auto* q : {&important_,&normal_}) {
        auto* slot=q->front();
        if (!slot || !slot->accepted || !(*slot->accepted==token)) continue;
        slot->accepted.reset();
        if (outcome==CanTxOutcome::Sent) {
            const unsigned parts=slot->event.kind==DiagnosticKind::Fault ? 3u : 1u;
            if (++slot->nextIndex==parts) {
                // 受付ではなく最後の送信終端でのみFIFOから削除する。
                q->head=(q->head+1)%q->capacity;
                --q->count;
            }
        }
        return true;
    }
    return false;
}
DiagnosticSummary DiagnosticsService::summary() const noexcept
{
    return {important_.count,normal_.count,important_.lost,normal_.lost,latest_};
}
}
