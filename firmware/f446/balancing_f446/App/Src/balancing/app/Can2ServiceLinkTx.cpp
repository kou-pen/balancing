#include "balancing/app/Can2ServiceLink.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"
#include <limits>

namespace balancing {
namespace {
std::optional<CanFrame> faultFrame(const DiagnosticPart& part) noexcept
{
    using namespace protocol;
    if (part.event.kind!=DiagnosticKind::Fault) return std::nullopt;
    const auto& e=part.event.fault;
    const auto id=static_cast<std::uint16_t>(e.eventSequence);
    const auto& f=e.record;
    // 内部イベントの下位16bitを3パート共通にし、元FAULT時刻と追加異常時刻を分離。
    switch(part.frameIndex) {
    case 0: return encodeDiagnostic(FaultPart1{id,e.occurredAtMs,static_cast<std::uint16_t>(f.primaryCause)});
    case 1: return encodeDiagnostic(FaultPart2{id,static_cast<std::uint16_t>(f.triggerFlags),
                                               static_cast<std::uint16_t>(f.subsequentFlags)});
    case 2: return encodeDiagnostic(FaultPart3{id,static_cast<std::uint32_t>(f.occurredAtUs/1000u)});
    default: return std::nullopt;
    }
}
}
void Can2ServiceLink::submitPending(CanTxScheduler& tx,std::uint64_t nowUs) noexcept
{
    using namespace protocol;
    if (!enabled_ || busOff_ || serial_==std::numeric_limits<std::uint64_t>::max()) return;
    // 期限切れは未受付の応答だけを捨てる。操作や電源要求は再実行しない。
    while(responseCount_ && !recent(responses_[0].atUs,nowUs)) {
        removeResponse();
        countDroppedResponse();
    }
    if (powerExchange_ && !recent(powerExchange_->atUs,nowUs)) powerExchange_.reset();
    const TxToken token{serial_+1,config_.tokenOrigin};
    const auto send=[&](const CanFrame& frame) {
        const auto result=tx.trySubmitBackground(frame,token,nowUs);
        if (result==CanTxSubmitResult::Accepted) ++serial_;
        return result==CanTxSubmitResult::Accepted;
    };
    // FAULTは最優先の背景データ。通常状態遷移のワイヤIDは未決定のため送らない。
    const auto part=diagnostics_.nextPending();
    if (part) if (const auto frame=faultFrame(*part)) {
        if (send(*frame)) (void)diagnostics_.onAccepted(*part,token);
        return;
    }
    if (responseCount_) {
        if (send(responses_[0].frame)) { responseInFlight_=token; removeResponse(); }
        return;
    }
    if (powerExchange_ && !powerExchange_->token) {
        const auto frame=encodePowerRequest({NodeId::F446,powerExchange_->kind,powerExchange_->id});
        if (frame && send(*frame)) powerExchange_->token=token;
        return;
    }
    // 最新の有効観測だけを周期配信。購読期限を過ぎたら基本周期へ戻る。
    if (!telemetryAtUs_ || nowUs<*telemetryAtUs_ ||
        nowUs-*telemetryAtUs_>=config_.telemetryLifetimeUs) return;
    for (std::size_t n=0;n<telemetry_.size();++n) {
        const auto i=(nextTelemetry_+n)%telemetry_.size();
        if (!telemetry_[i]) continue;
        const auto& lease=leases_[i];
        const auto period=nowUs<lease.untilUs?std::uint64_t{lease.periodMs}*1000u:config_.basePeriodUs;
        if (lastTelemetryTx_[i] && (nowUs<*lastTelemetryTx_[i] ||
            nowUs-*lastTelemetryTx_[i]<period)) continue;
        if (send(*telemetry_[i])) {
            lastTelemetryTx_[i]=nowUs;
            nextTelemetry_=(i+1)%telemetry_.size();
        }
        return;
    }
}
}
