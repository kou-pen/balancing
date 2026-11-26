#include "balancing/bench/MotorBench.hpp"
#include "balancing/app/CanEventPoll.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace balancing {
namespace {
// 旧Cベンチから保持する短時間・低出力のコード上限。機械的安全を保証しない。
constexpr float kHardLimit=0.5f;
constexpr std::uint64_t kEnableTimeoutUs=150000u,kControlGapUs=6000u;
constexpr std::uint32_t kBenchOrigin=0xB000u;
bool elapsed(std::uint64_t now,std::uint64_t at,std::uint64_t duration) noexcept
{ return now>=at && now-at>=duration; }
CanFrame convert(const BMotorCanFrame& raw) noexcept
{
    CanFrame frame{raw.id,{},raw.dlc,raw.extended};
    std::copy_n(raw.data,8,frame.data.begin());
    return frame;
}
BMotorCanFrame convert(const CanFrame& frame) noexcept
{
    BMotorCanFrame raw{frame.id,{},frame.dlc,frame.extended};
    std::copy(frame.data.begin(),frame.data.end(),raw.data);
    return raw;
}
Rs06TxResult outcome(CanTxOutcome result) noexcept
{
    if (result==CanTxOutcome::Sent) return Rs06TxResult::Completed;
    return result==CanTxOutcome::TransmitError?Rs06TxResult::Failed:Rs06TxResult::RetryableFailure;
}
}
MotorBench::MotorBench(CanPort& port,MotorBenchConfig config) noexcept
    :port_{port},config_{config},scheduler_{port},
     router_{*this,config.axis==BenchAxis::Rs06?&scheduler_:nullptr,
                  config.axis==BenchAxis::C620?&scheduler_:nullptr},
     rs06_{config.motorId,config.hostId}
{
    const bool validAxis=config.axis==BenchAxis::Rs06 ||
                         (config.axis==BenchAxis::C620 && config.motorId<=4u);
    if (config.verified && validAxis && config.motorId && std::isfinite(config.limit) &&
        config.limit>0 && config.limit<=kHardLimit && config.pulseDurationMs &&
        config.pulseDurationMs<=200u && config.txTimeoutUs)
        state_=BenchState::Stopping;
}
CanBus MotorBench::bus() const noexcept
{ return config_.axis==BenchAxis::Rs06?CanBus::Can1:CanBus::Can2; }
bool MotorBench::active() const noexcept
{ return state_==BenchState::Enabling || state_==BenchState::Pulsing; }
bool MotorBench::fresh(std::uint64_t now) const noexcept
{
    return bus()==CanBus::Can1?rs06_.status(now).feedbackFresh:c620_.feedback(now).fresh;
}
MotorBenchSummary MotorBench::summary(std::uint64_t now) const noexcept
{ return {state_,stopSent_,fresh(now),busOff_,pending_ && pending_->accepted,txFailures_}; }

bool MotorBench::requestPulse(float output,std::uint64_t now) noexcept
{
    if (state_!=BenchState::Idle || busOff_ || !stopSent_ || pending_ ||
        !lastPoll_ || now<*lastPoll_ || elapsed(now,*lastPoll_,kControlGapUs) ||
        !fresh(now) || !std::isfinite(output) || output==0 || std::fabs(output)>config_.limit)
        return false;
    // 新規Type2の無効化報告まで確認した軸だけ、新しい有効化へ進める。
    if (bus()==CanBus::Can1 && !rs06_.status(now).disableConfirmed) return false;
    output_=output; requestedAt_=now; pulseAt_=now; stopSent_=false;
    lastDue_.reset(); lastPulseSent_.reset();
    if (bus()==CanBus::Can1) { rs06_.requestEnable(); state_=BenchState::Enabling; }
    else state_=BenchState::Pulsing;
    return true;
}

void MotorBench::stop(bool fault) noexcept
{
    if (state_==BenchState::Disabled) return;
    const bool wasActive=active();
    if (wasActive) stopSent_=false;
    output_=0;
    if (bus()==CanBus::Can1) rs06_.requestStop();
    // 旧Type3や非ゼロが停止より後に送られないよう、終端まで待つ。
    if (pending_ && pending_->accepted && pending_->kind!=TxKind::Stop &&
        !pending_->abortRequested) {
        pending_->abortRequested=true;
        (void)port_.requestAbort(pending_->token);
    }
    if (fault || state_==BenchState::Fault) state_=BenchState::Fault;
    else if (wasActive || !stopSent_) state_=BenchState::Stopping;
}
void MotorBench::cancel() noexcept { stop(false); }

void MotorBench::submitStop(std::uint64_t now) noexcept
{
    if (pending_) return;
    BMotorCanFrame raw{};
    std::optional<Rs06TxToken> rsToken;
    if (bus()==CanBus::Can1) {
        const auto request=rs06_.nextRequest();
        if (!request || request->command!=Rs06Command::Stop) return;
        raw=request->frame; rsToken=request->token;
    } else {
        const float zero[4]{};
        if (!b_c620_current_frame(zero,config_.limit,&raw)) { stop(true); return; }
    }
    if (serial_==std::numeric_limits<std::uint64_t>::max()) { stop(true); return; }
    const TxToken token{++serial_,kBenchOrigin};
    // 停止は通常送信と独立した優先枠。失敗後は新トークンで再登録する。
    if (scheduler_.enqueueStop(0,convert(raw),token)==StopEnqueueResult::Queued)
        pending_=Pending{token,TxKind::Stop,rsToken,now,false,false};
    else stop(true);
    if (pending_ && bus()==CanBus::Can2) lastDue_=now;
}

void MotorBench::submitDrive(const BMotorCanFrame& raw,TxKind kind,
    std::optional<Rs06TxToken> rsToken,std::uint64_t now) noexcept
{
    if (pending_ || serial_==std::numeric_limits<std::uint64_t>::max()) {
        if (!pending_) stop(true);
        return;
    }
    const TxToken token{++serial_,kBenchOrigin};
    // 未受付の非ゼロは保存しない。次の周期に現在状態から作り直す。
    const auto result=scheduler_.trySubmit(convert(raw),token,now);
    if (result==CanTxSubmitResult::Accepted) {
        pending_=Pending{token,kind,rsToken,now,true,false};
        if (rsToken) rs06_.onTxResult(*rsToken,Rs06TxResult::Accepted,now);
    } else if (result==CanTxSubmitResult::Invalid) stop(true);
}

void MotorBench::poll(std::uint64_t now) noexcept
{
    if (state_==BenchState::Disabled) return;
    const bool backwards=lastPoll_ && now<*lastPoll_;
    // 時刻逆行の基準を巻き戻さない。停止方向の処理は引き続き行う。
    if (backwards || (active() && (now<requestedAt_ ||
        (lastPoll_ && elapsed(now,*lastPoll_,kControlGapUs))))) stop(true);
    if (backwards) now=*lastPoll_;
    lastPoll_=now;
    pollCanEvents(bus()==CanBus::Can1?&port_:nullptr,
                  bus()==CanBus::Can2?&port_:nullptr,router_);

    if (pending_ && elapsed(now,pending_->atUs,config_.txTimeoutUs)) {
        if (!pending_->abortRequested) {
            ++txFailures_;
            if (pending_->accepted) (void)port_.requestAbort(pending_->token);
            pending_->abortRequested=true;
        }
        stop(true);
    }
    if (state_==BenchState::Enabling) {
        const auto status=rs06_.status(now);
        if (busOff_ || status.motorFault || status.enable==Rs06EnableStatus::Failed ||
            elapsed(now,requestedAt_,kEnableTimeoutUs)) stop(true);
        else if (status.enable==Rs06EnableStatus::Confirmed) {
            state_=BenchState::Pulsing; pulseAt_=now; lastDue_.reset();
        }
    }
    if (state_==BenchState::Pulsing) {
        const bool good=bus()==CanBus::Can1?
            rs06_.status(now).enable==Rs06EnableStatus::Confirmed:fresh(now);
        if (busOff_ || !good || elapsed(now,lastPulseSent_.value_or(pulseAt_),config_.txTimeoutUs))
            stop(true);
        else if (elapsed(now,pulseAt_,std::uint64_t{config_.pulseDurationMs}*1000u)) stop(false);
    }

    if (state_==BenchState::Stopping && stopSent_ &&
        (bus()==CanBus::Can2 || rs06_.status(now).disableConfirmed))
        state_=BenchState::Idle;

    const auto period=bus()==CanBus::Can1?4000u:2000u;
    const bool due=!lastDue_ || elapsed(now,*lastDue_,period);
    if (!pending_) {
        if ((state_==BenchState::Stopping || state_==BenchState::Fault) && !stopSent_)
            submitStop(now);
        else if (state_==BenchState::Enabling && due) {
            lastDue_=now;
            if (const auto r=rs06_.nextRequest())
                if (r->command==Rs06Command::Enable) submitDrive(r->frame,TxKind::Enable,r->token,now);
        } else if (state_==BenchState::Pulsing && due) {
            lastDue_=now;
            BMotorCanFrame raw{};
            float current[4]{};
            if (bus()==CanBus::Can2) current[config_.motorId-1u]=output_;
            const bool encoded=bus()==CanBus::Can1?
                b_rs06_torque_frame(config_.motorId,output_,&raw):
                b_c620_current_frame(current,config_.limit,&raw);
            if (encoded) submitDrive(raw,TxKind::Pulse,{},now);
            else stop(true);
        } else if (bus()==CanBus::Can2 && due &&
                   (state_==BenchState::Idle || (state_==BenchState::Fault && stopSent_))) {
            // 従来通り停止中も2msごとに共通電流ゼロを更新する。
            lastDue_=now; submitStop(now);
        }
    }
    if (pending_ && !pending_->accepted) {
        const auto progress=scheduler_.service(now);
        if (progress.token==pending_->token && progress.result==ScheduledTxResult::Accepted) {
            pending_->accepted=true;
            pending_->atUs=now;
            pending_->abortRequested=false;
            if (pending_->rsToken) rs06_.onTxResult(*pending_->rsToken,Rs06TxResult::Accepted,now);
        }
    }
}

void MotorBench::onCanReceive(CanBus b,CanRxTarget,const CanFrame& frame,std::uint64_t at) noexcept
{
    if (b!=bus()) return;
    const auto raw=convert(frame);
    if (b==CanBus::Can1) {
        // IDの軸/主機とType2/24を照合。他軸を試運転に混ぜない。
        if (!frame.extended || ((frame.id>>8u)&0xffu)!=config_.motorId ||
            (frame.id&0xffu)!=config_.hostId ||
            (frame.id>>24u!=2u && frame.id>>24u!=24u)) return;
        const auto feedback=Rs06Codec::decodeFeedback(raw);
        if (!feedback || !rs06_.onFeedback(*feedback,at) || feedback->faultBits) stop(true);
    } else if (!frame.extended && frame.id==0x200u+config_.motorId) {
        // 3/4番の試運転も可能。通常ロボットの2軸分類に依存しない。
        BC620Feedback feedback{};
        if (!b_c620_parse_feedback(&raw,&feedback)) stop(true);
        else {
            const auto previous=c620_.feedback(at).latest;
            if (previous && at<previous->receivedAtUs) stop(true);
            else c620_.onFeedback(feedback,at);
        }
    }
}
void MotorBench::onCanTransmit(CanBus b,TxToken token,CanTxOutcome result,std::uint64_t at) noexcept
{
    if (b!=bus() || !pending_ || !pending_->accepted || !(pending_->token==token)) return;
    const auto finished=*pending_; pending_.reset();
    if (finished.rsToken) rs06_.onTxResult(*finished.rsToken,outcome(result),at);
    const bool late=elapsed(at,finished.atUs,config_.txTimeoutUs);
    if (late && !finished.abortRequested) { ++txFailures_; stop(true); }
    if (finished.kind==TxKind::Stop) {
        stopSent_=result==CanTxOutcome::Sent;
        if (!stopSent_) { ++txFailures_; stop(true); }
    } else if (result!=CanTxOutcome::Sent) {
        if (active()) { ++txFailures_; stop(true); }
    } else if (finished.kind==TxKind::Pulse && state_==BenchState::Pulsing) {
        // 期限超過の遅延完了で生存証跡を回復させない。
        if (!late) lastPulseSent_=at;
    }
}
void MotorBench::onCanPortStatus(CanBus b,CanEventKind kind,std::uint64_t) noexcept
{
    if (b!=bus()) return;
    if (kind==CanEventKind::BusOff) { busOff_=true; stop(true); }
    else if (kind==CanEventKind::BusRecovered) busOff_=false;
    else if (kind==CanEventKind::RxLost) { c620_.invalidateContinuity(); stop(true); }
}
}
