#include "DriveCycle.hpp"
#include <algorithm>
#include <limits>

namespace balancing {
namespace {
constexpr std::uint32_t origin=0xD000u; // MotorCanLink(0x100|軸ID)と診断とは別
constexpr std::uint64_t rsPeriod=4000, wheelPeriod=2000;

}
DriveCycle::DriveCycle(ControlCycle& control,RobotSequence& sequence,MotorService& motors,
                       MotorCanLink& link,CanTxScheduler& can1,CanTxScheduler& can2,
                       DriveSilConfig config) noexcept
    :control_{control},sequence_{sequence},motors_{motors},link_{link},can1_{can1},can2_{can2},
     frames_{motors.rs06BusConfig().motorIds},config_{config}
{
    configured_=motors_.hasRs06Configuration() && config_.rs06TxTimeoutUs &&
        config_.c620TxTimeoutUs;
}
FaultFlags DriveCycle::txFaults(std::uint64_t nowUs) const noexcept
{
    if (!activeSince_) return FaultFlags::None;
    // 受信が新しくても送信完了が途絶えれば独立してFAULTを要求する。
    for (std::size_t i=0;i<lastSent_.size();++i) {
        const auto since=lastSent_[i].value_or(*activeSince_);
        const auto timeout=i<4?config_.rs06TxTimeoutUs:config_.c620TxTimeoutUs;
        if (nowUs<since || nowUs-since>=timeout) return FaultFlags::ExternalFault;
    }
    return FaultFlags::None;
}
void DriveCycle::withdraw() noexcept
{
    // 旧トルクが停止フレームより後に到着しないよう、受理済み分にも中止を要求。
    // 中止要求の受付は完了ではないので、Schedulerは終端まで停止送信を待つ。
    can1_.invalidatePeriodic();
    can2_.invalidatePeriodic();
    queued_={};
    accepted_={};
    latest_={};
    lastSent_={};
    activeSince_.reset();
    lastRsDue_.reset();
    active_=false;
}
bool DriveCycle::queue(CanBus bus,const MotorFramePreview& preview,std::uint64_t nowUs) noexcept
{
    const bool rs=bus==CanBus::Can1;
    const auto period=rs?rsPeriod:wheelPeriod;
    auto& generation=rs?generation1_:generation2_;
    const auto n=rs?preview.can1Count:std::size_t{1};
    if (generation==std::numeric_limits<std::uint64_t>::max() ||
        serial_>std::numeric_limits<std::uint64_t>::max()-n ||
        nowUs>std::numeric_limits<std::uint64_t>::max()-period) return false;
    PeriodicTxBatch batch{++generation,nowUs+period,{}};
    // 未受付分は常に最新世代で全面置換。古い世代をFIFOへ蓄積しない。
    for (std::size_t i=0;i<n;++i) {
        const auto& raw=rs?preview.can1Type1[i]:preview.can2Current;
        batch.requests[i]=PeriodicTxRequest{raw,{++serial_,origin}};
    }
    auto& scheduler=rs?can1_:can2_;
    const auto status=scheduler.replacePeriodic(batch,nowUs);
    if (status!=PeriodicBatchResult::Queued) return false;
    for (std::size_t i=0;i<n;++i) {
        queued_[rs?i:4]=batch.requests[i]->token;
        latest_[rs?i:4]=batch.requests[i]->token;
    }
    return true;
}
DriveSilResult DriveCycle::step(ControlTickInput input) noexcept
{
    const auto now=input.observations.motors.sampledAtUs;
    input.armFeedback=motors_.rs06ArmFeedback(now);
    input.interlocks.driveLimitsVerified &= configured_;
    input.observedFaults=input.observedFaults|pending_|txFaults(now)|motors_.takeObservedFaults(now);
    pending_=FaultFlags::None;
    if (lastTick_ && now<=*lastTick_)
        input.observedFaults=input.observedFaults|FaultFlags::ControlTiming;
    if (!lastTick_ || now>*lastTick_) lastTick_=now;

    DriveSilResult result{};
    result.control=control_.step(input);
    auto decision=result.control.sequence;
    const bool allowed=decision.desiredMotorState==DesiredMotorState::DriveAllowed;
    // BALANCINGへの遷移周期は既存ControlCycleが数値指令をまだ返さない。
    // その周期は車輪ゼロを送り、次周期から通常送信する。
    const bool hasCommand=allowed && result.control.motorCommandPreview.has_value();
    if (allowed && !activeSince_) {
        activeSince_=now;
        lastSent_={};
    }
    if (!allowed && activeSince_) withdraw();
    if (hasCommand) {
        const auto plan=frames_.compose(decision,*result.control.motorCommandPreview);
        if (plan.status==MotorFramePreview::Status::Ready) {
            // ARMING/遷移周期のゼロ電流終端は通常指令の証跡にしない。
            if (!active_) accepted_={};
            result.frames=plan;
            const bool rsDue=!lastRsDue_ || now-*lastRsDue_>=rsPeriod;
            bool ok=true;
            if (rsDue) {
                // 4ms位相。遅れた過去周期は後追いしない。
                lastRsDue_=lastRsDue_ ? *lastRsDue_+
                    ((now-*lastRsDue_)/rsPeriod)*rsPeriod : now;
                ok=queue(CanBus::Can1,plan,now);
            }
            if (ok) ok=queue(CanBus::Can2,plan,now);
            if (ok) {
                active_=true;
                result.outputInhibited=false;
            }
        }
        if (result.outputInhibited) {
            // 符号化・予約失敗なら即時出力を止め、次周期に状態機械へ故障を渡す。
            // RobotSequenceを同周期に二重更新しない。
            pending_=pending_|FaultFlags::ExternalFault;
            withdraw();
            decision.desiredMotorState=DesiredMotorState::StopAll;
            result.frames.reset();
        }
    }
    if (result.outputInhibited) {
        auto zeroDecision=decision;
        zeroDecision.desiredMotorState=DesiredMotorState::StopAll;
        const auto zero=frames_.compose(zeroDecision,{});
        result.frames=zero;
        if (zero.status!=MotorFramePreview::Status::ZeroCurrentOnly ||
            !queue(CanBus::Can2,zero,now)) pending_=pending_|FaultFlags::ExternalFault;
    }
    motors_.applySequence(decision,now);
    link_.submitPending(now);
    poll(now);
    return result;
}
void DriveCycle::service(CanBus bus,std::uint64_t nowUs) noexcept
{
    auto& tx=bus==CanBus::Can1?can1_:can2_;
    // 1回のpollで最大4回。メールボックス満杯で待ち続けない。
    for (unsigned i=0;i<4;++i) {
        const auto progress=tx.service(nowUs);
        if (bus==CanBus::Can1) link_.onScheduledTxProgress(progress,nowUs);
        if (progress.result==ScheduledTxResult::Accepted ||
            progress.result==ScheduledTxResult::Expired ||
            progress.result==ScheduledTxResult::Invalid) {
            const auto begin=bus==CanBus::Can1?0u:4u;
            const auto end=bus==CanBus::Can1?4u:5u;
            for (auto axis=begin;axis<end;++axis) {
                if (queued_[axis].serial && queued_[axis]==progress.token) {
                    if (progress.result==ScheduledTxResult::Accepted)
                        accepted_[axis]=progress.token;
                    else if (progress.result==ScheduledTxResult::Invalid)
                        pending_=pending_|FaultFlags::ExternalFault;
                    queued_[axis]={};
                }
            }
        } else break;
    }
}
void DriveCycle::poll(std::uint64_t nowUs) noexcept
{
    pending_=pending_|txFaults(nowUs);
    if (active_ && pending_!=FaultFlags::None) {
        withdraw();
        // 出力側を停止方向へだけ上書きする。RobotStateの遷移は次tickで
        // RobotSequenceに委ね、別の状態機械を作らない。
        const RobotSequenceResult stopped{sequence_.state(),DesiredMotorState::StopAll,0};
        motors_.applySequence(stopped,nowUs);
        const auto zero=frames_.compose(stopped,{});
        if (zero.status==MotorFramePreview::Status::ZeroCurrentOnly)
            (void)queue(CanBus::Can2,zero,nowUs);
        link_.submitPending(nowUs);
    }
    service(CanBus::Can1,nowUs);
    service(CanBus::Can2,nowUs);
}
void DriveCycle::onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,
                             std::uint64_t atUs) noexcept
{
    if (!token.serial || token.origin!=origin) return;
    const auto begin=bus==CanBus::Can1?0u:4u;
    const auto end=bus==CanBus::Can1?4u:5u;
    for (auto i=begin;i<end;++i) {
        if (!(accepted_[i]==token)) continue;
        accepted_[i]={};
        // 非駆動中のゼロ完了、停止で中止した旧世代は通常運転の鮮度を更新しない。
        if (active_ && activeSince_ && atUs>=*activeSince_) {
            if (outcome==CanTxOutcome::Sent) {
                // 期限を超えた終端や置換済み世代の終端は占有だけ解放。
                // 旧C11契約に従い最新指令の送信鮮度を回復させない。
                const auto since=lastSent_[i].value_or(*activeSince_);
                const auto timeout=i<4?config_.rs06TxTimeoutUs:config_.c620TxTimeoutUs;
                if (atUs>=since && atUs-since>=timeout)
                    pending_=pending_|FaultFlags::ExternalFault;
                else if (latest_[i]==token && atUs>=since) lastSent_[i]=atUs;
            } else if (outcome==CanTxOutcome::TransmitError) pending_=pending_|FaultFlags::ExternalFault;
        }
        return;
    }
}
}
