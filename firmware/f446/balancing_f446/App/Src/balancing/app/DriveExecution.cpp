#include "balancing/app/DriveExecution.hpp"

namespace balancing {
namespace {
constexpr std::uint32_t driveOrigin=0xD000u,zeroOrigin=0xD001u;
constexpr std::uint64_t rsPeriod=4000;
}

DriveExecution::DriveExecution(RobotSequence& sequence,MotorService& motors,
    MotorCanLink& link,CanTxScheduler& can1,CanTxScheduler& can2,
    DriveExecutionConfig config,Compose compose) noexcept
    :sequence_{sequence},motors_{motors},link_{link},tx_{can1,can2,compose?driveOrigin:zeroOrigin},
     config_{config},compose_{compose}
{
    // 生成境界が存在しても軸割当と独立した期限が成立しなければARMを許可しない。
    // 通常実機起動には生成関数そのものを渡さない。
    configured_=compose_ && motors_.hasRs06Configuration() &&
        config_.rs06TxTimeoutUs && config_.c620TxTimeoutUs;
}
void DriveExecution::fault(FaultFlags flags) noexcept
{
    pending_=pending_|flags;
    outputFaults_=outputFaults_|flags;
}
FaultFlags DriveExecution::txFaults(std::uint64_t nowUs) const noexcept
{
    return tx_.deadlineFault(nowUs)?FaultFlags::ExternalFault:FaultFlags::None;
}
void DriveExecution::withdraw() noexcept
{
    tx_.withdraw();
    activeSince_.reset();
    lastRsDue_.reset();
    active_=false;
}
bool DriveExecution::queue(CanBus bus,const MotorFramePreview& preview,std::uint64_t nowUs) noexcept
{
    if (bus==CanBus::Can1) return tx_.queue(bus,preview.can1Type1,preview.can1Count,nowUs);
    if (preview.status==MotorFramePreview::Status::ZeroCurrentOnly) return tx_.queueZero(nowUs);
    return tx_.queue(bus,{preview.can2Current},1,nowUs);
}
bool DriveExecution::acceptStopCycle(const ApplicationObservation& obs,std::uint64_t now) noexcept
{
    // 停止専用は番号付きの新規回収だけを新世代にする。欠落・重複を後追いしない。
    const auto& health=obs.cycleHealth;
    if (!health.inProgress || !health.lastStarted || !obs.motors) return false;
    const auto ticket=*health.lastStarted;
    const bool fresh=(!lastGeneration_ || obs.motors->generation>*lastGeneration_) &&
        now>=ticket.startedAtUs && (!lastClock_ || now>=*lastClock_);
    const bool consecutive=!lastTicket_ ||
        (static_cast<std::uint32_t>(ticket.eventSequence-lastTicket_->eventSequence)==1u &&
         ticket.startedAtUs>lastTicket_->startedAtUs);
    if (!lastTicket_ || ticket.startedAtUs>lastTicket_->startedAtUs) lastTicket_=ticket;
    if (!fresh) return false;
    lastGeneration_=obs.motors->generation;
    if (!consecutive) return false;
    cycleSeen_=true;
    return true;
}

DriveExecutionResult DriveExecution::step(ControlCycle& control,const ApplicationObservation& obs,
                                           ControlTickInput input) noexcept
{
    const auto now=input.observations.motors.sampledAtUs;
    input.interlocks.driveLimitsVerified &= configured_;
    input.observedFaults=input.observedFaults|pending_;
    pending_=FaultFlags::None;
    if (compose_) {
        input.armFeedback=motors_.rs06ArmFeedback(now);
        input.observedFaults=input.observedFaults|txFaults(now)|motors_.takeObservedFaults(now);
        if (lastTick_ && now<=*lastTick_)
            input.observedFaults=input.observedFaults|FaultFlags::ControlTiming;
        if (!lastTick_ || now>*lastTick_) lastTick_=now;
    }

    // 両モードとも推定とRobotSequenceは同一のControlCycleを一度だけ実行。
    // 停止専用ではその前に新規ゼロ予約を確定し、従来の観測境界を保持する。
    if (!compose_ && acceptStopCycle(obs,now) && !tx_.queueZero(now))
        fault(FaultFlags::ExternalFault);
    DriveExecutionResult result{};
    result.control=control.step(input);
    auto decision=result.control.sequence;
    if (!compose_) {
        // 生成境界がない場合、アダプタが現在の時計でpollを呼びStopAllのみ適用。
        // 未成立周期で新しいゼロを作らず、以前の停止要求だけを進める。
        return result;
    }

    const bool allowed=configured_ && decision.desiredMotorState==DesiredMotorState::DriveAllowed;
    // BALANCING遷移周期は数値指令がまだないため車輪ゼロ。
    const bool hasCommand=allowed && result.control.motorCommandPreview.has_value();
    if (allowed && !activeSince_) {
        activeSince_=now;
        tx_.beginActive(now,config_.rs06TxTimeoutUs,config_.c620TxTimeoutUs);
    }
    if (!allowed && activeSince_) withdraw();
    if (hasCommand) {
        const auto plan=compose_(decision,*result.control.motorCommandPreview);
        if (plan.status==MotorFramePreview::Status::Ready && plan.fixtureOnly) {
            tx_.markDriving();
            result.frames=plan;
            const bool rsDue=!lastRsDue_ || now-*lastRsDue_>=rsPeriod;
            bool ok=true;
            if (rsDue) {
                // 過去の4ms周期は後追いせず、今の世代だけを登録。
                lastRsDue_=lastRsDue_?*lastRsDue_+
                    ((now-*lastRsDue_)/rsPeriod)*rsPeriod:now;
                ok=queue(CanBus::Can1,plan,now);
            }
            if (ok) ok=queue(CanBus::Can2,plan,now);
            if (ok) { active_=true;result.outputInhibited=false; }
        }
        if (result.outputInhibited) {
            // 一部の予約だけ成立しても旧指令を残さない。状態遷移は次周期へ委ねる。
            fault(FaultFlags::ExternalFault);
            withdraw();
            decision.desiredMotorState=DesiredMotorState::StopAll;
            result.frames.reset();
        }
    }
    if (!configured_) decision.desiredMotorState=DesiredMotorState::StopAll;
    if (result.outputInhibited) {
        auto stopped=decision;
        stopped.desiredMotorState=DesiredMotorState::StopAll;
        const auto zero=compose_(stopped,{});
        result.frames=zero;
        if (zero.status!=MotorFramePreview::Status::ZeroCurrentOnly ||
            !tx_.queueZero(now)) fault(FaultFlags::ExternalFault);
    }
    motors_.applySequence(decision,now);
    link_.submitPending(now);
    poll(now);
    return result;
}

void DriveExecution::service(CanBus bus,std::uint64_t nowUs) noexcept
{
    // 通常停止のCAN2は1回、SILは最大4回。どちらも同じ受付・終端管理。
    const unsigned limit=(!compose_ && bus==CanBus::Can2)?1u:4u;
    for (unsigned i=0;i<limit;++i) {
        const auto progress=tx_.service(bus,link_,nowUs);
        if (tx_.takeFault()) fault(FaultFlags::ExternalFault);
        if (progress.result!=ScheduledTxResult::Accepted &&
            (!compose_ || (progress.result!=ScheduledTxResult::Expired &&
                          progress.result!=ScheduledTxResult::Invalid))) break;
    }
}
void DriveExecution::poll(std::uint64_t nowUs) noexcept
{
    if (!compose_) {
        if (!cycleSeen_ || (lastClock_ && nowUs<*lastClock_)) return;
        lastClock_=nowUs;
        motors_.applySequence({RobotState::Disarmed,DesiredMotorState::StopAll,0},nowUs);
        link_.submitPending(nowUs);
    } else {
        fault(txFaults(nowUs));
        if (active_ && pending_!=FaultFlags::None) {
            withdraw();
            const RobotSequenceResult stopped{sequence_.state(),DesiredMotorState::StopAll,0};
            motors_.applySequence(stopped,nowUs);
            // 停止ゼロは生成関数を経由せず既存の共通ゼロ入口を用いる。
            (void)tx_.queueZero(nowUs);
            link_.submitPending(nowUs);
        }
    }
    service(CanBus::Can1,nowUs);
    service(CanBus::Can2,nowUs);
}
void DriveExecution::onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,
                                   std::uint64_t atUs) noexcept
{
    tx_.onTxTerminal(bus,token,outcome,atUs);
    if (tx_.takeFault() && (!compose_ || active_)) fault(FaultFlags::ExternalFault);
}
StopOutputSummary DriveExecution::stopSummary() const noexcept
{
    const auto& w=tx_.summary()[4];
    StopOutputSummary summary{};
    summary.enabled=!compose_;
    summary.zeroQueued=w.queued;summary.zeroAccepted=w.accepted;
    summary.zeroSent=w.sent;summary.zeroDropped=w.dropped;summary.zeroFailed=w.failed;
    summary.lastZeroSentUs=w.lastSentUs;summary.outputFaults=outputFaults_;
    if (lastClock_) summary.rs06=motors_.rs06Status(*lastClock_);
    return summary;
}
}
