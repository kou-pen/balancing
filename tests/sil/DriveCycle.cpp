#include "DriveCycle.hpp"

namespace balancing {
namespace {
constexpr std::uint32_t origin=0xD000u; // MotorCanLink(0x100|軸ID)と診断とは別
constexpr std::uint64_t rsPeriod=4000;

}
DriveCycle::DriveCycle(ControlCycle& control,RobotSequence& sequence,MotorService& motors,
                       MotorCanLink& link,CanTxScheduler& can1,CanTxScheduler& can2,
                       DriveSilConfig config) noexcept
    :control_{control},sequence_{sequence},motors_{motors},link_{link},tx_{can1,can2,origin},
     frames_{motors.rs06BusConfig().motorIds},config_{config}
{
    configured_=motors_.hasRs06Configuration() && config_.rs06TxTimeoutUs &&
        config_.c620TxTimeoutUs;
}
FaultFlags DriveCycle::txFaults(std::uint64_t nowUs) const noexcept
{
    return tx_.deadlineFault(nowUs)?FaultFlags::ExternalFault:FaultFlags::None;
}
void DriveCycle::withdraw() noexcept
{
    tx_.withdraw();
    activeSince_.reset();
    lastRsDue_.reset();
    active_=false;
}
bool DriveCycle::queue(CanBus bus,const MotorFramePreview& preview,std::uint64_t nowUs) noexcept
{
    if (bus==CanBus::Can1) return tx_.queue(bus,preview.can1Type1,preview.can1Count,nowUs);
    if (preview.status==MotorFramePreview::Status::ZeroCurrentOnly) return tx_.queueZero(nowUs);
    return tx_.queue(bus,{preview.can2Current},1,nowUs);
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
        tx_.beginActive(now,config_.rs06TxTimeoutUs,config_.c620TxTimeoutUs);
    }
    if (!allowed && activeSince_) withdraw();
    if (hasCommand) {
        const auto plan=frames_.compose(decision,*result.control.motorCommandPreview);
        if (plan.status==MotorFramePreview::Status::Ready) {
            // ARMING/遷移周期のゼロ電流終端は通常指令の証跡にしない。
            tx_.markDriving();
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
    for (unsigned i=0;i<4;++i) {
        const auto progress=tx_.service(bus,link_,nowUs);
        if (tx_.takeFault()) pending_=pending_|FaultFlags::ExternalFault;
        if (progress.result!=ScheduledTxResult::Accepted &&
            progress.result!=ScheduledTxResult::Expired &&
            progress.result!=ScheduledTxResult::Invalid) break;
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
    tx_.onTxTerminal(bus,token,outcome,atUs);
    // SILでは非駆動のゼロ終端異常を駆動期限の故障として扱わない。
    if (tx_.takeFault() && active_) pending_=pending_|FaultFlags::ExternalFault;
}
}
