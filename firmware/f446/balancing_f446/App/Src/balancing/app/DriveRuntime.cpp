#include "balancing/app/DriveRuntime.hpp"
#include "balancing/app/MotorCanLink.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"
#include "balancing/communication/C620Codec.hpp"
#include <algorithm>
#include <limits>

namespace balancing::drive_runtime {
namespace {
constexpr std::uint32_t kZeroOrigin=0xD001u; // 内部照合キー。CAN IDとは無関係。
constexpr std::uint64_t kWheelPeriodUs=2000u;

struct StopOutput {
    MotorService& motors;
    MotorCanLink& link;
    CanTxScheduler& can1;
    CanTxScheduler& can2;
    StopOutputSummary summary{};
    std::optional<CycleTicket> lastTicket{};
    std::optional<std::uint64_t> lastGeneration{},lastClock{};
    TxToken queued{},accepted{};
    FaultFlags pending{FaultFlags::None};
    bool cycleSeen{};

    void fault(FaultFlags f) noexcept {
        pending=pending|f;
        summary.outputFaults=summary.outputFaults|f;
    }

    void service(std::uint64_t now) noexcept {
        if (!cycleSeen || (lastClock && now<*lastClock)) return;
        lastClock=now;
        // 停止を維持。Type3/Type1を生成する状態要求はこの経路には存在しない。
        motors.applySequence({RobotState::Disarmed,DesiredMotorState::StopAll,0},now);
        link.submitPending(now);
        for (unsigned i=0;i<4;++i) {
            const auto progress=can1.service(now);
            link.onScheduledTxProgress(progress,now);
            if (progress.result!=ScheduledTxResult::Accepted) break;
        }
        // C620共通ゼロの周期枠は一件だけ。古い受付済み世代はSchedulerが終端まで保持する。
        const auto progress=can2.service(now);
        if (queued.serial && progress.token==queued) {
            if (progress.result==ScheduledTxResult::Accepted) {
                accepted=queued;
                ++summary.zeroAccepted;
                queued={};
            } else if (progress.result==ScheduledTxResult::Expired ||
                       progress.result==ScheduledTxResult::Invalid) {
                ++summary.zeroDropped;
                queued={};
                if (progress.result==ScheduledTxResult::Invalid) fault(FaultFlags::ExternalFault);
            }
        }
        summary.rs06=motors.rs06Status(now);
    }

    void cycle(const ApplicationObservation& observation,std::uint64_t now) noexcept {
        // 生成済みのゼロ要求を進めることと、新しい周期世代を作ることを分ける。
        // 未回収・重複・番号欠落時は世代を作らない。時刻を捏造しない。
        const auto& health=observation.cycleHealth;
        if (!health.inProgress || !health.lastStarted || !observation.motors) return;
        const auto ticket=*health.lastStarted;
        const bool fresh=(!lastGeneration || observation.motors->generation>*lastGeneration) &&
            now>=ticket.startedAtUs && (!lastClock || now>=*lastClock);
        const bool consecutive=!lastTicket ||
            (static_cast<std::uint32_t>(ticket.eventSequence-lastTicket->eventSequence)==1u &&
             ticket.startedAtUs>lastTicket->startedAtUs);
        if (!lastTicket || ticket.startedAtUs>lastTicket->startedAtUs) lastTicket=ticket;
        if (!fresh) return;
        lastGeneration=observation.motors->generation;
        if (!consecutive) return;
        cycleSeen=true;
        if (queued.serial) { ++summary.zeroDropped;queued={}; }
        // 期限は既存2msの周期予約枠。実機送達期限を新設しない。
        if (summary.zeroQueued==std::numeric_limits<std::uint64_t>::max() ||
            now>std::numeric_limits<std::uint64_t>::max()-kWheelPeriodUs) {
            fault(FaultFlags::ExternalFault);return;
        }
        const auto zero=C620Codec::encodeCurrent({},20.0f);
        if (!zero) { fault(FaultFlags::ExternalFault);return; }
        const auto generation=++summary.zeroQueued;
        PeriodicTxBatch batch{generation,now+kWheelPeriodUs,{}};
        const TxToken token{generation,kZeroOrigin};
        batch.requests[0]=PeriodicTxRequest{*zero,token};
        if (can2.replacePeriodic(batch,now)==PeriodicBatchResult::Queued) queued=token;
        else { ++summary.zeroDropped;fault(FaultFlags::ExternalFault); }
    }
};
std::optional<StopOutput> stop;
}

DriveControlConfig configuration() noexcept { return {}; }
void beginCycle() noexcept {}
bool handlesInvalidTiming() noexcept { return false; }

bool validateStopOutput(const MotorObservationConfig& config,bool selfDefinedCan2,
                        std::uint32_t diagnosticOrigin) noexcept
{
    if (!config.stopOutput.enabled) return true;
    if (!config.stopOutput.verified || !config.rs06MappingVerified ||
        !selfDefinedCan2 || !diagnosticOrigin || diagnosticOrigin==kZeroOrigin || stop) return false;
    // ID1/2の車輪割当と他チャネル未使用はverifiedの対象。RS06は同一バス内で重複不可。
    auto ids=config.rs06.motorIds;
    std::sort(ids.begin(),ids.end());
    return ids.front()!=0 && std::adjacent_find(ids.begin(),ids.end())==ids.end();
}

ControlTickResult step(ControlCycle& cycle,RobotSequence&,RemoteControlService&,
                       ApplicationObservation& observation,ControlTickInput input) noexcept
{
    if (stop) {
        // ARM入力があっても送信側の許可にはならない。実機モデルは空のまま。
        input.interlocks.driveLimitsVerified=false;
        input.observedFaults=input.observedFaults|stop->pending;
        stop->pending=FaultFlags::None;
        stop->cycle(observation,input.observations.motors.sampledAtUs);
    }
    const auto result=cycle.step(input);
    if (stop) {
        poll();
        observation.stopOutput=stop->summary;
    }
    return result;
}
void finishCycle(RobotSequence& sequence,RemoteControlService& remote,
                 ApplicationObservation& observation,std::uint64_t nowUs) noexcept
{
    observation.operatorTarget=remote.update(sequence.state(),nowUs);
}
void bind(ControlCycle&,RobotSequence&,MotorService& motors,MotorCanLink& link,
          CanTxScheduler& can1,CanTxScheduler& can2,StopOutputConfig config) noexcept
{
    if (config.enabled && config.verified && motors.hasRs06Configuration() && !stop) {
        stop.emplace(StopOutput{motors,link,can1,can2});
        stop->summary.enabled=true;
    }
}
void poll() noexcept
{
    if (stop) if (const auto now=observationClockUs()) stop->service(*now);
}
void onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,std::uint64_t atUs) noexcept
{
    // Routerで受付照合した終端のみ。別バス・別発行元・旧世代を数えない。
    if (!stop || bus!=CanBus::Can2 || !token.serial || !(token==stop->accepted)) return;
    stop->accepted={};
    if (outcome==CanTxOutcome::Sent) {
        ++stop->summary.zeroSent;
        stop->summary.lastZeroSentUs=atUs;
    } else {
        ++stop->summary.zeroFailed;
        if (outcome==CanTxOutcome::TransmitError) stop->fault(FaultFlags::ExternalFault);
    }
}
}
