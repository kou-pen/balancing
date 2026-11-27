#include "balancing/app/DriveRuntime.hpp"
#include "balancing/app/MotorCanLink.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

namespace balancing::drive_runtime {
namespace {
constexpr std::uint32_t kZeroOrigin=0xD001u; // 内部照合キー。CAN IDとは無関係。

struct StopOutput {
    StopOutput(MotorService& m,MotorCanLink& l,CanTxScheduler& a,CanTxScheduler& b) noexcept
        :motors{m},link{l},tx{a,b,kZeroOrigin} {}
    MotorService& motors;
    MotorCanLink& link;
    MotorPeriodicTx tx;
    StopOutputSummary summary{};
    std::optional<CycleTicket> lastTicket{};
    std::optional<std::uint64_t> lastGeneration{},lastClock{};
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
            const auto progress=tx.service(CanBus::Can1,link,now);
            if (progress.result!=ScheduledTxResult::Accepted) break;
        }
        (void)tx.service(CanBus::Can2,link,now);
        if (tx.takeFault()) fault(FaultFlags::ExternalFault);
        const auto& w=tx.summary()[4];
        summary.zeroQueued=w.queued;summary.zeroAccepted=w.accepted;
        summary.zeroSent=w.sent;summary.zeroDropped=w.dropped;summary.zeroFailed=w.failed;
        summary.lastZeroSentUs=w.lastSentUs;
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
        if (!tx.queueZero(now)) fault(FaultFlags::ExternalFault);
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
    return MotorService::validRs06Ids(config.rs06.motorIds);
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
        stop.emplace(motors,link,can1,can2);
        stop->summary.enabled=true;
    }
}
void poll() noexcept
{
    if (stop) if (const auto now=observationClockUs()) stop->service(*now);
}
void onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,std::uint64_t atUs) noexcept
{
    if (!stop) return;
    stop->tx.onTxTerminal(bus,token,outcome,atUs);
    if (stop->tx.takeFault()) stop->fault(FaultFlags::ExternalFault);
}
}
