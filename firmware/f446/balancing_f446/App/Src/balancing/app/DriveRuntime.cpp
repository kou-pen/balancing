#include "balancing/app/DriveRuntime.hpp"
#include "balancing/app/DriveExecution.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

namespace balancing::drive_runtime {
namespace {
constexpr std::uint32_t kZeroOrigin=0xD001u; // 内部照合キー。CAN IDとは無関係。

std::optional<DriveExecution> stop;
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
    if (!stop) return cycle.step(input);
    const auto result=stop->step(cycle,observation,input).control;
    poll();
    observation.stopOutput=stop->stopSummary();
    return result;
}
void finishCycle(RobotSequence& sequence,RemoteControlService& remote,
                 ApplicationObservation& observation,std::uint64_t nowUs) noexcept
{
    observation.operatorTarget=remote.update(sequence.state(),nowUs);
}
void bind(ControlCycle&,RobotSequence& sequence,MotorService& motors,MotorCanLink& link,
          CanTxScheduler& can1,CanTxScheduler& can2,StopOutputConfig config) noexcept
{
    if (config.enabled && config.verified && motors.hasRs06Configuration() && !stop) {
        stop.emplace(sequence,motors,link,can1,can2);
    }
}
void poll() noexcept
{
    if (stop) if (const auto now=observationClockUs()) stop->poll(*now);
}
void onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,std::uint64_t atUs) noexcept
{
    if (!stop) return;
    stop->onTxTerminal(bus,token,outcome,atUs);
}
}
