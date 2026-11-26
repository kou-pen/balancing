#include "balancing/app/DriveRuntime.hpp"

namespace balancing::drive_runtime {
// 実機のモデル・ゲイン・変換・期限は未検証。試験設定を受け取る入口を持たない。
DriveControlConfig configuration() noexcept { return {}; }
void beginCycle() noexcept {}
bool handlesInvalidTiming() noexcept { return false; }

ControlTickResult step(ControlCycle& cycle,RobotSequence&,RemoteControlService&,
                       ApplicationObservation&,ControlTickInput input) noexcept
{
    // 数学・状態の非駆動観測だけ。MotorServiceやCAN送信には接続しない。
    return cycle.step(input);
}
void finishCycle(RobotSequence& sequence,RemoteControlService& remote,
                 ApplicationObservation& observation,std::uint64_t nowUs) noexcept
{
    observation.operatorTarget=remote.update(sequence.state(),nowUs);
}
// 現段階の実機経路は受信専用。起動済みCAN所有者への駆動結合を行わない。
void bind(ControlCycle&,RobotSequence&,MotorService&,MotorCanLink&,
          CanTxScheduler&,CanTxScheduler&) noexcept {}
void poll() noexcept {}
void onTxTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t) noexcept {}
}
