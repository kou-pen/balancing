#include "DriveSil.hpp"

namespace balancing {
namespace {
// 試験バイナリだけが人工設定と実際の模擬CAN送信所有者を保持する。
// 初期化前の構築失敗では設定を破棄し、次の呼出しに持ち越さない。
std::optional<DriveSilProfile> profile;
std::optional<DriveExecution> drive;
std::optional<MotorFramePlan> framePlan;
MotorFramePreview composeFixture(const RobotSequenceResult& seq,const MotorCommandPreview& command) noexcept
{
    return framePlan->compose(seq,command);
}
std::optional<DriveSilResult> result;
}
bool initializeDriveSil(const DriveSilProfile& p) noexcept
{
    if (applicationObservation() || observationClockUs() ||
        p.communication.communication.allowMockPowerRequests ||
        p.communication.communication.tokenOrigin==0xD000u ||
        (p.communication.faultCapacity && !p.communication.faultSlots)) return false;
    profile=p;
    if (!initializeApplicationRuntime(p.communication)) {
        profile.reset();
        return false;
    }
    return true;
}
std::optional<DriveSilResult> driveSilObservation() noexcept { return result; }

namespace drive_runtime {
DriveControlConfig configuration() noexcept
{
    if (!profile) return {};
    return {profile->estimator,profile->arm,profile->controller,profile->converter};
}
void beginCycle() noexcept { result.reset(); }
bool handlesInvalidTiming() noexcept { return drive.has_value(); }

ControlTickResult step(ControlCycle& cycle,RobotSequence& sequence,RemoteControlService& remote,
                       ApplicationObservation& observation,ControlTickInput input) noexcept
{
    // SIL設定が存在しても模擬CAN所有者と結合するまでは非駆動。
    if (!drive) return cycle.step(input);
    input.interlocks=profile->interlocks;
    input.interlocks.powerReady &= observation.power.ready;
    input.interlocks.motorsReady &= observation.motors->rs06MappingVerified;
    const auto& cfg=profile->controller;
    // 一括readyでは未検証の個別ゲインを許可しない。
    input.interlocks.modelVerified &= cfg.lqr.verified && cfg.legPd.verified && cfg.yawPi.verified;
    const auto now=observation.motors->motors.sampledAtUs;
    observation.operatorTarget=remote.update(sequence.state(),now);
    input.movingTarget=observation.operatorTarget.value_or(OperatorTarget{});
    result=drive->step(cycle,observation,input);
    // 同一時刻のモード切替だけでゼロを確定。変化率処理は二度進めない。
    if (sequence.state()!=RobotState::Balancing)
        observation.operatorTarget=remote.update(sequence.state(),now);
    return result->control;
}
void finishCycle(RobotSequence& sequence,RemoteControlService& remote,
                 ApplicationObservation& observation,std::uint64_t nowUs) noexcept
{
    if (!drive) observation.operatorTarget=remote.update(sequence.state(),nowUs);
}
bool validateStopOutput(const MotorObservationConfig& c,bool,std::uint32_t) noexcept { return !c.stopOutput.enabled; }
void bind(ControlCycle&,RobotSequence& sequence,MotorService& motors,MotorCanLink& link,
          CanTxScheduler& tx1,CanTxScheduler& tx2,StopOutputConfig) noexcept
{
    if (profile && !drive) {
        framePlan.emplace(motors.rs06BusConfig().motorIds);
        drive.emplace(sequence,motors,link,tx1,tx2,profile->tx,composeFixture);
    }
}
void poll() noexcept
{
    if (drive) if (const auto now=observationClockUs()) drive->poll(*now);
}
void onTxTerminal(CanBus b,TxToken t,CanTxOutcome outcome,std::uint64_t atUs) noexcept
{
    if (drive) drive->onTxTerminal(b,t,outcome,atUs);
}
}
}
