#include "balancing_app_bridge.h"
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/app/DriveRuntime.hpp"
#include "balancing/device/ImuObservation.hpp"

#include <cstdint>
#include <optional>

/*
 * CubeMX生成main.cからはC ABIのみを公開する。C++のApplicationは
 * HALから独立した入口を保持し、初期化前や実機パラメータ未検証時には
 * いかなるモータ送信・ARM処理も開始しない。
 */
namespace balancing {

// 起動時固定の非駆動プロファイル。CAD・校正・ゲイン・上限・ARM期限の
// 未検証値を人工fixtureで埋めない。所有者の寿命はControlCycleより長い。
struct ObservationCycle {
    PowerBoardService power{};
    RemoteControlService remote;
    DualSenseService usb;
    UsbHostPort usbHost{};
    std::uint8_t usbReportLength{};
    // 実機FIFO容量・配信仕様は未確定。ゼロ容量で最新状態と欠落数だけ記録。
    DiagnosticsService diagnostics;
    Can2ServiceLink can2;
    StatusIndicator indicator{};
    StatusIndicatorPort indicatorOutput{};
    StateEstimator estimator{StateEstimatorConfig{}};
    RobotSequence sequence{ArmTimeoutConfig{}};
    BalanceControl controller{BalanceControlConfig{}};
    MotorCommand converter{MotorCommandConfig{}};
    std::optional<ControlCycle> cycle{std::in_place, estimator, sequence, controller, converter};

    explicit ObservationCycle(const Can2ObservationProfile& profile={},
                              const DriveControlConfig& config={}) noexcept
        : remote{profile.operatorConfig},usb{remote,profile.usb},
          usbReportLength{profile.usb.hid.length},
          diagnostics{profile.faultSlots,profile.faultCapacity,nullptr,0,
                      profile.communication.tokenOrigin},
          can2{power,remote,diagnostics,profile.communication},
          indicator{profile.indicator},indicatorOutput{profile.indicator.verified?profile.indicatorOutput:IndicatorOutputConfig{}},
          estimator{config.estimator},sequence{config.arm},controller{config.controller},
          converter{config.converter}
    {}

    void breakTiming() noexcept
    {
        // 観測不能区間をまたぐdtは使わない。状態機械と車輪原点は維持する。
        // このプロファイルはARM不可能。将来の稼働中停止判断には使わない。
        if (sequence.state() == RobotState::Disarmed || sequence.state() == RobotState::Fault) {
            cycle.emplace(estimator, sequence, controller, converter);
        }
    }
};

class Application final {
public:
    /*
     * 静的オブジェクトの初期値だけを確定する。通信初期化やモータ有効化は
     * 本番配線・校正条件が決まるまで追加しない。
     */
    bool init(const Can2ObservationProfile& profile={},bool requireCan2=false) noexcept
    {
        // 起動済みHostを別のApplicationへ暗黙再結合しない。
        if (control_ && control_->usbHost.summary().started) return false;
        if (control_) control_->indicatorOutput.disable();
        observation_ = {};
        previousUs_.reset();
        health_={};
        ticket_.reset();
        watchdog_=WatchdogPort{profile.watchdog};
        observation_.watchdog=watchdog_.summary();
        control_.emplace(profile,drive_runtime::configuration());
        if (requireCan2 && !control_->can2.enabled()) {
            control_.reset();
            return false;
        }
        initialized_ = true;
        return true;
    }

    void bindRuntime(MotorService& motors,MotorCanLink& link,CanTxScheduler& tx1,
                     CanTxScheduler& tx2) noexcept
    {
        if (control_) drive_runtime::bind(*control_->cycle,control_->sequence,motors,link,tx1,tx2);
    }
    DualSenseService* usbLink() noexcept
    { return control_ && control_->usb.enabled()? &control_->usb:nullptr; }
    bool startUsbHost(UsbHostConfig config,UsbHostBackend backend) noexcept
    {
        if (!usbLink() || !observationClockUs() || config.reportLength!=control_->usbReportLength)
            return false;
        const bool started=control_->usbHost.start(config,backend);
        observation_.usbHost=control_->usbHost.summary();
        return started;
    }
    bool selectUsb(BOperatorSource target,std::uint64_t now) noexcept
    { return usbLink() && usbLink()->selectSource(target,control_->sequence.state(),now); }
    Can2ServiceLink* can2Link() noexcept
    {
        return control_ && control_->can2.enabled() ? &control_->can2 : nullptr;
    }

    /*
     * 既存mainが両CANイベントを回収した直後の非駆動周期。
     * 実測時刻を採用し、TIM7の呼出し回数から2msを仮定しない。
     */
    void beginScheduledTick(std::uint32_t eventSequence) noexcept
    {
        if (!initialized_) return;
        ticket_.reset();
        if (const auto now=observationClockUs()) {
            const CycleTicket ticket{eventSequence,*now};
            if (health_.begin(ticket)) ticket_=ticket;
        } else {
            health_.abandon();
        }
        observation_.cycleHealth=health_.summary();
    }

    void runTick() noexcept
    {
        if (!initialized_) return;
        const bool processed=controlTick();
        if (ticket_) {
            // CAN回収から診断・表示まで戻った後にだけ証跡を確定する。
            if (const auto now=observationClockUs()) {
                if (const auto completion=health_.complete(*ticket_,*now,processed))
                    (void)watchdog_.refresh(*completion);
            } else {
                health_.abandon();
            }
            ticket_.reset();
        } else {
            health_.abandon();
        }
        observation_.cycleHealth=health_.summary();
        observation_.watchdog=watchdog_.summary();
    }

    bool controlTick() noexcept
    {
        if (!initialized_) {
            return false;
        }

        auto& timing_ = observation_.timing;
        ++timing_.control_ticks;
        observation_.motors.reset();
        observation_.imu = {};
        observation_.power = {};
        observation_.operatorTarget.reset();
        observation_.indicator.reset();
        observation_.cycle.reset();
        drive_runtime::beginCycle();
        timing_.sample_valid = false;
        timing_.elapsed_valid = false;
        timing_.sampled_at_us = 0;
        timing_.elapsed_us = 0;

        const auto motors = motorObservation();
        if (!motors) {
            // 採取不能時に直前の鳴動を残さない。表示異常は駆動判定から独立。
            control_->indicatorOutput.disable();
            observation_.indicatorOutput=control_->indicatorOutput.summary();
            if (can2Link() && previousUs_) control_->can2.publishTelemetry({},*previousUs_);
            // 取得不能区間をまたいだ差分は一周期の実測値として公開しない。
            previousUs_.reset();
            if (!drive_runtime::handlesInvalidTiming()) control_->breakTiming();
            return false;
        }
        const auto nowUs = motors->motors.sampledAtUs;
        const bool monotonicSample=!previousUs_ || nowUs>*previousUs_;
        if (previousUs_) {
            if (nowUs == *previousUs_) {
                control_->can2.publishTelemetry({},nowUs);
                ++timing_.duplicate_samples;
                if (!drive_runtime::handlesInvalidTiming()) return false;
            }
            if (nowUs < *previousUs_) {
                // 逆行した時計で正常な基準を巻き戻さない。
                control_->can2.publishTelemetry({},*previousUs_);
                ++timing_.clock_regressions;
                // 逆行を論理表示へも伝え、復帰時に古い点滅/鳴動を再開しない。
                observation_.indicator=control_->indicator.update(control_->sequence.state(),
                    control_->sequence.currentFault(),nowUs);
                control_->indicatorOutput.disable();
                observation_.indicatorOutput=control_->indicatorOutput.summary();
                if (!drive_runtime::handlesInvalidTiming()) return false;
            }
            if (nowUs>*previousUs_) {
                timing_.elapsed_us = nowUs - *previousUs_;
                timing_.elapsed_valid = true;
            }
        }
        // 初回は採取時刻だけ有効。次の異なる時刻が来て初めて差分を得る。
        if (!previousUs_ || nowUs>*previousUs_) previousUs_ = nowUs;
        timing_.sampled_at_us = nowUs;
        timing_.sample_valid = true;
        // 同じCAN回収時刻でIMUを評価し、その周期のコピーを固定する。
        // 受信時刻やIMUの新規性はそれぞれの入力元が所有する。
        observation_.motors = motors;
        observation_.imu = imuAttitude(nowUs);
        ControlTickInput input{};
        input.observations = {observation_.imu, motors->motors};
        observation_.power = control_->power.snapshot(nowUs);
        input.interlocks.powerReady = observation_.power.ready;
        const auto can2Input=control_->can2.takeCycleInput(nowUs,control_->sequence.state());
        input.operations=can2Input.operations;
        // CAN側の選択要求を確定した後でUSBイベントを回収する。
        // 停止と非常停止はどちらのARMより優先し、状態判断を二重化しない。
        const auto usb=control_->usb.takeEvents(nowUs);
        input.operations.disarm |= usb.disarm;
        input.operations.arm = (input.operations.arm || usb.arm) &&
            !input.operations.disarm && can2Input.faults==FaultFlags::None;
        input.observedFaults = motors->observedFaults | control_->power.takeObservedFaults(nowUs)
                               | can2Input.faults;
        // 電源・PS5・機構・校正・ゲイン・限界が未検証なので、既定の
        // interlocks=false / mechanism=Unverified / operator空を維持。
        // 初回・取得不能直後はfirstElapsed=0で推定を拒否する。
        observation_.cycle = drive_runtime::step(*control_->cycle,control_->sequence,
            control_->remote,observation_,input);
        control_->can2.completeCycle(observation_.cycle->sequence,nowUs);
        observation_.fault = control_->sequence.currentFault();
        // 現在状態で操作目標を評価するが、位置基準やARMは未接続。
        drive_runtime::finishCycle(control_->sequence,control_->remote,observation_,nowUs);
        control_->diagnostics.updateStatus({nowUs,control_->sequence.state(),observation_.fault});
        observation_.diagnostics = control_->diagnostics.summary();
        observation_.can2 = control_->can2.summary();
        publishCan2Telemetry(nowUs);
        observation_.indicator = control_->indicator.update(control_->sequence.state(), observation_.fault, nowUs);
        (void)control_->indicatorOutput.apply(*observation_.indicator);
        observation_.indicatorOutput=control_->indicatorOutput.summary();
        // 最新コピーを周期で消費してからのみ、受信側の異常を解除する。
        // 受信側のfault消費は実機FAULT解除ではない。
        const bool consumed=acknowledgeMotorObservation(motors->generation);
        // FAULTの停止処理も一周期の完了。重複/逆行/未消費では更新しない。
        return consumed && monotonicSample && (!ticket_ || nowUs>=ticket_->startedAtUs);
    }

    /*
     * メインループの非ブロッキング診断処理を置く場所。
     * I2C/USB/CAN受信を統合する場合は制御周期の締切を妨げない設計が必要。
     */
    void poll() noexcept
    {
        if (!initialized_) {
            return;
        }
        // 受信・状態機械の更新後に背景送信を1件だけ試行する。
        // USB Hostは余剰時間に一段だけ処理。制御周期側には列挙待ちを置かない。
        if (control_->usbHost.summary().started) {
            if (const auto now=observationClockUs()) {
                control_->usbHost.poll(*now);
                if (const auto event=control_->usbHost.takeConnectionEvent()) {
                    if (event->connected) (void)control_->usb.connect(event->generation,event->atUs);
                    else (void)control_->usb.disconnect(event->generation,event->atUs);
                }
                if (const auto report=control_->usbHost.takeReport())
                    (void)control_->usb.receive(report->token.generation,report->token.serial,
                        report->data.data(),report->length,report->receivedAtUs,*now);
                observation_.usbHost=control_->usbHost.summary();
            }
        }
        drive_runtime::poll();
        if (can2Link()) serviceCan2ApplicationTx();
    }

    /* 試験・診断でC ABIから観測するだけのカウンタ。
     * 実行時刻の正確性や安全動作の証明にはならない。 */
    std::uint32_t controlTickCount() const noexcept
    {
        return observation_.timing.control_ticks;
    }

    bool timingSnapshot(BF446AppTimingSnapshot* snapshot) const noexcept
    {
        if (!initialized_ || snapshot == nullptr) {
            return false;
        }
        *snapshot = observation_.timing;
        return true;
    }

    std::optional<ApplicationObservation> observation() const noexcept
    {
        return initialized_ ? std::optional<ApplicationObservation>{observation_} : std::nullopt;
    }

private:
    void publishCan2Telemetry(std::uint64_t nowUs) noexcept
    {
        if (!can2Link()) return;
        using namespace protocol;
        Can2Telemetry frames{};
        // 状態・故障は既存Sequenceから得る。未割当ビットは0のままとし、
        // 有効性の意味をここで新規定義しない。物理量は独立した正常観測だけ配信。
        if (const auto source=control_->remote.selectedSource()) {
            const auto state=static_cast<WireRobotState>(control_->sequence.state());
            const auto bits=observation_.fault
                ? static_cast<std::uint16_t>(observation_.fault->triggerFlags |
                                             observation_.fault->subsequentFlags) : std::uint16_t{0};
            frames[0]=encodeDiagnostic(RobotStatusPayload{state,
                static_cast<WireInputSource>(*source),0,bits,0});
        }
        const auto& imu=observation_.imu;
        if (imu.valid && imu.fresh) {
            frames[1]=encodeDiagnostic(AttitudeAngle{imu.pitchRad,imu.rollRad});
            frames[2]=encodeDiagnostic(AttitudeRate{imu.pitchRateRadS,imu.rollRateRadS});
            frames[3]=encodeDiagnostic(YawRate{imu.yawRateRadS});
        }
        const auto& estimated=observation_.cycle->estimate;
        if (estimated && !estimated->fixtureOnly) {
            frames[4]=encodeDiagnostic(protocol::ChassisState{estimated->chassis.positionM,
                                                               estimated->chassis.velocityMps});
            frames[6]=encodeDiagnostic(LeftLegState{estimated->legs[0].mechanism.lengthM,
                                                   estimated->legs[0].mechanism.thetaBodyRad});
            frames[7]=encodeDiagnostic(RightLegState{estimated->legs[1].mechanism.lengthM,
                                                    estimated->legs[1].mechanism.thetaBodyRad});
        }
        if (observation_.operatorTarget) {
            const auto& target=*observation_.operatorTarget;
            frames[8]=encodeDiagnostic(ControlTarget{target.speed_m_s,target.yaw_rate_rad_s});
        }
        // 車輪角速度換算、周期超過累計の取得は未接続。0で代用しない。
        control_->can2.publishTelemetry(frames,nowUs);
    }

    CycleHealthMonitor health_{};
    WatchdogPort watchdog_{};
    std::optional<CycleTicket> ticket_{};
    ApplicationObservation observation_{};
    std::optional<ObservationCycle> control_{};
    std::optional<std::uint64_t> previousUs_{};
    bool initialized_{false};
};

// 静的に一つだけ保持。malloc/newも静的初期化時のHAL利用も行わない。
Application application;


bool initializeApplicationRuntime(const Can2ObservationProfile& profile) noexcept
{
    // 一回限りの明示設定。既存の非駆動CAN2初期化とは別に成功時だけ公開する。
    if (application.observation() || profile.communication.allowMockPowerRequests ||
        (profile.faultCapacity && !profile.faultSlots)) return false;
    return application.init(profile,true);
}

void bindApplicationRuntime(MotorService& motors,MotorCanLink& link,CanTxScheduler& tx1,
                            CanTxScheduler& tx2) noexcept
{
    application.bindRuntime(motors,link,tx1,tx2);
}

bool initializeCan2Observation(const Can2ObservationProfile& profile) noexcept
{
    // C ABIの既定起動からは利用しない。模擬電源送信を実ランタイムへ持ち込まない。
    if (profile.communication.allowMockPowerRequests ||
        (profile.faultCapacity && !profile.faultSlots) || application.can2Link()) return false;
    application.init(profile);
    return application.can2Link()!=nullptr;
}

DualSenseService* applicationDualSense() noexcept { return application.usbLink(); }
bool startApplicationUsbHost(UsbHostConfig config,UsbHostBackend backend) noexcept
{ return application.startUsbHost(config,backend); }
bool selectApplicationUsbSource(BOperatorSource target,std::uint64_t now) noexcept
{ return application.selectUsb(target,now); }

Can2ServiceLink* applicationCan2Link() noexcept
{
    return application.can2Link();
}

std::optional<ApplicationObservation> applicationObservation() noexcept
{
    return application.observation();
}

} // namespace balancing

// main.cはCとしてコンパイルするので、公開関数だけCリンケージにする。
extern "C" {

/* CubeMX CからC++を初期化するブリッジ。リンク境界以外の処理を持たない。 */
void balancing_app_init(void)
{
    if (!balancing::applicationCan2Link()) balancing::application.init();
}

/* タイマ番号をCAN回収前に記録。主ループからだけ呼ぶ。 */
void balancing_app_begin_control_tick(uint32_t event_sequence)
{
    balancing::application.beginScheduledTick(event_sequence);
}

/* メインのタイマ側から呼ぶC ABI。制御ロジックはC++内に閉じる。 */
void balancing_app_control_tick(void)
{
    balancing::application.runTick();
}

/* 低優先度のポーリング入口。ISR内のブロッキング動作を増やさない。 */
void balancing_app_poll(void)
{
    balancing::application.poll();
}

/* ホスト試験でブリッジの呼出し整合を検証するための読み取り専用API。 */
uint32_t balancing_app_control_tick_count(void)
{
    return balancing::application.controlTickCount();
}

/* 主ループの診断コピー。センサ・駆動の状態判定とは分離する。 */
bool balancing_app_timing_snapshot(BF446AppTimingSnapshot* snapshot)
{
    return balancing::application.timingSnapshot(snapshot);
}

} // extern "C"
