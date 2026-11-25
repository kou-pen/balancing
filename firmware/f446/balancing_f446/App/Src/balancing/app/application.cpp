#include "balancing_app_bridge.h"
#include "balancing/app/ApplicationObservation.hpp"
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
    RemoteControlService remote{};
    StateEstimator estimator{StateEstimatorConfig{}};
    RobotSequence sequence{ArmTimeoutConfig{}};
    BalanceControl controller{BalanceControlConfig{}};
    MotorCommand converter{MotorCommandConfig{}};
    std::optional<ControlCycle> cycle{std::in_place, estimator, sequence, controller, converter};

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
    void init() noexcept
    {
        observation_ = {};
        previousUs_.reset();
        control_.emplace();
        initialized_ = true;
    }

    /*
     * 既存mainが両CANイベントを回収した直後の非駆動周期。
     * 実測時刻を採用し、TIM7の呼出し回数から2msを仮定しない。
     */
    void controlTick() noexcept
    {
        if (!initialized_) {
            return;
        }

        auto& timing_ = observation_.timing;
        ++timing_.control_ticks;
        observation_.motors.reset();
        observation_.imu = {};
        observation_.power = {};
        observation_.operatorTarget.reset();
        observation_.cycle.reset();
        timing_.sample_valid = false;
        timing_.elapsed_valid = false;
        timing_.sampled_at_us = 0;
        timing_.elapsed_us = 0;

        const auto motors = motorObservation();
        if (!motors) {
            // 取得不能区間をまたいだ差分は一周期の実測値として公開しない。
            previousUs_.reset();
            control_->breakTiming();
            return;
        }
        const auto nowUs = motors->motors.sampledAtUs;
        if (previousUs_) {
            if (nowUs == *previousUs_) {
                ++timing_.duplicate_samples;
                return;
            }
            if (nowUs < *previousUs_) {
                // 逆行した時計で正常な基準を巻き戻さない。
                ++timing_.clock_regressions;
                return;
            }
            timing_.elapsed_us = nowUs - *previousUs_;
            timing_.elapsed_valid = true;
        }
        // 初回は採取時刻だけ有効。次の異なる時刻が来て初めて差分を得る。
        previousUs_ = nowUs;
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
        input.observedFaults = motors->observedFaults | control_->power.takeObservedFaults(nowUs);
        // 電源・PS5・機構・校正・ゲイン・限界が未検証なので、既定の
        // interlocks=false / mechanism=Unverified / operator空を維持。
        // 初回・取得不能直後はfirstElapsed=0で推定を拒否する。
        observation_.cycle = control_->cycle->step(input);
        observation_.fault = control_->sequence.currentFault();
        // 現在状態で操作目標を評価するが、位置基準やARMは未接続。
        observation_.operatorTarget = control_->remote.update(control_->sequence.state(), nowUs);
        // 最新コピーを周期で消費してからのみ、受信側の異常を解除する。
        // 受信側のfault消費は実機FAULT解除ではない。
        (void)acknowledgeMotorObservation(motors->generation);
        // MotorService::applySequence、MotorFramePlan、送信APIは呼ばない。
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
        // TODO(T-F446-05): 非ブロッキング通信・診断処理を置く。
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
    ApplicationObservation observation_{};
    std::optional<ObservationCycle> control_{};
    std::optional<std::uint64_t> previousUs_{};
    bool initialized_{false};
};

// 静的に一つだけ保持。malloc/newも静的初期化時のHAL利用も行わない。
Application application;

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
    balancing::application.init();
}

/* メインのタイマ側から呼ぶC ABI。制御ロジックはC++内に閉じる。 */
void balancing_app_control_tick(void)
{
    balancing::application.controlTick();
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
