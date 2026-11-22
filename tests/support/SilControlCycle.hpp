#pragma once

#include "balancing/app/RobotSequence.hpp"
#include "balancing/control/controller/BalanceControl.hpp"
#include "balancing/control/controller/MotorCommand.hpp"
#include "balancing/control/estimator/StateEstimator.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <variant>

namespace balancing::sil {

// この型はtests/配下のみ。人工の「検証済み」入力を本番システムへ輸出しない。
// MotorService/CanPort/実モータ指令への関数は持たない。
struct TickInput {
    StateEstimatorInput observations{};
    OperatorEvents operations{};
    SequenceInterlocks interlocks{};
    Rs06ArmFeedback armFeedback{};
    FaultFlags injectedFaults{FaultFlags::None};
    BalanceReference target{};
};

struct TickResult {
    RobotSequenceResult sequence{};
    std::optional<ControlSnapshot> estimate{};
    std::optional<StateEstimateError> estimateError{};
    std::optional<BalanceControlOutput> mathematicalTorques{};
    std::optional<MotorCommandPreview> motorCommandPreview{};
    FaultFlags observedFaults{FaultFlags::None};
};

/** @brief 仮想時計で500 Hzの処理順を検証するSIL専用オーケストレータ。
 * @note CAN送信APIを持たず、数学トルクは実機で使えないfixtureOnly固定。
 * @spec{CTRL-LQR-001}
 * @spec{BEHAV-001}
 */
class SilControlCycle final {
public:
    SilControlCycle(StateEstimator& estimator, RobotSequence& sequence,
                    BalanceControl& control, MotorCommand& converter) noexcept
        : estimator_{estimator}, sequence_{sequence}, control_{control},
          converter_{converter} {}

    /** @brief 観測固定→推定→安全状態遷移→条件付き数学制御を1回実行する。
     *  @param[in] tick この周期でコピーされた入力と操作イベント。
     *  @return 推定・状態・駆動には使えない数学的トルクをまとめた結果。
     *  @spec{CTRL-LQR-001}
     */
    TickResult step(const TickInput& tick) noexcept
    {
        using std::chrono::microseconds;
        const auto nowUs = tick.observations.motors.sampledAtUs;
        auto observed = tick.injectedFaults;
        microseconds elapsed{2000}; // 初周期は仮想時計の2ms開始。実測値とは主張しない。
        if (previousUs_) {
            if (nowUs < *previousUs_) {
                elapsed = microseconds{0};
                observed = observed | FaultFlags::ClockRegression;
            } else if (nowUs == *previousUs_) {
                elapsed = microseconds{0};
                observed = observed | FaultFlags::ControlTiming;
            } else {
                const auto delta = nowUs - *previousUs_;
                // 旧C11の稼働中更新停止6ms以上という契約と同じ境界。
                if (delta >= 6000) {
                    observed = observed | FaultFlags::ControlTiming;
                }
                elapsed = delta > static_cast<std::uint64_t>(
                                         std::numeric_limits<microseconds::rep>::max())
                              ? microseconds{0}
                              : microseconds{static_cast<microseconds::rep>(delta)};
            }
        }
        // 時計逆行時は過去の検証時刻を巻き戻さず、次周期も故障を保持する。
        if (!previousUs_ || nowUs > *previousUs_) {
            previousUs_ = nowUs;
        }

        const auto estimated = estimator_.update(tick.observations, elapsed);
        TickResult outcome{};
        outcome.observedFaults = observed;
        if (const auto* state = std::get_if<ControlSnapshot>(&estimated)) {
            outcome.estimate = *state;
        } else {
            outcome.estimateError = std::get<StateEstimateError>(estimated);
        }

        RobotSequenceInput sequenceInput{};
        sequenceInput.operations = tick.operations;
        sequenceInput.rs06 = tick.armFeedback;
        sequenceInput.interlocks = tick.interlocks;
        sequenceInput.observedFaults = observed;

        // 上流が「準備済み」と主張しても、今周期の推定が欠損ならARM/走行を認めない。
        // modelVerified=trueなどは仮想試験のためだけにTickInputで明示する。
        const bool currentEstimate = outcome.estimate.has_value();
        sequenceInput.interlocks.imuReady &= currentEstimate;
        sequenceInput.interlocks.motorsReady &= currentEstimate;
        sequenceInput.interlocks.driveLimitsVerified &= converter_.hasVerifiedConfiguration();

        const RobotState before = sequence_.state();
        if (before == RobotState::Disarmed && tick.operations.arm && currentEstimate) {
            if (!control_.initialize(*outcome.estimate)) {
                sequenceInput.interlocks.modelVerified = false;
            }
        }

        // BALANCING中の制御計算異常を同周期のFAULT入力へ反映するため、
        // 状態機械を更新する前に計算候補を作り、更新後に許可があれば返す。
        // それ以外ではYaw積分や制御出力を進めない。
        std::optional<BalanceControlOutput> candidate{};
        std::optional<MotorCommandPreview> converted{};
        if (before == RobotState::Balancing && currentEstimate) {
            candidate = control_.compute(*outcome.estimate, tick.target);
            if (candidate) {
                const auto preview = converter_.evaluate(*candidate);
                if (preview.status == CommandStatus::Ready ||
                    preview.status == CommandStatus::Saturated) {
                    converted = preview;
                }
            }
            // 変換器の未検証設定・不正入力も、送信前の同周期にFAULTとして扱う。
            // ただし数学fixtureの成功は実機の駆動を許可しない。
            if (!candidate || !converted) {
                sequenceInput.observedFaults =
                    sequenceInput.observedFaults | FaultFlags::ExternalFault;
            }
        }

        outcome.sequence = sequence_.update(sequenceInput, nowUs);
        outcome.observedFaults = sequenceInput.observedFaults;
        if (outcome.sequence.desiredMotorState == DesiredMotorState::DriveAllowed) {
            // あくまで数学トルク。fixtureOnlyが偽になる変換器は存在しない。
            outcome.mathematicalTorques = candidate;
            outcome.motorCommandPreview = converted;
        } else if (outcome.sequence.state == RobotState::Disarmed ||
                   outcome.sequence.state == RobotState::Fault) {
            control_.reset();
        }
        return outcome;
    }

private:
    StateEstimator& estimator_;
    RobotSequence& sequence_;
    BalanceControl& control_;
    MotorCommand& converter_;
    std::optional<std::uint64_t> previousUs_{};
};

} // namespace balancing::sil
