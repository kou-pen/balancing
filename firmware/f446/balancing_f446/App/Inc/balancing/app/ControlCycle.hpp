#pragma once

#include "balancing/app/RobotSequence.hpp"
#include "balancing/control/controller/BalanceControl.hpp"
#include "balancing/control/controller/MotorCommand.hpp"
#include "balancing/control/estimator/StateEstimator.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace balancing {

/** @brief 主ループが一周期の同じ単調時刻で固定した6軸・IMU・保護条件。
 *  @note 現物で確認されていないinterlocksをtrueにしてはならない。
 *  @spec{BOARD-F446-001}
 *  @spec{BEHAV-001}
 */
struct ControlTickInput {
    StateEstimatorInput observations{};
    OperatorEvents operations{};
    SequenceInterlocks interlocks{};
    Rs06ArmFeedback armFeedback{};
    FaultFlags observedFaults{FaultFlags::None};
    BalanceReference target{};
    // 初回のみ使用。TIM7の単なるカウントから実測時刻を捏造しない。
    std::chrono::microseconds firstElapsed{0};
};

/** @brief 状態機械を確定した後の周期観測結果。ハードウェアの指令ではない。
 *  @spec{CTRL-EST-001}
 *  @spec{BEHAV-001}
 */
struct ControlTickResult {
    RobotSequenceResult sequence{};
    std::optional<ControlSnapshot> estimate{};
    std::optional<StateEstimateError> estimateError{};
    std::optional<BalanceControlOutput> mathematicalTorques{};
    std::optional<MotorCommandPreview> motorCommandPreview{};
    FaultFlags observedFaults{FaultFlags::None};
};

/** @brief F446制御主ループのHAL非依存な500 Hz処理順序を一箇所に定義する。
 *  @note 通常時の観測固定→推定→ARM/FAULT→制御→軸別換算の数値処理を実行。
 *  @warning 本クラスはMotorService、CAN、HALや送信APIを持たない。
 *           人工fixtureで計算できても実機駆動許可にはならない。
 *  @spec{BOARD-F446-001}
 *  @spec{CTRL-LQR-001}
 */
class ControlCycle final {
public:
    /** @brief 長寿命の制御・状態機械を参照し、所有権は移動しない。
     *  @param[in,out] estimator IMU・6軸から周期状態を推定する唯一の保持者。
     *  @param[in,out] sequence ARM/FAULTの唯一の決定者。
     *  @param[in,out] control Yaw積分とARM時脚長目標の保持者。
     *  @param[in,out] converter 6軸の起動時固定換算設定。
     *  @spec{BOARD-F446-001}
     */
    ControlCycle(StateEstimator& estimator, RobotSequence& sequence,
                 BalanceControl& control, MotorCommand& converter) noexcept;

    /** @brief 1回だけ周期を実行し、安全結果確定後に数値プレビューを返す。
     *  @param[in] input 上流が単調時計で採取し今周期に固定した観測・状態。
     *  @return 欠損周期では部分的な駆動プレビューを返さない。
     *  @pre 主ループからのみ呼び、各周期に新しい観測時刻を与える。
     *  @warning 本結果を実機の非ゼロCANフレームへ変換して送ってはならない。
     *  @spec{BOARD-F446-001}
     */
    ControlTickResult step(const ControlTickInput& input) noexcept;

private:
    StateEstimator& estimator_;
    RobotSequence& sequence_;
    BalanceControl& control_;
    MotorCommand& converter_;
    std::optional<std::uint64_t> previousUs_{};
};

} // namespace balancing
