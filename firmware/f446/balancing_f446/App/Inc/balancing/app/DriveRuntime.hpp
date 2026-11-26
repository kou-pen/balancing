#pragma once
#include "balancing/app/ApplicationObservation.hpp"

namespace balancing {
class MotorService;
class MotorCanLink;
class CanTxScheduler;

/** @brief 制御周期の起動時設定。既定値では物理モデル・駆動を有効にしない。
 * @spec{BOARD-F446-001}
 */
struct DriveControlConfig {
    StateEstimatorConfig estimator{};
    ArmTimeoutConfig arm{};
    BalanceControlConfig controller{};
    MotorCommandConfig converter{};
};

/** @brief 制御周期の実行境界。リンクする実装をビルド対象ごとに選ぶ。
 * @note 本番側は非駆動のControlCycleと明示設定済みの停止専用送信を実行。実装は静的に一つ。
 * @spec{BOARD-F446-001}
 */
namespace drive_runtime {
/** @brief 起動時の固定設定。実機側は未検証設定を返す。
 * @spec{BOARD-F446-001}
 */
DriveControlConfig configuration() noexcept;
/** @brief 新周期開始時に前周期の実行結果を無効化する。
 * @spec{BOARD-F446-001}
 */
void beginCycle() noexcept;
/** @brief 無効時刻の周期を状態判定に渡す実行経路があるか。
 * @spec{BOARD-F446-001}
 */
bool handlesInvalidTiming() noexcept;
/** @brief 制御周期を一度実行し、現在の制御結果を返す。
 * @spec{BOARD-F446-001}
 */
ControlTickResult step(ControlCycle&,RobotSequence&,RemoteControlService&,
                       ApplicationObservation&,ControlTickInput) noexcept;
/** @brief 制御後の操作量を確定する。状態・時刻は同周期のもの。
 * @spec{BOARD-F446-001}
 */
void finishCycle(RobotSequence&,RemoteControlService&,ApplicationObservation&,
                 std::uint64_t nowUs) noexcept;
/** @brief CAN開始前に停止専用設定と内部トークンの衝突を検査する。
 * @spec{BOARD-F446-001}
 */
bool validateStopOutput(const MotorObservationConfig&,bool selfDefinedCan2,
                        std::uint32_t diagnosticOrigin) noexcept;
/** @brief 起動済みCANの所有者を借用する。設定は起動時固定。
 * @spec{BOARD-F446-001}
 */
void bind(ControlCycle&,RobotSequence&,MotorService&,MotorCanLink&,
          CanTxScheduler&,CanTxScheduler&,StopOutputConfig) noexcept;
/** @brief 有限回の背景処理。
 * @spec{BOARD-F446-001}
 */
void poll() noexcept;
/** @brief 照合済みCAN送信終端の配送。
 * @spec{BOARD-F446-001}
 */
void onTxTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t atUs) noexcept;
}
/** @brief 起動前の明示設定を一度だけ構築し、CAN2が有効な場合のみ成功。
 * @spec{BOARD-F446-001}
 */
bool initializeApplicationRuntime(const Can2ObservationProfile&) noexcept;
/** @brief Applicationの制御所有者を起動済みCAN実行境界へ結合する。
 * @spec{BOARD-F446-001}
 */
void bindApplicationRuntime(MotorService&,MotorCanLink&,CanTxScheduler&,CanTxScheduler&,StopOutputConfig) noexcept;
}
