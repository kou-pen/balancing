#pragma once
#include "balancing_app_bridge.h"
#include "balancing/app/ControlCycle.hpp"
#include "balancing/app/DriveCycle.hpp"
#include "balancing/app/PowerBoardService.hpp"
#include "balancing/app/Can2ServiceLink.hpp"
#include "balancing/app/RemoteControlService.hpp"
#include "balancing/app/DiagnosticsService.hpp"
#include "balancing/app/StatusIndicator.hpp"
#include "balancing/device/MotorObservation.hpp"

namespace balancing {
/** @brief 非駆動Applicationの一周期分の観測・推定結果。古い結果を再公開しない。
 * @spec{BOARD-F446-001}
 * @spec{CTRL-EST-001}
 */
struct ApplicationObservation {
    BF446AppTimingSnapshot timing{};
    std::optional<MotorObservationSnapshot> motors{};
    ImuAttitude imu{};
    PowerBoardSnapshot power{};
    std::optional<BOperatorTarget> operatorTarget{};
    std::optional<ControlTickResult> cycle{};
    std::optional<FaultRecord> fault{};
    DiagnosticSummary diagnostics{};
    Can2ServiceSummary can2{};
    std::optional<IndicatorState> indicator{};
#if defined(BALANCING_HOST_SIL) && !defined(__arm__) && !defined(__thumb__)
    std::optional<DriveSilResult> driveSil{};
#endif
};

/** @brief 主ループで最後の周期診断を取得。初期化前は空。
 * @warning 数学プレビューも駆動許可も実機送信へ接続しない。
 * @spec{BOARD-F446-001}
 */
std::optional<ApplicationObservation> applicationObservation() noexcept;

/** @brief 非駆動のCAN2結合試験設定。診断領域はApplicationより長寿命。
 * @spec{BOARD-F446-001}
 */
struct Can2ObservationProfile {
    Can2ServiceConfig communication{};
    BOperatorInputConfig operatorConfig{};
    DiagnosticSlot* faultSlots{};
    std::size_t faultCapacity{};
};
/** @brief 起動前にだけ非駆動試験設定を注入する。実機電源要求は常に無効。
 * @spec{BOARD-F446-001}
 */
bool initializeCan2Observation(const Can2ObservationProfile&) noexcept;
/** @brief 主ループ専用。無効設定・未初期化では空。
 * @spec{BOARD-F446-001}
 */
Can2ServiceLink* applicationCan2Link() noexcept;
/** @brief CANランタイムの背景送信を一度進める。既定起動では何もしない。
 * @spec{BOARD-F446-001}
 */
void serviceCan2ApplicationTx() noexcept;
#if defined(BALANCING_HOST_SIL) && !defined(__arm__) && !defined(__thumb__)
/** @brief 人工の制御・通信設定。ホスト試験専用で実機バイナリには存在しない。
 * @spec{BOARD-F446-001}
 */
struct DriveSilProfile {
    Can2ObservationProfile communication{};
    StateEstimatorConfig estimator{};
    BalanceControlConfig controller{};
    MotorCommandConfig converter{};
    ArmTimeoutConfig arm{};
    SequenceInterlocks interlocks{};
    DriveSilConfig tx{};
};
/** @brief 模擬CAN起動前の一回限りのSIL設定。
 * @spec{BOARD-F446-001}
 */
bool initializeDriveSil(const DriveSilProfile&) noexcept;
/** @brief 起動済み模擬ランタイムの所有者をApplicationへ貸す。
 * @spec{BOARD-F446-001}
 */
bool bindDriveSilRuntime(MotorService&,MotorCanLink&,CanTxScheduler&,CanTxScheduler&) noexcept;
/** @brief 模擬CANの照合済み終端を周期統合に配送。
 * @spec{BOARD-F446-001}
 */
void driveSilTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t atUs) noexcept;
#endif
}
