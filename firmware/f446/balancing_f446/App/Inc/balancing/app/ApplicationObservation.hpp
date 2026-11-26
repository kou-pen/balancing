#pragma once
#include "balancing_app_bridge.h"
#include "balancing/app/ControlCycle.hpp"
#include "balancing/app/PowerBoardService.hpp"
#include "balancing/app/Can2ServiceLink.hpp"
#include "balancing/app/RemoteControlService.hpp"
#include "balancing/app/DualSenseService.hpp"
#include "balancing/app/DiagnosticsService.hpp"
#include "balancing/app/StatusIndicator.hpp"
#include "balancing/device/StatusIndicatorPort.hpp"
#include "balancing/device/UsbHostPort.hpp"
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
    IndicatorOutputSummary indicatorOutput{};
    UsbHostSummary usbHost{};
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
    DualSenseConfig usb{};
    IndicatorConfig indicator{};
    IndicatorOutputConfig indicatorOutput{};
};
/** @brief 起動前にだけ非駆動試験設定を注入する。実機電源要求は常に無効。
 * @spec{BOARD-F446-001}
 */
bool initializeCan2Observation(const Can2ObservationProfile&) noexcept;
/** @brief 主ループ専用。無効設定・未初期化では空。
 * @spec{BOARD-F446-001}
 */
Can2ServiceLink* applicationCan2Link() noexcept;
/** @brief 主ループのUSB完了通知境界。未設定なら空。実USB列挙は別工程。
 * @spec{PROTO-PS5-001}
 */
DualSenseService* applicationDualSense() noexcept;
/** @brief 検証済みUSB設定と非ブロッキング下位Hostを明示接続。既定起動は無効。
 * @spec{PROTO-PS5-001}
 */
bool startApplicationUsbHost(UsbHostConfig,UsbHostBackend) noexcept;
/** @brief USB側の明示操作元選択。状態判断はApplicationの現在状態を用いる。
 * @spec{PROTO-PS5-001}
 */
bool selectApplicationUsbSource(BOperatorSource target,std::uint64_t nowUs) noexcept;
/** @brief CANランタイムの背景送信を一度進める。既定起動では何もしない。
 * @spec{BOARD-F446-001}
 */
void serviceCan2ApplicationTx() noexcept;
}
