#pragma once
#include "balancing_app_bridge.h"
#include "balancing/app/ControlCycle.hpp"
#include "balancing/app/PowerBoardService.hpp"
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
    std::optional<ControlTickResult> cycle{};
    std::optional<FaultRecord> fault{};
};

/** @brief 主ループで最後の周期診断を取得。初期化前は空。
 * @warning 数学プレビューも駆動許可も実機送信へ接続しない。
 * @spec{BOARD-F446-001}
 */
std::optional<ApplicationObservation> applicationObservation() noexcept;
}
