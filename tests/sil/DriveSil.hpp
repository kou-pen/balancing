#pragma once
#include "balancing/app/DriveRuntime.hpp"
#include "DriveCycle.hpp"

namespace balancing {
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
/** @brief 最新のSIL周期固有結果。共通ApplicationObservationには含めない。
 * @spec{BOARD-F446-001}
 */
std::optional<DriveSilResult> driveSilObservation() noexcept;
}
