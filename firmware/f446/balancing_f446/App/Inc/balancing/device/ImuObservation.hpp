#pragma once
#include "balancing/control/estimator/StateEstimator.hpp"
namespace balancing {
/** @brief 主ループで最後の姿勢を指定周期時刻で評価。未起動は無効。
 * @spec{CTRL-EST-001}
 */
ImuAttitude imuAttitude(std::uint64_t nowUs) noexcept;
}
