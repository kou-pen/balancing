#ifndef BALANCING_F446_IMU_BRIDGE_H
#define BALANCING_F446_IMU_BRIDGE_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/** @brief I2C1観測入口。既定の未検証設定では転送を開始しない。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_imu_start(I2C_HandleTypeDef* handle);
/** @brief メインループでのみ非同期I2Cを進める。
 * @spec{BOARD-F446-001}
 */
void balancing_f446_imu_poll(void);
#ifdef __cplusplus
}
#include "balancing/control/filter/ImuAttitudeSource.hpp"
#include "balancing/device/I2cPort.hpp"
namespace balancing {
struct ImuRuntimeConfig {
    I2cPortConfig port{};
    Bno055Config sensor{};
    AttitudeFilterConfig filter{};
};
/** @brief 起動時の固定設定でI2C観測を構築。静止証明は本番未接続。
 * @spec{BOARD-F446-001}
 */
bool startImuObservation(I2C_HandleTypeDef* handle,const ImuRuntimeConfig& config) noexcept;
}
#endif
#endif
