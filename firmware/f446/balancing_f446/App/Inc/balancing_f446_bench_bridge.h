#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/** @brief BenchDebug専用の起動。未検証設定では拒否し、CANを開始しない。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_bench_start(CAN_HandleTypeDef* can1,CAN_HandleTypeDef* can2);
/** @brief TIM7の周期イベントに応じて停止確認だけを進める。
 * @spec{BOARD-F446-001}
 */
void balancing_f446_bench_service(uint32_t sequence);
/** @brief 選択バスのIRQを一度だけ採取する。1=CAN1、2=CAN2。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_bench_capture_irq(uint8_t bus);
#ifdef __cplusplus
}
#endif
