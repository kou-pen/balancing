#ifndef BALANCING_F446_CAN1_BRIDGE_H
#define BALANCING_F446_CAN1_BRIDGE_H

#include "stm32f4xx_hal.h"
#include "balancing/device/Can1Observation.h"
#include "balancing/device/Can2Observation.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAN1/2の受信フィルタ、共有時計、割込みを起動する。
 * @param[in,out] can1 CubeMXで初期化済みのCAN1ハンドル。
 * @param[in,out] can2 CubeMXで初期化済みのCAN2ハンドル。
 * @return 起動できた場合にtrue。
 * @note 両バスのモータ指令は送信せず、受信・異常を観測する段階。
 * @spec{BOARD-F446-001}
 * @spec{PROTO-RS06-001}
 */
bool balancing_f446_can_start(CAN_HandleTypeDef* can1, CAN_HandleTypeDef* can2);

/**
 * @brief CAN1 TX／RX0／SCE割込みのイベントを固定キューへ取り込む。
 * @return CAN1ポートが割込みを所有し、生成側のHAL処理をスキップする場合にtrue。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_can1_capture_irq(void);

/**
 * @brief CAN2 TX/RX0/SCEのイベントをCAN2専用キューへ取り込む。
 * @return ポートが割込みを所有していればtrue。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_can2_capture_irq(void);

/**
 * @brief 500 Hzのメイン側で取得済みイベントを観測結果へ反映する。
 * @note 割込みと同じDWT時計を定期的にサンプリングし、32 bitの周回を追跡する。
 * @spec{BOARD-F446-001}
 */
void balancing_f446_can_service(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_F446_CAN1_BRIDGE_H */
