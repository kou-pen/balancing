#ifndef BALANCING_F446_CAN1_BRIDGE_H
#define BALANCING_F446_CAN1_BRIDGE_H

#include "stm32f4xx_hal.h"
#include "balancing/device/Can1Observation.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAN1のRS06受信フィルタ、イベント取得時計、割込みを起動する。
 * @param[in,out] handle CubeMXで初期化済みのCAN1ハンドル。
 * @return 起動できた場合にtrue。
 * @note CAN1モータ指令は送信せず、受信・異常を観測する段階。
 * @spec{BOARD-F446-001}
 * @spec{PROTO-RS06-001}
 */
bool balancing_f446_can1_start(CAN_HandleTypeDef* handle);

/**
 * @brief CAN1 TX／RX0／SCE割込みのイベントを固定キューへ取り込む。
 * @return CAN1ポートが割込みを所有し、生成側のHAL処理をスキップする場合にtrue。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_can1_capture_irq(void);

/**
 * @brief 500 Hzのメイン側で取得済みイベントを観測結果へ反映する。
 * @note 割込みと同じDWT時計を定期的にサンプリングし、32 bitの周回を追跡する。
 * @spec{BOARD-F446-001}
 */
void balancing_f446_can1_service(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_F446_CAN1_BRIDGE_H */
