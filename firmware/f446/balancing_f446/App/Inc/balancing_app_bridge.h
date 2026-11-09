#ifndef BALANCING_APP_BRIDGE_H
#define BALANCING_APP_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CubeMX生成main.cからC++アプリケーションを初期化する。
 * @spec{BOARD-F446-001}
 * @pre MX_GPIO_Init等の生成済み周辺初期化が完了していること。
 */
void balancing_app_init(void);

/**
 * @brief TIM7の500 Hzイベントをメインループで消費した際に呼ぶ。
 * @spec{BOARD-F446-001}
 * @warning 割り込みから直接呼び出さない。実機モータ駆動はまだ実装しない。
 */
void balancing_app_control_tick(void);

/**
 * @brief メインループの余剰時間に呼ぶ。ブロックしない。
 * @spec{BOARD-F446-001}
 */
void balancing_app_poll(void);

/**
 * @brief ホスト試験・診断向けの処理済み制御周期数。
 * @spec{BOARD-F446-001}
 */
uint32_t balancing_app_control_tick_count(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_APP_BRIDGE_H */
