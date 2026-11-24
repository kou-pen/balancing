#ifndef BALANCING_APP_BRIDGE_H
#define BALANCING_APP_BRIDGE_H

#include <stdbool.h>
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

/** @brief 非駆動の主ループ周期診断。単調時計のµs値。
 *  @note sample_valid=falseでは時刻を、elapsed_valid=falseでは差分を使用しない。
 *  @spec{BOARD-F446-001}
 */
typedef struct {
    uint32_t control_ticks;
    uint64_t sampled_at_us;
    uint64_t elapsed_us;
    uint32_t duplicate_samples;
    uint32_t clock_regressions;
    bool sample_valid;
    bool elapsed_valid;
} BF446AppTimingSnapshot;

/**
 * @brief 最後のApplication周期診断を読み取る。
 * @param[out] snapshot 出力先。
 * @return Application初期化済みかつ出力先が有効ならtrue。
 * @pre 主ループのみ。診断カウンタは周回する。
 * @warning 監視値は駆動許可やFAULT判定ではない。
 * @spec{BOARD-F446-001}
 */
bool balancing_app_timing_snapshot(BF446AppTimingSnapshot* snapshot);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_APP_BRIDGE_H */
