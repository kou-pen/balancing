#ifndef BALANCING_CAN1_OBSERVATION_H
#define BALANCING_CAN1_OBSERVATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN1の主ループ側受信監視。時刻は共通DWT時計のµs。
 *  @spec{BOARD-F446-001}
 */
typedef struct {
    uint32_t rx_frames;
    uint32_t rx_loss_events;
    uint32_t bus_off_events;
    uint64_t last_rx_us;    /* 最後のRXイベント取得時刻 */
    bool bus_off;
    uint64_t sampled_at_us; /* イベント回収後の評価時刻 */
    bool sampled;           /* 最初のservice完了前はfalse */
} BF446Can1Snapshot;

/**
 * @brief CAN1ランタイムの最後の主ループ監視結果をコピーする。
 * @param[out] snapshot 出力先。sampled=falseなら採取前。
 * @return 起動済みかつ出力先が有効ならtrue。
 * @pre 主ループのみ。カウンタの型幅を超えると周回する。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_can1_snapshot(BF446Can1Snapshot* snapshot);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_CAN1_OBSERVATION_H */
