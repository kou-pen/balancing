#ifndef BALANCING_CAN2_OBSERVATION_H
#define BALANCING_CAN2_OBSERVATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN2受信監視。分類件数は内容の正常性・車輪の鮮度を保証しない。
 *  @spec{BOARD-F446-001}
 *  @spec{PROTO-C620-001}
 */
typedef struct {
    uint32_t rx_frames;
    uint32_t left_frames;
    uint32_t right_frames;
    uint32_t rx_loss_events;
    uint32_t bus_off_events;
    uint64_t last_rx_us;
    bool bus_off;
    uint64_t sampled_at_us;
    bool sampled;
} BF446Can2Snapshot;

/**
 * @brief CAN2の最後の主ループ監視結果をコピーする。
 * @param[out] snapshot 出力先。sampled=falseは採取前。
 * @return 両バス起動済みかつ出力先が有効ならtrue。
 * @pre 主ループのみ。カウンタは周回する。
 * @spec{BOARD-F446-001}
 */
bool balancing_f446_can2_snapshot(BF446Can2Snapshot* snapshot);

#ifdef __cplusplus
}
#endif
#endif
