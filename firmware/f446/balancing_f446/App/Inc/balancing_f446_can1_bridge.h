#ifndef BALANCING_F446_CAN1_BRIDGE_H
#define BALANCING_F446_CAN1_BRIDGE_H

#include "stm32f4xx_hal.h"

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

#include "balancing/device/MotorObservation.hpp"

namespace balancing {
/** @brief 検証済み観測設定を固定して両CANを起動する。送信は開始しない。
 *  @spec{BOARD-F446-001}
 */
bool startCanObservation(CAN_HandleTypeDef* can1, CAN_HandleTypeDef* can2,
                         const MotorObservationConfig& config, bool selfDefinedCan2=false) noexcept;
}
#endif

#endif /* BALANCING_F446_CAN1_BRIDGE_H */
