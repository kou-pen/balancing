#include "balancing/device/balancing_f446_motor_bench_port.h"

#include <string.h>

/*
 * HALとBMotorBenchの間だけを扱う。受信割込みとメインループの共有データは
 * PRIMASKで短時間保護する。タイマ割込みの中で呼ばず、メインループから呼ぶ。
 * CANフィルタ・ピン・クロック等のボード固有設定を勝手に追加しない。
 */

/* CANフィードバックの生存判定はI/F境界でのみ行い、ベンチは結果を消費する。
 * 型ごとの閾値は既存仕様値で変更しない。RS06有効化/パルスの期限とは別。 */
#define BENCH_RS_RX_TIMEOUT_US UINT64_C(30000)
#define BENCH_C620_RX_TIMEOUT_US UINT64_C(20000)

/* 最後の新規受信からの単調経過だけを判定する。未来時刻、未受信は無効。
 * 現行仕様と同じく閾値ちょうどは有効、超過後に無効とする。 */
static bool rx_alive(bool seen, uint64_t last_us, uint64_t now_us,
                     uint64_t timeout_us)
{
    return seen && now_us >= last_us && now_us - last_us <= timeout_us;
}

/*
 * 試運転の共有状態をRX/TX割込みと主ループで更新するため、PRIMASKを保存する。
 * 既に外側で割込み禁止ならleave()で勝手に有効化しない。
 * HALの送信受付もこの区間で呼ばれるので、実機での割込み遅延は未計測。
 */
static uint32_t enter_critical(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

/* enter_critical()より前に割込み許可だった場合だけ元に戻す。
 * ハードウェアの共有領域を全て網羅するロックの保証ではない。 */
static void leave_critical(uint32_t previous)
{
    if (previous == 0u) {
        __enable_irq();
    }
}

/*
 * C11のフレームをHAL CAN送信ヘッダーへ変換する薄い境界。
 * IDE/DLC/IDの対応以外のプロトコル判断はここに置かない。
 * HAL_OKはメールボックスへの「受付」であり実線への送信完了ではない。
 */
static bool tx_frame(CAN_HandleTypeDef *can, const BMotorCanFrame *frame,
                     uint32_t *mailbox_mask)
{
    CAN_TxHeaderTypeDef header = {0};
    header.IDE = frame->extended ? CAN_ID_EXT : CAN_ID_STD;
    header.StdId = frame->extended ? 0u : frame->id;
    header.ExtId = frame->extended ? frame->id : 0u;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->dlc;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(can, &header, (uint8_t *)frame->data,
                                 mailbox_mask) == HAL_OK;
}

/* HALのビットマスクはメールボックス番号ではない。
 * 送信完了IRQで同じスロットを特定するため、0..2の添字へ変換する。 */
static int mailbox_index(uint32_t mailbox_mask)
{
    if (mailbox_mask == CAN_TX_MAILBOX0) return 0;
    if (mailbox_mask == CAN_TX_MAILBOX1) return 1;
    if (mailbox_mask == CAN_TX_MAILBOX2) return 2;
    return -1;
}

/*
 * これは本番4軸バランシングではなく、低出力の単軸ベンチ専用アダプタ。
 * CAN1とCAN2のハンドル混同を拒否し、受信時刻・送信状態をゼロ初期化。
 * 初期化だけでモータの有効化やCANバス起動は行わない。
 */
bool b_f446_motor_port_init(BF446MotorBenchPort *port,
                             CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2,
                             const BMotorBenchConfig *config)
{
    if (port == NULL || can1 == NULL || can2 == NULL ||
        can1 == can2 || config == NULL) {
        return false;
    }
    memset(port, 0, sizeof(*port));
    port->rs_can = can1;
    port->wheel_can = can2;
    return b_bench_init(&port->bench, config);
}

/*
 * メインループ側の2ms契機でC620 0x200の最新試運転要求を送信する。
 * RS06はdividerで4msに間引く。ハードウェアのTIM7割込みだけで
 * 実制御周期が成立したと扱わず、実CAN遅延は別途計測が必要。
 */
void b_f446_motor_port_tick_2ms(BF446MotorBenchPort *port, uint64_t now_us)
{
    if (port == NULL) return;
    const uint32_t mask = enter_critical();
    BMotorCanFrame wheel;
    /* 既に0x200が受付済みなら次の要求を重ねない。
     * ハードウェアの送信完了IRQでpendingを解放する設計。 */
    const bool wheel_alive = rx_alive(port->wheel_rx_valid,
                                       port->wheel_last_rx_us, now_us,
                                       BENCH_C620_RX_TIMEOUT_US);
    const bool rs_alive = rx_alive(port->rs_rx_valid,
                                    port->rs_last_rx_us, now_us,
                                    BENCH_RS_RX_TIMEOUT_US);
    if (HAL_CAN_GetTxMailboxesFreeLevel(port->wheel_can) > 0u &&
        !port->wheel_tx_pending &&
        b_bench_c620_frame(&port->bench, now_us, wheel_alive, &wheel)) {
        uint32_t mailbox_mask = 0u;
        if (tx_frame(port->wheel_can, &wheel, &mailbox_mask)) {
            port->wheel_tx_pending = true;
        }
    }
    /* 2msティック2回につきRS06を1回だけ確認する。
     * 単軸ベンチのType3/4/トルク要求であり、本番4軸送信とは別経路。 */
    port->divider ^= 1u;
    if (port->divider == 0u &&
        HAL_CAN_GetTxMailboxesFreeLevel(port->rs_can) > 0u) {
        bool any_pending = false;
        for (int i = 0; i < 3; ++i) any_pending |= port->rs_mailbox_pending[i];
        if (!any_pending) {
            BMotorCanFrame rs;
            if (b_bench_rs06_frame(&port->bench, now_us, rs_alive, &rs)) {
                uint32_t mailbox_mask = 0u;
                if (tx_frame(port->rs_can, &rs, &mailbox_mask)) {
                    const int idx = mailbox_index(mailbox_mask);
                    if (idx >= 0) {
                        port->rs_mailbox_frame[idx] = rs;
                        port->rs_mailbox_pending[idx] = true;
                    }
                }
            }
        }
    }
    leave_critical(mask);
}

/*
 * FIFO0の全フレームを取り出し、CAN1/2に応じたデコーダへ渡す。
 * RTRやDLC不一致は無視して状態機械へ誤った鮮度を渡さない。
 * 受信割込みと制御側が共有する状態の直列化はHAL統合時に再確認する。
 */
void b_f446_motor_port_on_rx0(BF446MotorBenchPort *port,
                               CAN_HandleTypeDef *hcan, uint64_t now_us)
{
    if (port == NULL || hcan == NULL ||
        (hcan != port->rs_can && hcan != port->wheel_can)) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u) {
        CAN_RxHeaderTypeDef header = {0};
        BMotorCanFrame frame = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header,
                                 frame.data) != HAL_OK) break;
        if (header.RTR != CAN_RTR_DATA || header.DLC != 8u) continue;
        frame.dlc = (uint8_t)header.DLC;
        frame.extended = (header.IDE == CAN_ID_EXT);
        /* 標準と拡張IDは別の名前空間。異なるバスのIDを混同せず
         * デコーダで妥当性を判定してからのみ試運転状態へ渡す。 */
        frame.id = frame.extended ? header.ExtId : header.StdId;
        if (hcan == port->rs_can) {
            BRS06Feedback fb;
            if (b_rs06_parse_feedback(&frame, &fb) &&
                fb.motor_id == port->bench.config.rs06_motor_id &&
                fb.host_id == port->bench.config.rs06_host_id &&
                (!port->rs_rx_valid || now_us > port->rs_last_rx_us)) {
                /* フレームが対象モータ由来と確定したときだけ時刻を更新。
                 * 別IDや古い時刻で通信断を隠さない。 */
                port->rs_rx_valid = true;
                port->rs_last_rx_us = now_us;
                b_bench_on_rs06_feedback(&port->bench, &fb, now_us);
            }
        } else {
            BC620Feedback fb;
            if (b_c620_parse_feedback(&frame, &fb) &&
                fb.motor_id == port->bench.config.c620_motor_id &&
                (!port->wheel_rx_valid || now_us > port->wheel_last_rx_us)) {
                /* C620ベンチは電流パルス用に数値フィードバックを使用しない。
                 * 受信時刻の所有をCANポートへ集約する。 */
                port->wheel_rx_valid = true;
                port->wheel_last_rx_us = now_us;
            }
        }
    }
}

/*
 * TxMailboxのIRQが来た時点で試運転の送信完了を確定する。
 * RS06 Type3は特に「送信完了後の新規正常応答」が必要なので
 * 送信受付時刻で代用してはいけない。C620はpendingを解放する。
 */
void b_f446_motor_port_on_tx_done(BF446MotorBenchPort *port,
                                   CAN_HandleTypeDef *hcan,
                                   uint8_t mailbox_index_value, uint64_t now_us)
{
    if (port == NULL || hcan == NULL || mailbox_index_value > 2u) return;
    if (hcan == port->rs_can && port->rs_mailbox_pending[mailbox_index_value]) {
        b_bench_rs06_tx_done(&port->bench,
                             &port->rs_mailbox_frame[mailbox_index_value],
                             now_us);
        port->rs_mailbox_pending[mailbox_index_value] = false;
    } else if (hcan == port->wheel_can) {
        port->wheel_tx_pending = false;
    }
}

/* 明示的な操作でのみ試験用の短いトルクパルスを要求する。
 * ここで許可するのはbenchへの要求で、直ちにCAN出力するわけではない。 */
bool b_f446_motor_port_pulse_rs06(BF446MotorBenchPort *port, float torque_nm,
                                   uint64_t now_us)
{
    if (port == NULL) return false;
    const uint32_t mask = enter_critical();
    const bool ok = b_bench_request_rs06(&port->bench, torque_nm, now_us);
    leave_critical(mask);
    return ok;
}

/* CAN2電流パルスの開始時にもポートで受信有効性を確認する。
 * bench本体は電流上限・パルス期限だけを判断する。 */
bool b_f446_motor_port_pulse_c620(BF446MotorBenchPort *port, float current_a,
                                   uint64_t now_us)
{
    if (port == NULL) return false;
    const uint32_t mask = enter_critical();
    const bool alive = rx_alive(port->wheel_rx_valid,
                                port->wheel_last_rx_us, now_us,
                                BENCH_C620_RX_TIMEOUT_US);
    const bool ok = b_bench_request_c620(&port->bench, current_a, now_us, alive);
    leave_critical(mask);
    return ok;
}

/* 中止は試運転状態機械の停止要求を作る。CAN通信が
 * 失われていればType4が届かないため物理非常停止を代替しない。 */
void b_f446_motor_port_cancel(BF446MotorBenchPort *port)
{
    if (port == NULL) return;
    const uint32_t mask = enter_critical();
    b_bench_cancel(&port->bench);
    leave_critical(mask);
}
