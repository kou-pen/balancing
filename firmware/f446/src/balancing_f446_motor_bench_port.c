#include "balancing_f446_motor_bench_port.h"

#include <string.h>

/*
 * HALとBMotorBenchの間だけを扱う。受信割込みとメインループの共有データは
 * PRIMASKで短時間保護する。タイマ割込みの中で呼ばず、メインループから呼ぶ。
 * CANフィルタ・ピン・クロック等のボード固有設定を勝手に追加しない。
 */

static uint32_t enter_critical(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

static void leave_critical(uint32_t previous)
{
    if (previous == 0u) {
        __enable_irq();
    }
}

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

static int mailbox_index(uint32_t mailbox_mask)
{
    if (mailbox_mask == CAN_TX_MAILBOX0) return 0;
    if (mailbox_mask == CAN_TX_MAILBOX1) return 1;
    if (mailbox_mask == CAN_TX_MAILBOX2) return 2;
    return -1;
}

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

void b_f446_motor_port_tick_2ms(BF446MotorBenchPort *port, uint64_t now_us)
{
    if (port == NULL) return;
    const uint32_t mask = enter_critical();
    BMotorCanFrame wheel;
    if (HAL_CAN_GetTxMailboxesFreeLevel(port->wheel_can) > 0u &&
        !port->wheel_tx_pending &&
        b_bench_c620_frame(&port->bench, now_us, &wheel)) {
        uint32_t mailbox_mask = 0u;
        if (tx_frame(port->wheel_can, &wheel, &mailbox_mask)) {
            port->wheel_tx_pending = true;
        }
    }
    port->divider ^= 1u;
    if (port->divider == 0u &&
        HAL_CAN_GetTxMailboxesFreeLevel(port->rs_can) > 0u) {
        bool any_pending = false;
        for (int i = 0; i < 3; ++i) any_pending |= port->rs_mailbox_pending[i];
        if (!any_pending) {
            BMotorCanFrame rs;
            if (b_bench_rs06_frame(&port->bench, now_us, &rs)) {
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
        frame.id = frame.extended ? header.ExtId : header.StdId;
        if (hcan == port->rs_can) {
            BRS06Feedback fb;
            if (b_rs06_parse_feedback(&frame, &fb)) {
                b_bench_on_rs06_feedback(&port->bench, &fb, now_us);
            }
        } else {
            BC620Feedback fb;
            if (b_c620_parse_feedback(&frame, &fb)) {
                b_bench_on_c620_feedback(&port->bench, &fb, now_us);
            }
        }
    }
}

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

bool b_f446_motor_port_pulse_rs06(BF446MotorBenchPort *port, float torque_nm,
                                   uint64_t now_us)
{
    if (port == NULL) return false;
    const uint32_t mask = enter_critical();
    const bool ok = b_bench_request_rs06(&port->bench, torque_nm, now_us);
    leave_critical(mask);
    return ok;
}

bool b_f446_motor_port_pulse_c620(BF446MotorBenchPort *port, float current_a,
                                   uint64_t now_us)
{
    if (port == NULL) return false;
    const uint32_t mask = enter_critical();
    const bool ok = b_bench_request_c620(&port->bench, current_a, now_us);
    leave_critical(mask);
    return ok;
}

void b_f446_motor_port_cancel(BF446MotorBenchPort *port)
{
    if (port == NULL) return;
    const uint32_t mask = enter_critical();
    b_bench_cancel(&port->bench);
    leave_critical(mask);
}
