#ifndef BALANCING_F446_MOTOR_BENCH_PORT_H
#define BALANCING_F446_MOTOR_BENCH_PORT_H

#include "balancing_motor_bench.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAN1=RS06、CAN2=C620のHAL接続。試運転時のみ使用する。
 * @spec{BOARD-F446-001}
 * @warning CubeMX生成のCANピン、ビットタイミング、フィルタ、
 *          HAL_CAN_Start、割り込み設定は別途必要。
 */
typedef struct {
    CAN_HandleTypeDef *rs_can;
    CAN_HandleTypeDef *wheel_can;
    BMotorBench bench;
    BMotorCanFrame rs_mailbox_frame[3];
    bool rs_mailbox_pending[3];
    bool wheel_tx_pending;
    uint8_t divider;
} BF446MotorBenchPort;

/**
 * @brief CANを起動せず、試運転状態を安全側で初期化する。
 * @spec{BOARD-F446-001}
 */
bool b_f446_motor_port_init(BF446MotorBenchPort *port,
                             CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2,
                             const BMotorBenchConfig *config);

/**
 * @brief 500 Hz主ループから呼ぶ。CAN1は2回に1回、CAN2は毎回。
 * @spec{BOARD-F446-001}
 * @warning 自動でモータを有効化しない。電流0のみが自動送信される。
 */
void b_f446_motor_port_tick_2ms(BF446MotorBenchPort *port, uint64_t now_us);

/**
 * @brief CAN Rx FIFO0の割り込みから呼ぶ。フィードバックを収集する。
 * @spec{PROTO-RS06-001}
 * @spec{PROTO-C620-001}
 */
void b_f446_motor_port_on_rx0(BF446MotorBenchPort *port,
                               CAN_HandleTypeDef *hcan, uint64_t now_us);

/**
 * @brief CAN1送信完了IRQから、ハードウェアメールボックス番号0〜2で呼ぶ。
 * @spec{PROTO-RS06-001}
 */
void b_f446_motor_port_on_tx_done(BF446MotorBenchPort *port,
                                   CAN_HandleTypeDef *hcan,
                                   uint8_t mailbox_index, uint64_t now_us);

/**
 * @brief 明示的な操作からRS06短時間試運転を要求する。
 * @spec{PROTO-RS06-001}
 */
bool b_f446_motor_port_pulse_rs06(BF446MotorBenchPort *port, float torque_nm,
                                   uint64_t now_us);

/**
 * @brief 明示的な操作からC620短時間試運転を要求する。
 * @spec{PROTO-C620-001}
 */
bool b_f446_motor_port_pulse_c620(BF446MotorBenchPort *port, float current_a,
                                   uint64_t now_us);

/**
 * @brief 手動中止。CAN送信可能ならRS06 Type4・C620電流0を出す。
 * @spec{BEHAV-001}
 */
void b_f446_motor_port_cancel(BF446MotorBenchPort *port);

#ifdef __cplusplus
}
#endif

#endif
