#ifndef BALANCING_MOTOR_BENCH_H
#define BALANCING_MOTOR_BENCH_H

#include "balancing/communication/balancing_motor_can.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 実機試験用の単軸状態。通常のバランシング状態機械とは独立する。 */
typedef enum {
    B_BENCH_IDLE,
    B_BENCH_WAIT_RS06,
    B_BENCH_RS06_PULSE,
    B_BENCH_C620_PULSE,
    B_BENCH_STOPPING,
    B_BENCH_FAILED
} BMotorBenchState;

/** @brief 実機で個別に確認して与えるパラメータ。危険な既定トルクを持たない。 */
typedef struct {
    uint8_t rs06_motor_id;
    uint8_t rs06_host_id;
    uint8_t c620_motor_id; /**< 0x200グループのID 1〜4。 */
    float rs06_limit_nm; /**< 初期試運転のコード上限0.5 N·m以下。 */
    float c620_limit_a; /**< 初期試運転のコード上限0.5 A以下。 */
    uint16_t pulse_duration_ms; /**< 1〜200 ms。自動終了する。 */
} BMotorBenchConfig;

/** @brief RS06 1台とC620 1台を個別に短時間試験するための状態。 */
typedef struct {
    BMotorBenchConfig config;
    BMotorBenchState state;
    float requested_output;
    uint64_t requested_us;
    uint64_t pulse_start_us;
    uint64_t rs_enable_sent_us;
    uint64_t rs_enable_offer_us;
    uint64_t rs_feedback_us;
    bool rs_enable_sent_valid;
    bool rs_enable_offer_valid;
    bool rs_feedback_valid;
    bool rs_operational;
    bool rs_stop_pending;
} BMotorBench;

/**
 * @brief 試験器を駆動無効状態で初期化する。
 * @spec{PROTO-RS06-001}
 * @spec{PROTO-C620-001}
 * @param[out] bench 初期化する試験器。
 * @param[in] config 実機で確認済みのID・低いトルクと電流の限度。
 * @return 設定が正しければtrue。
 */
bool b_bench_init(BMotorBench *bench, const BMotorBenchConfig *config);

/**
 * @brief RS06 Type2/24の新しい受信値を試験器へ渡す。
 * @spec{PROTO-RS06-001}
 */
void b_bench_on_rs06_feedback(BMotorBench *bench,
                              const BRS06Feedback *feedback, uint64_t now_us);

/**
 * @brief RS06へ短いトルクパルスを要求する。Type3有効化応答までは出力しない。
 * @spec{PROTO-RS06-001}
 * @warning 安全に固定したモータで、試験操作からのみ呼ぶこと。自動実行は禁止。
 */
bool b_bench_request_rs06(BMotorBench *bench, float torque_nm, uint64_t now_us);

/**
 * @brief C620へ短い電流パルスを要求する。通信状態はデバイス層で判定する。
 * @spec{PROTO-C620-001}
 * @param[in] feedback_current CAN受信ポートが期限内のフィードバックを確認した結果。
 * @warning 本関数で受信期限を再計算しない。通信不良ならfalseを渡す。
 */
bool b_bench_request_c620(BMotorBench *bench, float current_a, uint64_t now_us,
                          bool feedback_current);

/**
 * @brief 試験を中止し、RS06の停止指令を優先する。
 * @spec{PROTO-RS06-001}
 */
void b_bench_cancel(BMotorBench *bench);

/**
 * @brief CAN1で次に送るRS06フレームを生成する。4 msごとに呼ぶ。
 * @spec{PROTO-RS06-001}
 * @param[in] feedback_current CAN受信ポートが期限内の返信を確認した結果。
 * @return 送信するフレームがあればtrue。
 * @warning 受信期限はポート層のみで管理し、ここでは経過時間を再判定しない。
 */
bool b_bench_rs06_frame(BMotorBench *bench, uint64_t now_us,
                        bool feedback_current, BMotorCanFrame *out);

/**
 * @brief RS06の有効化・停止フレームのCAN送信完了時刻を反映する。
 * @spec{PROTO-RS06-001}
 * @pre メールボックスへ投入した時刻ではなく、CAN送信完了割り込みで呼ぶ。
 */
void b_bench_rs06_tx_done(BMotorBench *bench, const BMotorCanFrame *frame,
                          uint64_t now_us);

/**
 * @brief CAN2で次に送る0x200フレームを生成する。2 msごとに呼ぶ。
 * @spec{PROTO-C620-001}
 * @param[in] feedback_current CAN受信ポートが期限内の返信を確認した結果。
 * @return 初期化済みならtrue。停止中は全チャネル0 A。
 * @warning ポート層が必ず最新の受信状態を渡すこと。
 */
bool b_bench_c620_frame(BMotorBench *bench, uint64_t now_us,
                        bool feedback_current, BMotorCanFrame *out);

#ifdef __cplusplus
}
#endif
#endif
