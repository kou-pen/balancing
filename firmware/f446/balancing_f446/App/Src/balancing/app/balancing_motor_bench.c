#include "balancing/app/balancing_motor_bench.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 以下は単軸の短時間試験専用であり、4軸バランシング時の制御ゲイン・
 * 実機の許容トルクや非常停止の保証値ではない。
 * 受信期限はCANポート層が一括管理する。ここでは有効化とパルス時間だけ扱う。
 */
#define RS_ENABLE_TIMEOUT_US 150000u
#define RS_ENABLE_RETRY_US 20000u
/* 初期単軸試運転専用のコード上限。機械的安全を保証する数値ではない。 */
#define BENCH_RS06_HARD_LIMIT_NM 0.5f
#define BENCH_C620_HARD_LIMIT_A 0.5f
#define BENCH_HARD_PULSE_MS 200u

/* 期限判定は差分で行い、時刻逆行を「経過済み」と誤認しない。
 * 期限に等しい時刻は終了側の条件に入る。 */
static bool elapsed(uint64_t now_us, uint64_t since_us, uint64_t delta_us)
{
    return now_us >= since_us && (now_us - since_us) >= delta_us;
}

/* RS06停止はゼロトルク要求ではなくType4無効化の要求を残す。
 * 通信断ではType4の実送達を保証できないため物理停止機構が必要。 */
static void stop_rs(BMotorBench *bench, bool failed)
{
    bench->rs_stop_pending = true;
    bench->state = failed ? B_BENCH_FAILED : B_BENCH_STOPPING;
    bench->requested_output = 0.0f;
}

/*
 * 実機で確認したIDと低出力上限・試験時間の入力がなければ初期化しない。
 * 短時間パルスでも常時駆動状態を残す危険があるため、上限と時間を
 * 別々に拘束する。ここで初期化してもモータ自動有効化はしない。
 */
bool b_bench_init(BMotorBench *bench, const BMotorBenchConfig *config)
{
    if (!bench || !config || config->rs06_motor_id == 0u ||
        config->c620_motor_id < 1u || config->c620_motor_id > 4u ||
        !isfinite(config->rs06_limit_nm) || config->rs06_limit_nm <= 0.0f ||
        config->rs06_limit_nm > BENCH_RS06_HARD_LIMIT_NM ||
        !isfinite(config->c620_limit_a) || config->c620_limit_a <= 0.0f ||
        config->c620_limit_a > BENCH_C620_HARD_LIMIT_A ||
        config->pulse_duration_ms == 0u || config->pulse_duration_ms > BENCH_HARD_PULSE_MS) {
        return false;
    }
    memset(bench, 0, sizeof(*bench));
    bench->config = *config;
    bench->state = B_BENCH_IDLE;
    return true;
}

/*
 * 対象モータIDとhost IDを両方照合し、別の装置からの状態通知で
 * 追従状態や有効化判定を更新しない。FWのmode=2かつfault=0を
 * 「正常なフィードバック」と扱うのはベンチの仮定で、実機で要照合。
 */
void b_bench_on_rs06_feedback(BMotorBench *bench,
                              const BRS06Feedback *feedback, uint64_t now_us)
{
    if (!bench || !feedback || feedback->motor_id != bench->config.rs06_motor_id ||
        feedback->host_id != bench->config.rs06_host_id) {
        return;
    }
    bench->rs_feedback_valid = true;
    bench->rs_feedback_us = now_us;
    bench->rs_operational = (feedback->mode == 2u && feedback->fault_bits == 0u);
    if (feedback->fault_bits != 0u &&
        (bench->state == B_BENCH_WAIT_RS06 || bench->state == B_BENCH_RS06_PULSE)) {
        stop_rs(bench, true);
    }
}

/*
 * パルスはIDLE中かつ明示要求があった場合だけ開始する。
 * 先にType3送信完了→新しい正常応答を確認し、待機中のトルクは出さない。
 * 直前の有効化送信履歴を無効にして旧レスポンスの再利用を防ぐ。
 */
bool b_bench_request_rs06(BMotorBench *bench, float torque_nm, uint64_t now_us)
{
    if (!bench || bench->state != B_BENCH_IDLE || bench->rs_stop_pending ||
        !isfinite(torque_nm) || torque_nm == 0.0f ||
        fabsf(torque_nm) > bench->config.rs06_limit_nm) {
        return false;
    }
    bench->requested_output = torque_nm;
    bench->requested_us = now_us;
    bench->rs_enable_sent_valid = false;
    bench->rs_enable_offer_valid = false;
    bench->state = B_BENCH_WAIT_RS06;
    return true;
}

/*
 * C620はRS06と違って有効化トランザクションを持たないので、
 * CANポートの受信状態が正常なことをパルス開始前の条件とする。
 * タイムアウト値と受信時刻は持たず、ポートからの判定だけを消費する。
 */
bool b_bench_request_c620(BMotorBench *bench, float current_a, uint64_t now_us,
                          bool feedback_current)
{
    if (!bench || bench->state != B_BENCH_IDLE ||
        !feedback_current ||
        !isfinite(current_a) || current_a == 0.0f ||
        fabsf(current_a) > bench->config.c620_limit_a) {
        return false;
    }
    bench->requested_output = current_a;
    bench->pulse_start_us = now_us;
    bench->state = B_BENCH_C620_PULSE;
    return true;
}

/* キャンセルは単に状態をIDLEへ書き換えるだけでは不十分。
 * 有効化済みRS06はType4送信完了までSTOPPINGを維持する。 */
void b_bench_cancel(BMotorBench *bench)
{
    if (!bench) {
        return;
    }
    /* 異常終了は手動中止でも解除しない。再試験には明示的な再初期化が必要。 */
    if (bench->state == B_BENCH_FAILED) {
        bench->requested_output = 0.0f;
        return;
    }
    if (bench->state == B_BENCH_WAIT_RS06 || bench->state == B_BENCH_RS06_PULSE ||
        bench->rs_enable_sent_valid) {
        stop_rs(bench, false);
    } else {
        bench->state = B_BENCH_IDLE;
        bench->requested_output = 0.0f;
    }
}

/*
 * 4msの送信機会に、最優先はType4停止、次にType3有効化、
 * 最後にType1トルクパルスの順で1フレームを選ぶ。
 * 有効化要求時刻ではなくType3送信完了時刻より後の応答を採用する。
 */
bool b_bench_rs06_frame(BMotorBench *bench, uint64_t now_us,
                        bool feedback_current, BMotorCanFrame *out)
{
    if (!bench || !out) {
        return false;
    }
    if (bench->rs_stop_pending) {
        return b_rs06_control_frame(bench->config.rs06_motor_id,
                                    bench->config.rs06_host_id, false, out);
    }
    if (bench->state == B_BENCH_WAIT_RS06) {
        /* 有効化に時間がかかっても永続的な待機にはせず、
         * 期限到達時に停止要求を発生させる。 */
        if (elapsed(now_us, bench->requested_us, RS_ENABLE_TIMEOUT_US)) {
            stop_rs(bench, true);
            return b_rs06_control_frame(bench->config.rs06_motor_id,
                                        bench->config.rs06_host_id, false, out);
        }
        if (bench->rs_enable_sent_valid && bench->rs_feedback_valid &&
            bench->rs_feedback_us > bench->rs_enable_sent_us &&
            bench->rs_operational && feedback_current) {
            bench->state = B_BENCH_RS06_PULSE;
            bench->pulse_start_us = now_us;
        } else {
            /* 有効化済みの送信確認後は、状態応答まで再送しない。 */
            if (bench->rs_enable_sent_valid ||
                (bench->rs_enable_offer_valid &&
                 !elapsed(now_us, bench->rs_enable_offer_us, RS_ENABLE_RETRY_US))) {
                return false;
            }
            bench->rs_enable_offer_valid = true;
            bench->rs_enable_offer_us = now_us;
            return b_rs06_control_frame(bench->config.rs06_motor_id,
                                        bench->config.rs06_host_id, true, out);
        }
    }
    /* トルク出力中はポートから受け取ったリンク状態だけで停止を判断する。
     * 返信時刻・鮮度期限の比較はポートで行い、ここでは再計算しない。 */
    if (bench->state == B_BENCH_RS06_PULSE) {
        if (!bench->rs_feedback_valid || !bench->rs_operational ||
            !feedback_current) {
            stop_rs(bench, true);
        } else if (elapsed(now_us, bench->pulse_start_us,
                           (uint64_t)bench->config.pulse_duration_ms * 1000u)) {
            stop_rs(bench, false);
        }
        if (bench->rs_stop_pending) {
            return b_rs06_control_frame(bench->config.rs06_motor_id,
                                        bench->config.rs06_host_id, false, out);
        }
        return b_rs06_torque_frame(bench->config.rs06_motor_id,
                                   bench->requested_output, out);
    }
    return false;
}

/*
 * ここはHAL送信受付ではなくCAN Tx完了IRQの通知先。
 * Type3後の返信だけを有効化成功と認め、Type4完了後にSTOPPINGを解除する。
 * 実際に物理モータが停止したことはこの通知だけでは証明できない。
 */
void b_bench_rs06_tx_done(BMotorBench *bench, const BMotorCanFrame *frame,
                          uint64_t now_us)
{
    if (!bench || !frame || !frame->extended || frame->dlc != 8u ||
        (uint8_t)frame->id != bench->config.rs06_motor_id) {
        return;
    }
    const uint8_t type = (uint8_t)(frame->id >> 24);
    if (type == 3u && bench->state == B_BENCH_WAIT_RS06) {
        bench->rs_enable_sent_us = now_us;
        bench->rs_enable_sent_valid = true;
    } else if (type == 4u && bench->rs_stop_pending) {
        bench->rs_stop_pending = false;
        bench->rs_enable_sent_valid = false;
        if (bench->state == B_BENCH_STOPPING) {
            bench->state = B_BENCH_IDLE;
        }
    }
}

/*
 * CAN2の共通0x200フレームを毎回構成し、指定1軸以外は0Aにする。
 * パルス終了またはポートから通信異常が通知されたら0Aへ戻す。
 * トルク・電流ゼロのCAN指令だけでは電源遮断にならない。
 */
bool b_bench_c620_frame(BMotorBench *bench, uint64_t now_us,
                        bool feedback_current, BMotorCanFrame *out)
{
    if (!bench || !out) {
        return false;
    }
    float current[4] = {0.0f};
    if (bench->state == B_BENCH_C620_PULSE) {
        if (!feedback_current) {
            bench->state = B_BENCH_FAILED;
            bench->requested_output = 0.0f;
        } else if (elapsed(now_us, bench->pulse_start_us,
                           (uint64_t)bench->config.pulse_duration_ms * 1000u)) {
            bench->state = B_BENCH_IDLE;
            bench->requested_output = 0.0f;
        } else {
            current[bench->config.c620_motor_id - 1u] = bench->requested_output;
        }
    }
    return b_c620_current_frame(current, bench->config.c620_limit_a, out);
}
