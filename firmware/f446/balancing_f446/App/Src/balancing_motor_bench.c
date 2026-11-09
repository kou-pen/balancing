#include "balancing_motor_bench.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RS_FEEDBACK_TIMEOUT_US 30000u
#define C620_FEEDBACK_TIMEOUT_US 20000u
#define RS_ENABLE_TIMEOUT_US 150000u
#define RS_ENABLE_RETRY_US 20000u
/* 初期単軸試運転専用のコード上限。機械的安全を保証する数値ではない。 */
#define BENCH_RS06_HARD_LIMIT_NM 0.5f
#define BENCH_C620_HARD_LIMIT_A 0.5f
#define BENCH_HARD_PULSE_MS 200u

static bool elapsed(uint64_t now_us, uint64_t since_us, uint64_t delta_us)
{
    return now_us >= since_us && (now_us - since_us) >= delta_us;
}

static bool fresh(uint64_t now_us, uint64_t since_us, uint64_t maximum_age)
{
    return now_us >= since_us && (now_us - since_us) <= maximum_age;
}

static void stop_rs(BMotorBench *bench, bool failed)
{
    bench->rs_stop_pending = true;
    bench->state = failed ? B_BENCH_FAILED : B_BENCH_STOPPING;
    bench->requested_output = 0.0f;
}

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

void b_bench_on_c620_feedback(BMotorBench *bench,
                              const BC620Feedback *feedback, uint64_t now_us)
{
    if (bench && feedback &&
        feedback->motor_id == bench->config.c620_motor_id) {
        bench->wheel_feedback_valid = true;
        bench->wheel_feedback_us = now_us;
    }
}

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

bool b_bench_request_c620(BMotorBench *bench, float current_a, uint64_t now_us)
{
    if (!bench || bench->state != B_BENCH_IDLE ||
        !bench->wheel_feedback_valid ||
        !fresh(now_us, bench->wheel_feedback_us, C620_FEEDBACK_TIMEOUT_US) ||
        !isfinite(current_a) || current_a == 0.0f ||
        fabsf(current_a) > bench->config.c620_limit_a) {
        return false;
    }
    bench->requested_output = current_a;
    bench->pulse_start_us = now_us;
    bench->state = B_BENCH_C620_PULSE;
    return true;
}

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

bool b_bench_rs06_frame(BMotorBench *bench, uint64_t now_us, BMotorCanFrame *out)
{
    if (!bench || !out) {
        return false;
    }
    if (bench->rs_stop_pending) {
        return b_rs06_control_frame(bench->config.rs06_motor_id,
                                    bench->config.rs06_host_id, false, out);
    }
    if (bench->state == B_BENCH_WAIT_RS06) {
        if (elapsed(now_us, bench->requested_us, RS_ENABLE_TIMEOUT_US)) {
            stop_rs(bench, true);
            return b_rs06_control_frame(bench->config.rs06_motor_id,
                                        bench->config.rs06_host_id, false, out);
        }
        if (bench->rs_enable_sent_valid && bench->rs_feedback_valid &&
            bench->rs_feedback_us > bench->rs_enable_sent_us &&
            bench->rs_operational &&
            fresh(now_us, bench->rs_feedback_us, RS_FEEDBACK_TIMEOUT_US)) {
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
    if (bench->state == B_BENCH_RS06_PULSE) {
        if (!bench->rs_feedback_valid || !bench->rs_operational ||
            !fresh(now_us, bench->rs_feedback_us, RS_FEEDBACK_TIMEOUT_US)) {
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

bool b_bench_c620_frame(BMotorBench *bench, uint64_t now_us, BMotorCanFrame *out)
{
    if (!bench || !out) {
        return false;
    }
    float current[4] = {0.0f};
    if (bench->state == B_BENCH_C620_PULSE) {
        if (!bench->wheel_feedback_valid ||
            !fresh(now_us, bench->wheel_feedback_us, C620_FEEDBACK_TIMEOUT_US)) {
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
