#include "balancing/control/controller/balancing_core.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool sample_is_recent(BSampleStamp sample, uint64_t now_us,
                             uint64_t max_age_us)
{
    return sample.valid && now_us >= sample.stamp_us &&
           now_us - sample.stamp_us <= max_age_us;
}

BFreshness b_check_freshness(const BObservedStamps *stamps, uint64_t now_us)
{
    BFreshness result = {0};
    unsigned i;

    if (stamps == NULL) {
        return result;
    }

    result.imu = sample_is_recent(stamps->imu, now_us, 50000u);
    result.power = sample_is_recent(stamps->power, now_us, 500000u);

    for (i = 0; i < 4; ++i) {
        result.rs[i] = sample_is_recent(stamps->rs[i], now_us, 30000u);
    }
    for (i = 0; i < 2; ++i) {
        result.wheel[i] = sample_is_recent(stamps->wheel[i], now_us, 20000u);
    }

    return result;
}

bool b_freshness_all(BFreshness freshness)
{
    unsigned i;

    if (!freshness.imu || !freshness.power) {
        return false;
    }
    for (i = 0; i < 4; ++i) {
        if (!freshness.rs[i]) {
            return false;
        }
    }
    for (i = 0; i < 2; ++i) {
        if (!freshness.wheel[i]) {
            return false;
        }
    }
    return true;
}

void b_mode_init(BModeMachine *machine)
{
    if (machine == NULL) {
        return;
    }
    memset(machine, 0, sizeof(*machine));
    machine->mode = B_MODE_DISARMED;
}

static bool arm_preconditions(const BModeInput *input)
{
    return b_freshness_all(input->fresh) && input->power_on &&
           input->calibration_verified && input->model_verified &&
           input->limits_verified && input->posture_ok;
}

static bool all_enabled_after_request(const BModeMachine *machine,
                                      const BModeInput *input,
                                      uint64_t now_us)
{
    unsigned i;
    for (i = 0; i < 4; ++i) {
        /* 要求時刻と送信成功時刻は異なる。送信後の応答だけ採用する。 */
        if (!input->rs_operational[i] ||
            !input->rs_enable_sent[i].valid ||
            input->rs_enable_sent[i].stamp_us < machine->enable_request_us ||
            input->rs_enable_sent[i].stamp_us > now_us ||
            !input->rs_reply[i].valid ||
            input->rs_reply[i].stamp_us <= input->rs_enable_sent[i].stamp_us ||
            !sample_is_recent(input->rs_reply[i], now_us, 30000u)) {
            return false;
        }
    }
    return true;
}

BModeOutput b_mode_step(BModeMachine *machine, const BModeInput *input,
                        uint64_t now_us)
{
    BModeOutput result = {0};
    bool ready;

    if (machine == NULL) {
        result.mode = B_MODE_FAULT;
        result.fault_cause = B_FAULT_GUARD;
        result.request_rs_disable = true;
        return result;
    }
    if (input == NULL) {
        machine->mode = B_MODE_FAULT;
        machine->fault_cause = B_FAULT_GUARD;
        machine->arm_pending = false;
        result.request_rs_disable = true;
        goto finish;
    }

    if (input->estop_active || input->external_fault) {
        machine->mode = B_MODE_FAULT;
        machine->fault_cause = input->estop_active ?
                               B_FAULT_ESTOP : B_FAULT_EXTERNAL;
        machine->arm_pending = false;
        result.request_rs_disable = true;
        result.request_power_off = input->estop_active;
        goto finish;
    }

    /* 3周期分（6 ms）の制御停止は、稼働中のみ独立したFAULT。
     * 時計の逆行も同様に扱う。起動前やDISARMEDではラッチしない。 */
    if (machine->mode == B_MODE_BALANCING && machine->has_last_step &&
        (now_us < machine->last_step_us ||
         now_us - machine->last_step_us >= UINT64_C(6000))) {
        machine->mode = B_MODE_FAULT;
        machine->fault_cause = B_FAULT_SCHEDULER;
        machine->arm_pending = false;
        result.request_rs_disable = true;
        goto finish;
    }

    ready = arm_preconditions(input);

    if (machine->mode == B_MODE_FAULT) {
        /* 明示的な故障原因解除許可なしに再ARMはしない。 */
        if (input->clear_fault_edge && input->clear_authorized) {
            machine->mode = B_MODE_DISARMED;
            machine->fault_cause = B_FAULT_NONE;
            machine->arm_pending = false;
        }
    } else if (machine->mode == B_MODE_BALANCING) {
        if (!ready) {
            machine->mode = B_MODE_FAULT;
            machine->fault_cause = B_FAULT_GUARD;
            machine->arm_pending = false;
            result.request_rs_disable = true;
        } else if (input->disarm_edge) {
            machine->mode = B_MODE_DISARMED;
            machine->arm_pending = false;
            result.request_rs_disable = true;
        }
    } else {
        /* DISARMED中のARMは二段階：要求→4台の有効化後応答。 */
        if (input->disarm_edge) {
            result.request_rs_disable = machine->arm_pending;
            machine->arm_pending = false;
        } else if (machine->arm_pending) {
            if (!ready || now_us < machine->enable_request_us) {
                machine->arm_pending = false;
                result.request_rs_disable = true;
                result.arm_rejected = true;
            } else if (all_enabled_after_request(machine, input, now_us)) {
                machine->arm_pending = false;
                machine->mode = B_MODE_BALANCING;
            }
        } else if (input->arm_edge) {
            if (!ready) {
                result.arm_rejected = true;
            } else {
                machine->arm_pending = true;
                machine->enable_request_us = now_us;
                result.request_rs_enable = true;
            }
        }
    }

finish:
    machine->last_step_us = now_us;
    machine->has_last_step = true;
    result.mode = machine->mode;
    result.fault_cause = machine->fault_cause;
    result.drive_allowed = machine->mode == B_MODE_BALANCING;
    return result;
}

bool b_rate_limit(float previous, float request, float max_rate,
                  float dt_s, float *result)
{
    float max_step;
    float delta;

    if (result == NULL) {
        return false;
    }
    *result = 0.0f;
    if (!isfinite(previous) || !isfinite(request) || !isfinite(max_rate) ||
        !isfinite(dt_s) || max_rate <= 0.0f || dt_s <= 0.0f) {
        return false;
    }
    max_step = max_rate * dt_s;
    if (!isfinite(max_step)) {
        return false;
    }

    delta = request - previous;
    if (!isfinite(delta)) {
        return false;
    }
    delta = fmaxf(-max_step, fminf(max_step, delta));
    *result = previous + delta;
    if (!isfinite(*result)) {
        *result = 0.0f;
        return false;
    }
    return true;
}

bool b_wheel_allocate(float total_balance_nm, float yaw_delta_nm,
                      float left_limit_nm, float right_limit_nm,
                      BWheelAllocation *result)
{
    float base_request;
    float base_limit;
    float base;
    float yaw_limit;
    float yaw;

    if (result == NULL) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    if (!isfinite(total_balance_nm) || !isfinite(yaw_delta_nm) ||
        !isfinite(left_limit_nm) || !isfinite(right_limit_nm) ||
        left_limit_nm <= 0.0f || right_limit_nm <= 0.0f) {
        return false;
    }

    base_request = total_balance_nm / 2.0f;
    base_limit = fminf(left_limit_nm, right_limit_nm);
    base = fmaxf(-base_limit, fminf(base_limit, base_request));
    yaw_limit = fminf(left_limit_nm - fabsf(base),
                      right_limit_nm - fabsf(base));
    yaw = fmaxf(-yaw_limit, fminf(yaw_limit, yaw_delta_nm));

    result->base_each_nm = base;
    result->yaw_applied_nm = yaw;
    result->left_nm = base - yaw;
    result->right_nm = base + yaw;
    result->saturated = (base != base_request) || (yaw != yaw_delta_nm);
    return true;
}

bool b_lqr_compute(const BLqrConfig *config, const BControlState *state,
                   const BControlState *target, BControlTorque *torque)
{
    float current[B_STATE_COUNT];
    float desired[B_STATE_COUNT];
    float error[B_STATE_COUNT];
    float result[B_OUTPUT_COUNT];
    unsigned i;
    unsigned j;

    if (torque == NULL) {
        return false;
    }
    *torque = (BControlTorque){0};

    if (config == NULL || state == NULL || target == NULL ||
        !config->verified || !isfinite(config->max_position_error_m) ||
        config->max_position_error_m <= 0.0f) {
        return false;
    }

    /* 変換はこの境界に集約し、構造体のメモリ配置に依存しない。 */
    current[B_STATE_THETA] = state->theta_rad;
    current[B_STATE_THETA_RATE] = state->theta_rate_rad_s;
    current[B_STATE_POSITION] = state->position_m;
    current[B_STATE_VELOCITY] = state->velocity_m_s;
    current[B_STATE_PITCH] = state->pitch_rad;
    current[B_STATE_PITCH_RATE] = state->pitch_rate_rad_s;

    desired[B_STATE_THETA] = target->theta_rad;
    desired[B_STATE_THETA_RATE] = target->theta_rate_rad_s;
    desired[B_STATE_POSITION] = target->position_m;
    desired[B_STATE_VELOCITY] = target->velocity_m_s;
    desired[B_STATE_PITCH] = target->pitch_rad;
    desired[B_STATE_PITCH_RATE] = target->pitch_rate_rad_s;

    for (i = 0; i < B_STATE_COUNT; ++i) {
        if (!isfinite(current[i]) || !isfinite(desired[i])) {
            return false;
        }
        error[i] = current[i] - desired[i];
        if (!isfinite(error[i])) {
            return false;
        }
    }
    error[B_STATE_POSITION] =
        fmaxf(-config->max_position_error_m,
              fminf(config->max_position_error_m, error[B_STATE_POSITION]));

    for (j = 0; j < B_OUTPUT_COUNT; ++j) {
        double value = config->equilibrium[j];
        if (!isfinite(value)) {
            return false;
        }
        for (i = 0; i < B_STATE_COUNT; ++i) {
            if (!isfinite(config->gain[j][i])) {
                return false;
            }
            value -= (double)config->gain[j][i] * error[i];
        }
        if (!isfinite(value) || fabs(value) > FLT_MAX) {
            return false;
        }
        result[j] = (float)value;
    }

    torque->wheel_total_nm = result[B_OUTPUT_WHEEL_TOTAL];
    torque->hip_total_nm = result[B_OUTPUT_HIP_TOTAL];
    return true;
}
