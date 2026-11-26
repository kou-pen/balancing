#include "balancing/control/controller/balancing_core.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 2輪の合計バランストルクをまず左右同量に分け、そのあとYaw差動を配分する。
 * 左右個別の上限が異なるときは小さい側を共通成分の上限とし、
 * 残容量の範囲でのみYawを入れるため、姿勢保持を優先する。
 */
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
    /* 共通成分を使った後の左右の余剰トルクだけをYawへ割り当てる。
     * 飽和は正常な制限動作であって、それだけではFAULTにしない。 */
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

/*
 * 6状態誤差と2×6ゲインの純粋な行列積。配置順を構造体レイアウトに
 * 依存させず、状態・目標の単位付きフィールドから明示的に転記する。
 * 状態/ゲイン/平衡入力を外部で検証するまでARM条件は満たされない。
 */
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
    /* 車輪の積算位置が走行で無制限に離れても、その誤差だけで
     * 必要以上のトルクを要求しないよう位置成分のみに制限を掛ける。 */
    error[B_STATE_POSITION] =
        fmaxf(-config->max_position_error_m,
              fminf(config->max_position_error_m, error[B_STATE_POSITION]));

    /* u = u_eq - K(x - x_ref)。
     * doubleで積を蓄積しfloat出力へ戻す前にオーバーフローを拒否する。
     * これは行列演算の正しさであって、実機のKが安全という意味ではない。 */
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
