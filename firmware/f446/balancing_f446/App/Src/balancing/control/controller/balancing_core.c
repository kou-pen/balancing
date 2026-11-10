#include "balancing/control/controller/balancing_core.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 鮮度は「有効ビット」だけでは判断せず、最後に確認できた測定時刻からの経過で決める。
 * 同じ時刻を再利用しても鮮度を回復させない。共通の単調増加時計を前提とし、
 * 未来時刻を含む測定は拒否する。期限ちょうどまでは有効とする。
 */
static bool sample_is_recent(BSampleStamp sample, uint64_t now_us,
                             uint64_t max_age_us)
{
    return sample.valid && now_us >= sample.stamp_us &&
           now_us - sample.stamp_us <= max_age_us;
}

/*
 * 各センサで規定の更新期限が異なるため、IMU・電源・RS06・車輪を個別判定する。
 * タイムアウト値は現行の仕様契約であり、数学テスト都合で変更しない。
 * NULL入力なら全てfalseのスナップショットを返す。
 */
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

/*
 * ARMと稼働監視に必要な全観測系統の論理積。
 * 未接続のセンサを暗黙にtrueとして扱うと危険なので、1軸でも欠ければfalse。
 */
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

/* 起動時に前回のARM要求やFAULT原因を引き継がず、DISARMEDから開始する。
 * 電源断後にFAULTを保持する方式は別途未決定であり、この実装で保証しない。 */
void b_mode_init(BModeMachine *machine)
{
    /* 状態自体が存在しない場合でも、結果には停止要求を明示する。
     * 出力コマンドが実際にモータに届くことまでは保証しない。 */
    if (machine == NULL) {
        return;
    }
    memset(machine, 0, sizeof(*machine));
    machine->mode = B_MODE_DISARMED;
}

/*
 * ARMを開始するための検証済み条件をまとめる。validな入力と「実機で検証した」
 * 校正・モデル・上限は別概念。呼出元が仮の値をtrueにすると安全証明にならない。
 * 現行posture_okは上位が与えるブール値で、実機姿勢範囲ゲートとの仕様差分は
 * 別の監査Issueで管理する（コメント追加で挙動を変更しない）。
 */
static bool arm_preconditions(const BModeInput *input)
{
    return b_freshness_all(input->fresh) && input->power_on &&
           input->calibration_verified && input->model_verified &&
           input->limits_verified && input->posture_ok;
}

/*
 * RS06の有効化は要求した瞬間には成立しない。
 * 4軸すべてについて「当該ARM要求以降に実送信されたType3」と
 * 「その送信時刻より厳密に後の新しい正常応答」の両方を要求する。
 * 要求時刻をHAL受付・CAN完了の代わりに使わない。
 */
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

/*
 * 呼出し1回を制御状態機械の1イベントとして扱う。
 * 判定優先順位はE-STOP/外部FAULT、稼働中の周期異常、通常遷移。
 * 最後にdrive_allowedを状態から一意に導出し、FAULT解除と同じイベントで
 * 暗黙に再ARMしないようにする。物理非常停止はこの関数の外に必要。
 */
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
    /* 欠損した入力で最後のトルク許可を保持しない。FAULTへラッチする。 */
    if (input == NULL) {
        machine->mode = B_MODE_FAULT;
        machine->fault_cause = B_FAULT_GUARD;
        machine->arm_pending = false;
        result.request_rs_disable = true;
        goto finish;
    }

    /* 最優先の異常入力。E-STOPでは電源遮断の「要求」を追加するのみ。
     * 実電源を遮断できるかはF303と独立した物理経路の責務である。 */
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

    /* FAULT→DISARMEDの解除は明示エッジと許可の両方を要求する。
     * この同一ティック中に後続のARM遷移を実行しない分岐順序を維持する。 */
    if (machine->mode == B_MODE_FAULT) {
        /* 明示的な故障原因解除許可なしに再ARMはしない。 */
        if (input->clear_fault_edge && input->clear_authorized) {
            machine->mode = B_MODE_DISARMED;
            machine->fault_cause = B_FAULT_NONE;
            machine->arm_pending = false;
        }
    } else if (machine->mode == B_MODE_BALANCING) {
        /* 稼働中の鮮度・校正前提の欠落は中止より強いFAULTとし、
         * 明示DISARMではFAULTを生成せず駆動禁止に遷移する。 */
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
    /* 状態から許可を導き、外部が内部フラグを二重管理しないようにする。
     * 現在の値を記録しても、その周期に新規センサが来たことにはならない。 */
    machine->last_step_us = now_us;
    machine->has_last_step = true;
    result.mode = machine->mode;
    result.fault_cause = machine->fault_cause;
    result.drive_allowed = machine->mode == B_MODE_BALANCING;
    return result;
}

/*
 * 前回の目標値から1周期に進める量を max_rate × dt に制限する。
 * スティックがニュートラルへ戻った際にも瞬間的な指令反転を避けるため、
 * 出力範囲そのものではなく「前回からの変化量」を飽和する。
 * NaN・Infと不正周期は0へフェイルクローズする。
 */
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
