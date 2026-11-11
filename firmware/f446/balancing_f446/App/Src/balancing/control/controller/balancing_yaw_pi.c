#include "balancing/control/controller/balancing_yaw_pi.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 積分をセッション間で持ち越すと、再ARM時に停止中の誤差だけで
 * 急なYaw差動が生じる。FAULT/モード変更の責務をこの状態に閉じ込める。
 */
void b_yaw_pi_reset(BYawPiState *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

/*
 * 単位は e [rad/s], dt [s], Ki [N m/rad] なので
 * 積分増分 Ki*e*dt は [N m]。差動指令ΔTはb_wheel_allocateで
 * 左側から減算、右側へ加算される（車輪順序を逆にしない）。
 *
 * 積分器に出力上限をそのまま適用しない。積分自体の上限と
 * 現在の車輪余力は異なり、後者はLQR倒立トルクにより変化する。
 * 候補積分が現在の飽和を悪化させるときだけ候補を棄却し、
 * 誤差反転による積分解消は許す。外部の配分器でも最終クリップが必要。
 */
bool b_yaw_pi_step(BYawPiState *state, const BYawPiGains *gains,
                   const BYawPiInput *input, BYawPiOutput *output)
{
    if (output == NULL) {
        b_yaw_pi_reset(state);
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (state == NULL || input == NULL) {
        b_yaw_pi_reset(state);
        return false;
    }

    /* 鮮度・通信断の判断は安全状態機械に集約する。
     * 本層はBALANCING以外で差動を出さず積分を持ち越さない。
     * 安全層が判断を終えた後に、制御周期ごとに本関数を呼ぶ。 */
    if (!input->balancing) {
        b_yaw_pi_reset(state);
        return true;
    }
    if (gains == NULL || !gains->verified ||
        !isfinite(gains->kp_nm_s_rad) || gains->kp_nm_s_rad < 0.0f ||
        !isfinite(gains->ki_nm_rad) || gains->ki_nm_rad < 0.0f ||
        !isfinite(gains->integral_limit_nm) || gains->integral_limit_nm < 0.0f ||
        !isfinite(input->target_rate_rad_s) ||
        !isfinite(input->measured_rate_rad_s) ||
        !isfinite(input->dt_s) || input->dt_s <= 0.0f ||
        !isfinite(input->available_delta_nm) ||
        input->available_delta_nm < 0.0f ||
        !isfinite(state->integral_nm)) {
        b_yaw_pi_reset(state);
        return false;
    }

    /* BALANCING初回は初期化状態から計算。明示resetを忘れた
     * アクティブでないセッションの過去積分も採用しない。 */
    const double old_i = state->active ? state->integral_nm : 0.0;
    const double error = (double)input->target_rate_rad_s -
                         (double)input->measured_rate_rad_s;
    const double p = (double)gains->kp_nm_s_rad * error;
    const double change = (double)gains->ki_nm_rad * error * input->dt_s;
    const double i_max = gains->integral_limit_nm;
    const double capacity = input->available_delta_nm;
    double next_i = fmax(-i_max, fmin(i_max, old_i + change));

    /*
     * 積分候補が飽和方向へ押し続けるなら前回積分を維持する。
     * 反対方向の誤差は積分を戻すため、飽和中でも棄却しない。
     * 平衡トルクを削ってYawを優先する制御には変更しない。
     */
    const bool freeze = (p + next_i > capacity && next_i > old_i) ||
                        (p + next_i < -capacity && next_i < old_i);
    if (freeze) {
        next_i = old_i;
    }
    const double raw = p + next_i;
    const double applied = fmax(-capacity, fmin(capacity, raw));
    if (!isfinite(error) || !isfinite(p) || !isfinite(change) ||
        !isfinite(raw) || !isfinite(applied) ||
        fabs(next_i) > FLT_MAX || fabs(applied) > FLT_MAX) {
        b_yaw_pi_reset(state);
        return false;
    }
    state->integral_nm = (float)next_i;
    state->active = true;
    output->delta_nm = (float)applied;
    output->integral_nm = state->integral_nm;
    output->saturated = raw != applied;
    output->integral_frozen = freeze;
    return true;
}
