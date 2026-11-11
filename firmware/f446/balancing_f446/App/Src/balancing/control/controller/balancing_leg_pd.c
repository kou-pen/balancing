#include "balancing/control/controller/balancing_leg_pd.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 絶対的な脚長安全範囲はCADと実測で未決定なので、ここでは定義しない。
 * 負または0の測定長、NaN/Infだけを数学的に無効とする。
 */
static bool measured_lengths_ok(const BLegPdFeedback *feedback)
{
    return feedback != NULL && isfinite(feedback->left_length_m) &&
           isfinite(feedback->right_length_m) &&
           feedback->left_length_m > 0.0f && feedback->right_length_m > 0.0f;
}

/*
 * この操作はARM遷移直前、観測スナップショットを確定した後に限る。
 * 途中で失敗した場合に前回の脚長目標を再利用させないため、先に無効化する。
 * 差分を半分として持てば初期目標は左右の測定長を正確に再現できる。
 */
bool b_leg_pd_capture(BLegPdReference *reference, const BLegPdFeedback *feedback,
                      bool disarmed, bool lengths_fresh)
{
    if (reference == NULL) {
        return false;
    }
    memset(reference, 0, sizeof(*reference));
    if (!disarmed || !lengths_fresh || !measured_lengths_ok(feedback)) {
        return false;
    }

    /* 中間結果をdoubleで計算し、floatへ戻す際の範囲逸脱を拒否する。
     * 各脚長そのものの有効性と、平均/差が表現可能かは別の検査。 */
    const double common = ((double)feedback->left_length_m +
                           (double)feedback->right_length_m) * 0.5;
    const double delta = ((double)feedback->left_length_m -
                          (double)feedback->right_length_m) * 0.5;
    if (!isfinite(common) || !isfinite(delta) || common <= 0.0 ||
        common > FLT_MAX || fabs(delta) > FLT_MAX) {
        return false;
    }
    reference->common_length_m = (float)common;
    reference->half_difference_m = (float)delta;
    reference->valid = true;
    return true;
}

/*
 * D成分は目標値の微分を用いない。ARM直後に参照値をラッチしたときの
 * 微分キックを避け、実際のRS06速度から計算した伸縮速度のみを使う。
 * Rollは水平(0 rad)を目標にし、左+・右-の抽象差動と定義する。
 * 実機の力方向/符号は未検証なので、CANへは一切接続しない。
 */
bool b_leg_pd_compute(const BLegPdReference *reference, const BLegPdGains *gains,
                      const BLegPdFeedback *feedback, BLegPdOutput *output)
{
    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (reference == NULL || gains == NULL || !reference->valid ||
        !gains->verified || !measured_lengths_ok(feedback) ||
        !isfinite(reference->common_length_m) ||
        !isfinite(reference->half_difference_m) ||
        reference->common_length_m <= 0.0f ||
        !isfinite(feedback->left_velocity_m_s) ||
        !isfinite(feedback->right_velocity_m_s) ||
        !isfinite(feedback->roll_rad) ||
        !isfinite(feedback->roll_rate_rad_s) ||
        !isfinite(gains->length_kp_n_m) ||
        !isfinite(gains->length_kd_n_s_m) ||
        !isfinite(gains->roll_kp_n_rad) ||
        !isfinite(gains->roll_kd_n_s_rad) ||
        !isfinite(gains->gravity_feedforward_n) ||
        gains->length_kp_n_m < 0.0f || gains->length_kd_n_s_m < 0.0f ||
        gains->roll_kp_n_rad < 0.0f || gains->roll_kd_n_s_rad < 0.0f) {
        return false;
    }

    const double left_ref = (double)reference->common_length_m +
                            (double)reference->half_difference_m;
    const double right_ref = (double)reference->common_length_m -
                             (double)reference->half_difference_m;
    if (!isfinite(left_ref) || !isfinite(right_ref) ||
        left_ref <= 0.0 || right_ref <= 0.0 ||
        left_ref > FLT_MAX || right_ref > FLT_MAX) {
        return false;
    }

    /* Fleg=Kp*(Lref-L)-Kd*dL+Fgravity。
     * Froll=Kp_roll*(0-roll)-Kd_roll*droll。
     * 左右の力にそれぞれ+Froll/-Frollを加える。
     * 飽和クリップはここで行わない（ヤコビアン経由の各モータ値を優先）。 */
    const double roll = -(double)gains->roll_kp_n_rad * feedback->roll_rad -
                        (double)gains->roll_kd_n_s_rad * feedback->roll_rate_rad_s;
    const double left = (double)gains->length_kp_n_m *
                        (left_ref - feedback->left_length_m) -
                        (double)gains->length_kd_n_s_m * feedback->left_velocity_m_s +
                        (double)gains->gravity_feedforward_n + roll;
    const double right = (double)gains->length_kp_n_m *
                         (right_ref - feedback->right_length_m) -
                         (double)gains->length_kd_n_s_m * feedback->right_velocity_m_s +
                         (double)gains->gravity_feedforward_n - roll;
    if (!isfinite(roll) || !isfinite(left) || !isfinite(right) ||
        fabs(roll) > FLT_MAX || fabs(left) > FLT_MAX ||
        fabs(right) > FLT_MAX) {
        return false;
    }
    output->roll_differential_n = (float)roll;
    output->left_force_n = (float)left;
    output->right_force_n = (float)right;
    return true;
}
