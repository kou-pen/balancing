#include "balancing/control/estimator/balancing_leg_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * これは実機六節リンクの確定モデルではなく、閉リンク演算の数学fixture。
 * CADと関節の接続・出力点・原点が一致するまで実機トルク指令へ使わない。
 */
#define B_LEG_PI 3.14159265358979323846f

/* 入力2軸のうち片側だけでもNaN/Infなら拘束式・ヤコビアンに伝播する。 */
static bool finite_pair(const float pair[2])
{
    return pair != NULL && isfinite(pair[0]) && isfinite(pair[1]);
}

/*
 * 符号は単なる倍率ではなく回転軸方向を定義するので±1だけ認める。
 * verified相当の校正フラグは外部から渡される契約であり、
 * 実機エンコーダのゼロ点・多回転の正しさをここで認定しない。
 */
static bool valid_motor_map(const BLegMotorMap *map)
{
    if (map == NULL || !map->calibration_valid ||
        !isfinite(map->driving_teeth) || !isfinite(map->driven_teeth) ||
        map->driving_teeth <= 0.0f || map->driven_teeth <= 0.0f) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (!isfinite(map->motor_zero_rad[i]) ||
            !isfinite(map->joint_offset_rad[i]) ||
            (map->sign[i] != 1.0f && map->sign[i] != -1.0f)) {
            return false;
        }
    }
    return true;
}

bool b_leg_motor_to_joint(const BLegMotorMap *map, const float motor_angle_rad[2],
                          const float motor_rate_rad_s[2], BLegJointState *joint)
{
    if (joint == NULL) {
        return false;
    }
    memset(joint, 0, sizeof(*joint));
    if (!valid_motor_map(map) || !finite_pair(motor_angle_rad) ||
        !finite_pair(motor_rate_rad_s)) {
        return false;
    }
    /* 外部伝達比は入力軸の角変位倍率。原点差と符号は軸ごとに適用する。
     * 差動速度には原点オフセットが寄与しない点に注意する。 */
    const float ratio = map->driving_teeth / map->driven_teeth;
    if (!isfinite(ratio) || ratio <= 0.0f) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        const float scale = ratio * map->sign[i];
        const float q = scale * (motor_angle_rad[i] - map->motor_zero_rad[i]) +
                        map->joint_offset_rad[i];
        const float qdot = scale * motor_rate_rad_s[i];
        if (!isfinite(q) || !isfinite(qdot)) {
            /* 途中の片軸だけ成功した状態を下流へ流さない。 */
            memset(joint, 0, sizeof(*joint));
            return false;
        }
        joint->q_rad[i] = q;
        joint->qdot_rad_s[i] = qdot;
    }
    return true;
}

/* 負長・ゼロ長・無効な特異閾値を早期拒否し、未確定CAD値を補完しない。 */
static bool valid_geometry(const BLegFixtureGeometry *g)
{
    return g != NULL && isfinite(g->base_span_m) &&
           isfinite(g->left_crank_m) && isfinite(g->right_crank_m) &&
           isfinite(g->left_rod_m) && isfinite(g->right_rod_m) &&
           isfinite(g->min_link_cross_sine) &&
           isfinite(g->min_leg_length_m) &&
           g->base_span_m > 0.0f && g->left_crank_m > 0.0f &&
           g->right_crank_m > 0.0f && g->left_rod_m > 0.0f &&
           g->right_rod_m > 0.0f && g->min_link_cross_sine > 0.0f &&
           g->min_link_cross_sine < 1.0f && g->min_leg_length_m > 0.0f;
}

BLegStatus b_leg_fixture_forward(const BLegFixtureGeometry *g,
                                 BLegAssemblyBranch branch,
                                 const BLegJointState *joint, BLegKinematics *out)
{
    if (out == NULL) {
        return B_LEG_INVALID_INPUT;
    }
    memset(out, 0, sizeof(*out));
    if (!valid_geometry(g) || joint == NULL ||
        (branch != B_LEG_BRANCH_NEGATIVE && branch != B_LEG_BRANCH_POSITIVE) ||
        !finite_pair(joint->q_rad) || !finite_pair(joint->qdot_rad_s)) {
        return B_LEG_INVALID_INPUT;
    }

    /*
     * 数学fixtureの平面内座標：左固定軸A=(0,0)、右固定軸B=(base_span,0)。
     * 入力角q0,q1からクランク先端E,Fを求める。E,Fを中心とした2円の
     * 交点Pを「仮想足先」と仮定するが、実機六節リンクの接続ではない。
     * 入力角を2π周期へ正規化し、大角入力で三角関数の誤差を増やさない。
     */
    const float q0 = remainderf(joint->q_rad[0], 2.0f * B_LEG_PI);
    const float q1 = remainderf(joint->q_rad[1], 2.0f * B_LEG_PI);
    const float ex = g->left_crank_m * cosf(q0);
    const float ey = g->left_crank_m * sinf(q0);
    const float fx = g->base_span_m + g->right_crank_m * cosf(q1);
    const float fy = g->right_crank_m * sinf(q1);
    const float dx = fx - ex;
    const float dy = fy - ey;
    const float d = hypotf(dx, dy);
    if (!isfinite(d) || d <= 0.0f) {
        return B_LEG_SINGULAR;
    }
    const float left = g->left_rod_m;
    const float right = g->right_rod_m;
    /* 2円の交差可能条件：|left-right| <= |F-E| <= left+right。
     * 交点が存在しない場合は近傍へ勝手に丸めず到達不能を返す。 */
    const float reach_max = left + right;
    const float reach_min = fabsf(left - right);
    if (d > reach_max || d < reach_min) {
        return B_LEG_UNREACHABLE;
    }

    /*
     * EからEF方向への射影長aと垂直距離hを円の幾何から算出。
     * P = E + a*unit(EF) ± h*perp(unit(EF))。branchの±は組立分岐で、
     * 同じ角度で別の脚姿勢を取り得るため自動切替をしない。
     */
    const float projection = (left * left - right * right + d * d) / (2.0f * d);
    float height_sq = left * left - projection * projection;
    if (!isfinite(height_sq)) {
        return B_LEG_INVALID_INPUT;
    }
    /*
     * h^2がわずかに負になる浮動小数点丸めだけをepsで吸収する。
     * 2円が接する幾何は特異なので、後段で外積により必ず拒否する。
     * epsは数学fixtureの数値許容差であり、実機の安全しきい値ではない。
     */
    const float eps = 16.0f * 1.1920928955078125e-7f * (left * left + right * right);
    if (height_sq < -eps) {
        return B_LEG_UNREACHABLE;
    }
    height_sq = fmaxf(0.0f, height_sq);
    const float h = sqrtf(height_sq);
    const float ux = dx / d;
    const float uy = dy / d;
    const float side = (float)branch;
    const float px = ex + projection * ux + side * h * (-uy);
    const float py = ey + projection * uy + side * h * ux;
    const float c0x = px - ex;
    const float c0y = py - ey;
    const float c1x = px - fx;
    const float c1y = py - fy;
    /*
     * 拘束勾配(PE,PF)を行とする2×2行列の行列式。
     * 2本の先端ロッドがほぼ平行なときdet→0となり逆行列が発散する。
     * |det|/(left*right)=|sin(ロッド間角)|で条件を判定し、特異を拒否する。
     */
    const float determinant = c0x * c1y - c0y * c1x;
    const float cross_sine = fabsf(determinant) / (left * right);
    if (!isfinite(cross_sine) || cross_sine < g->min_link_cross_sine) {
        return B_LEG_SINGULAR;
    }

    /* 脚原点は固定軸A,Bの中点。実際の機体座標系との対応は未確定。
     * 仮想脚長0では角度の微分が定義できないため別途拒否する。 */
    const float leg_x = px - g->base_span_m * 0.5f;
    const float leg_y = py;
    const float length = hypotf(leg_x, leg_y);
    if (!isfinite(length) || length < g->min_leg_length_m) {
        return B_LEG_SINGULAR;
    }
    /*
     * 円拘束 |P-E|^2=left^2 と |P-F|^2=right^2 を入力角で陰関数微分する。
     * (P-E)・dP=(P-E)・dE、(P-F)・dP=(P-F)・dF の線形2元系を解き、
     * 各モータ入力角に対するPの偏微分(dPx/dq, dPy/dq)を得る。
     * 先に行列式の特異判定を終えているので、ここでのみdetで除算する。
     */
    const float rhs0 = c0x * (-g->left_crank_m * sinf(q0)) +
                       c0y * (g->left_crank_m * cosf(q0));
    const float rhs1 = c1x * (-g->right_crank_m * sinf(q1)) +
                       c1y * (g->right_crank_m * cosf(q1));
    const float jpx[2] = {rhs0 * c1y / determinant, -c0y * rhs1 / determinant};
    const float jpy[2] = {-rhs0 * c1x / determinant, c0x * rhs1 / determinant};

    BLegKinematics value = {0};
    value.foot_x_m = leg_x;
    value.foot_y_m = leg_y;
    value.length_m = length;
    /* θ=atan2(脚x,-脚y)と定めることで数学fixtureでは下向きが0。
     * 実機のpitch軸・符号・ゼロ姿勢と一致するかはCAD校正が必要。 */
    value.theta_rad = atan2f(leg_x, -leg_y);
    const float inv_length = 1.0f / length;
    const float inv_length_sq = inv_length * inv_length;
    /*
     * dL/dq=(x*dPx/dq+y*dPy/dq)/L。
     * dθ/dq=(x*dPy/dq-y*dPx/dq)/L^2（atan2(x,-y)の微分）。
     * 2軸の速度は同じ解析Jとの積で求め、有限差分はテスト側でのみ使用。
     */
    for (unsigned i = 0; i < 2; ++i) {
        value.jacobian.dlength_dq_m[i] =
            (leg_x * jpx[i] + leg_y * jpy[i]) * inv_length;
        value.jacobian.dtheta_dq[i] =
            (leg_x * jpy[i] - leg_y * jpx[i]) * inv_length_sq;
        value.length_rate_m_s += value.jacobian.dlength_dq_m[i] * joint->qdot_rad_s[i];
        value.theta_rate_rad_s += value.jacobian.dtheta_dq[i] * joint->qdot_rad_s[i];
    }
    if (!isfinite(value.foot_x_m) || !isfinite(value.foot_y_m) ||
        !isfinite(value.length_m) || !isfinite(value.theta_rad) ||
        !isfinite(value.length_rate_m_s) || !isfinite(value.theta_rate_rad_s) ||
        !finite_pair(value.jacobian.dlength_dq_m) ||
        !finite_pair(value.jacobian.dtheta_dq)) {
        return B_LEG_INVALID_INPUT;
    }
    *out = value;
    return B_LEG_OK;
}

bool b_leg_virtual_to_joint_torque(const BLegKinematics *state,
                                   const BLegVirtualLoad *load,
                                   BLegTorquePair *torque)
{
    if (torque == NULL) {
        return false;
    }
    memset(torque, 0, sizeof(*torque));
    if (state == NULL || load == NULL ||
        !isfinite(load->axial_force_n) || !isfinite(load->pitch_torque_nm) ||
        !finite_pair(state->jacobian.dlength_dq_m) ||
        !finite_pair(state->jacobian.dtheta_dq)) {
        return false;
    }
    /* 仮想仕事の保存：τ_joint=J^T*[軸力, 仮想脚トルク]。
     * 実アクチュエータの上限、ガスシリンダ補償、摩擦は含まない。 */
    for (unsigned i = 0; i < 2; ++i) {
        const float value = load->axial_force_n * state->jacobian.dlength_dq_m[i] +
                            load->pitch_torque_nm * state->jacobian.dtheta_dq[i];
        if (!isfinite(value)) {
            memset(torque, 0, sizeof(*torque));
            return false;
        }
        torque->torque_nm[i] = value;
    }
    return true;
}

bool b_leg_joint_to_ideal_motor_torque(const BLegMotorMap *map,
                                       const BLegTorquePair *joint_torque,
                                       BLegTorquePair *motor_torque)
{
    if (motor_torque == NULL) {
        return false;
    }
    memset(motor_torque, 0, sizeof(*motor_torque));
    if (!valid_motor_map(map) || joint_torque == NULL ||
        !finite_pair(joint_torque->torque_nm)) {
        return false;
    }
    /* 理想歯車の入力仕事率保存：τ_motor = τ_joint * dq_joint/dq_motor。
     * 効率・バックラッシュは仮定していないため、安全なトルク指令ではない。 */
    const float ratio = map->driving_teeth / map->driven_teeth;
    for (unsigned i = 0; i < 2; ++i) {
        const float value = joint_torque->torque_nm[i] * map->sign[i] * ratio;
        if (!isfinite(value)) {
            memset(motor_torque, 0, sizeof(*motor_torque));
            return false;
        }
        motor_torque->torque_nm[i] = value;
    }
    return true;
}
