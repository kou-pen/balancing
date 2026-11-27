#include "balancing/control/controller/BalanceControl.hpp"
#include "balancing/control/estimator/LegKinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace balancing {
namespace balance_math {
/*
 * 2輪の合計バランストルクをまず左右同量に分け、そのあとYaw差動を配分する。
 * 左右個別の上限が異なるときは小さい側を共通成分の上限とし、
 * 残容量の範囲でのみYawを入れるため、姿勢保持を優先する。
 */
bool allocateWheels(float total_balance_nm, float yaw_delta_nm,
                      float left_limit_nm, float right_limit_nm,
                      WheelAllocation& result) noexcept
{
    float base_request;
    float base_limit;
    float base;
    float yaw_limit;
    float yaw;

    result = {};
    if (!std::isfinite(total_balance_nm) || !std::isfinite(yaw_delta_nm) ||
        !std::isfinite(left_limit_nm) || !std::isfinite(right_limit_nm) ||
        left_limit_nm <= 0.0f || right_limit_nm <= 0.0f) {
        return false;
    }

    base_request = total_balance_nm / 2.0f;
    base_limit = std::fmin(left_limit_nm, right_limit_nm);
    base = std::fmax(-base_limit, std::fmin(base_limit, base_request));
    /* 共通成分を使った後の左右の余剰トルクだけをYawへ割り当てる。
     * 飽和は正常な制限動作であって、それだけではFAULTにしない。 */
    yaw_limit = std::fmin(left_limit_nm - std::fabs(base),
                      right_limit_nm - std::fabs(base));
    yaw = std::fmax(-yaw_limit, std::fmin(yaw_limit, yaw_delta_nm));

    result.base_each_nm = base;
    result.yaw_applied_nm = yaw;
    result.left_nm = base - yaw;
    result.right_nm = base + yaw;
    result.saturated = (base != base_request) || (yaw != yaw_delta_nm);
    return true;
}

/*
 * 6状態誤差と2×6ゲインの純粋な行列積。配置順を構造体レイアウトに
 * 依存させず、状態・目標の単位付きフィールドから明示的に転記する。
 * 状態/ゲイン/平衡入力を外部で検証するまでARM条件は満たされない。
 */
bool computeLqr(const LqrConfig& config, const LqrState& state,
                   const LqrState& target, LqrTorque& torque) noexcept
{
    float current[LqrStateCount];
    float desired[LqrStateCount];
    float error[LqrStateCount];
    float result[LqrOutputCount];
    unsigned i;
    unsigned j;

    torque = {};

    if (!config.verified || !std::isfinite(config.max_position_error_m) ||
        config.max_position_error_m <= 0.0f) {
        return false;
    }

    /* 変換はこの境界に集約し、構造体のメモリ配置に依存しない。 */
    current[LqrTheta] = state.theta_rad;
    current[LqrThetaRate] = state.theta_rate_rad_s;
    current[LqrPosition] = state.position_m;
    current[LqrVelocity] = state.velocity_m_s;
    current[LqrPitch] = state.pitch_rad;
    current[LqrPitchRate] = state.pitch_rate_rad_s;

    desired[LqrTheta] = target.theta_rad;
    desired[LqrThetaRate] = target.theta_rate_rad_s;
    desired[LqrPosition] = target.position_m;
    desired[LqrVelocity] = target.velocity_m_s;
    desired[LqrPitch] = target.pitch_rad;
    desired[LqrPitchRate] = target.pitch_rate_rad_s;

    for (i = 0; i < LqrStateCount; ++i) {
        if (!std::isfinite(current[i]) || !std::isfinite(desired[i])) {
            return false;
        }
        error[i] = current[i] - desired[i];
        if (!std::isfinite(error[i])) {
            return false;
        }
    }
    /* 車輪の積算位置が走行で無制限に離れても、その誤差だけで
     * 必要以上のトルクを要求しないよう位置成分のみに制限を掛ける。 */
    error[LqrPosition] =
        std::fmax(-config.max_position_error_m,
              std::fmin(config.max_position_error_m, error[LqrPosition]));

    /* u = u_eq - K(x - x_ref)。
     * doubleで積を蓄積しfloat出力へ戻す前にオーバーフローを拒否する。
     * これは行列演算の正しさであって、実機のKが安全という意味ではない。 */
    for (j = 0; j < LqrOutputCount; ++j) {
        double value = config.equilibrium[j];
        if (!std::isfinite(value)) {
            return false;
        }
        for (i = 0; i < LqrStateCount; ++i) {
            if (!std::isfinite(config.gain[j][i])) {
                return false;
            }
            value -= static_cast<double>(config.gain[j][i]) * error[i];
        }
        if (!std::isfinite(value) || std::fabs(value) > std::numeric_limits<float>::max()) {
            return false;
        }
        result[j] = static_cast<float>(value);
    }

    torque.wheel_total_nm = result[LqrWheelTotal];
    torque.hip_total_nm = result[LqrHipTotal];
    return true;
}

/*
 * 絶対的な脚長安全範囲はCADと実測で未決定なので、ここでは定義しない。
 * 負または0の測定長、NaN/Infだけを数学的に無効とする。
 */
static bool measuredLengthsOk(const LegPdFeedback& feedback) noexcept
{
    return std::isfinite(feedback.left_length_m) &&
           std::isfinite(feedback.right_length_m) &&
           feedback.left_length_m > 0.0f && feedback.right_length_m > 0.0f;
}

/*
 * この操作はARM遷移直前、上位安全層で左右RS06の鮮度を判定し、
 * 観測スナップショットを確定した後に限る。新規時刻の判定を重複実装しない。
 * 途中で失敗した場合に前回の脚長目標を再利用させないため、先に無効化する。
 * 差分を半分として持てば初期目標は左右の測定長を正確に再現できる。
 */
bool captureLegReference(LegPdReference& reference, const LegPdFeedback& feedback,
                      bool disarmed) noexcept
{
    reference = {};
    if (!disarmed || !measuredLengthsOk(feedback)) {
        return false;
    }

    /* 中間結果をdoubleで計算し、floatへ戻す際の範囲逸脱を拒否する。
     * 各脚長そのものの有効性と、平均/差が表現可能かは別の検査。 */
    const double common = (static_cast<double>(feedback.left_length_m) +
                           static_cast<double>(feedback.right_length_m)) * 0.5;
    const double delta = (static_cast<double>(feedback.left_length_m) -
                          static_cast<double>(feedback.right_length_m)) * 0.5;
    if (!std::isfinite(common) || !std::isfinite(delta) || common <= 0.0 ||
        common > std::numeric_limits<float>::max() || std::fabs(delta) > std::numeric_limits<float>::max()) {
        return false;
    }
    reference.common_length_m = static_cast<float>(common);
    reference.half_difference_m = static_cast<float>(delta);
    reference.valid = true;
    return true;
}

/*
 * D成分は目標値の微分を用いない。ARM直後に参照値をラッチしたときの
 * 微分キックを避け、実際のRS06速度から計算した伸縮速度のみを使う。
 * Rollは水平(0 rad)を目標にし、左+・右-の抽象差動と定義する。
 * 実機の力方向/符号は未検証なので、CANへは一切接続しない。
 */
bool computeLegSupport(const LegPdReference& reference, const LegPdGains& gains,
                      const LegPdFeedback& feedback, LegPdOutput& output) noexcept
{
    output = {};
    if (!reference.valid ||
        !gains.verified || !measuredLengthsOk(feedback) ||
        !std::isfinite(reference.common_length_m) ||
        !std::isfinite(reference.half_difference_m) ||
        reference.common_length_m <= 0.0f ||
        !std::isfinite(feedback.left_velocity_m_s) ||
        !std::isfinite(feedback.right_velocity_m_s) ||
        !std::isfinite(feedback.roll_rad) ||
        !std::isfinite(feedback.roll_rate_rad_s) ||
        !std::isfinite(gains.length_kp_n_m) ||
        !std::isfinite(gains.length_kd_n_s_m) ||
        !std::isfinite(gains.roll_kp_n_rad) ||
        !std::isfinite(gains.roll_kd_n_s_rad) ||
        !std::isfinite(gains.gravity_feedforward_n) ||
        gains.length_kp_n_m < 0.0f || gains.length_kd_n_s_m < 0.0f ||
        gains.roll_kp_n_rad < 0.0f || gains.roll_kd_n_s_rad < 0.0f) {
        return false;
    }

    const double left_ref = static_cast<double>(reference.common_length_m) +
                            static_cast<double>(reference.half_difference_m);
    const double right_ref = static_cast<double>(reference.common_length_m) -
                             static_cast<double>(reference.half_difference_m);
    if (!std::isfinite(left_ref) || !std::isfinite(right_ref) ||
        left_ref <= 0.0 || right_ref <= 0.0 ||
        left_ref > std::numeric_limits<float>::max() || right_ref > std::numeric_limits<float>::max()) {
        return false;
    }

    /* Fleg=Kp*(Lref-L)-Kd*dL+Fgravity。
     * Froll=Kp_roll*(0-roll)-Kd_roll*droll。
     * 左右の力にそれぞれ+Froll/-Frollを加える。
     * 飽和クリップはここで行わない（ヤコビアン経由の各モータ値を優先）。 */
    const double roll = -static_cast<double>(gains.roll_kp_n_rad) * feedback.roll_rad -
                        static_cast<double>(gains.roll_kd_n_s_rad) * feedback.roll_rate_rad_s;
    const double left = static_cast<double>(gains.length_kp_n_m) *
                        (left_ref - feedback.left_length_m) -
                        static_cast<double>(gains.length_kd_n_s_m) * feedback.left_velocity_m_s +
                        static_cast<double>(gains.gravity_feedforward_n) + roll;
    const double right = static_cast<double>(gains.length_kp_n_m) *
                         (right_ref - feedback.right_length_m) -
                         static_cast<double>(gains.length_kd_n_s_m) * feedback.right_velocity_m_s +
                         static_cast<double>(gains.gravity_feedforward_n) - roll;
    if (!std::isfinite(roll) || !std::isfinite(left) || !std::isfinite(right) ||
        std::fabs(roll) > std::numeric_limits<float>::max() || std::fabs(left) > std::numeric_limits<float>::max() ||
        std::fabs(right) > std::numeric_limits<float>::max()) {
        return false;
    }
    output.roll_differential_n = static_cast<float>(roll);
    output.left_force_n = static_cast<float>(left);
    output.right_force_n = static_cast<float>(right);
    return true;
}

/*
 * 積分をセッション間で持ち越すと、再ARM時に停止中の誤差だけで
 * 急なYaw差動が生じる。FAULT/モード変更の責務をこの状態に閉じ込める。
 */
void resetYaw(YawPiState& state) noexcept
{
    state = {};
}

/*
 * 単位は e [rad/s], dt [s], Ki [N m/rad] なので
 * 積分増分 Ki*e*dt は [N m]。差動指令ΔTはallocateWheelsで
 * 左側から減算、右側へ加算される（車輪順序を逆にしない）。
 *
 * 積分器に出力上限をそのまま適用しない。積分自体の上限と
 * 現在の車輪余力は異なり、後者はLQR倒立トルクにより変化する。
 * 候補積分が現在の飽和を悪化させるときだけ候補を棄却し、
 * 誤差反転による積分解消は許す。外部の配分器でも最終クリップが必要。
 */
bool stepYaw(YawPiState& state, const YawPiGains& gains,
                   const YawPiInput& input, YawPiOutput& output) noexcept
{
    output = {};

    /* 鮮度・通信断の判断は安全状態機械に集約する。
     * 本層はBALANCING以外で差動を出さず積分を持ち越さない。
     * 安全層が判断を終えた後に、制御周期ごとに本関数を呼ぶ。 */
    if (!input.balancing) {
        resetYaw(state);
        return true;
    }
    if (!gains.verified ||
        !std::isfinite(gains.kp_nm_s_rad) || gains.kp_nm_s_rad < 0.0f ||
        !std::isfinite(gains.ki_nm_rad) || gains.ki_nm_rad < 0.0f ||
        !std::isfinite(gains.integral_limit_nm) || gains.integral_limit_nm < 0.0f ||
        !std::isfinite(input.target_rate_rad_s) ||
        !std::isfinite(input.measured_rate_rad_s) ||
        !std::isfinite(input.dt_s) || input.dt_s <= 0.0f ||
        !std::isfinite(input.available_delta_nm) ||
        input.available_delta_nm < 0.0f ||
        !std::isfinite(state.integral_nm)) {
        resetYaw(state);
        return false;
    }

    /* BALANCING初回は初期化状態から計算。明示resetを忘れた
     * アクティブでないセッションの過去積分も採用しない。 */
    const double old_i = state.active ? state.integral_nm : 0.0;
    const double error = static_cast<double>(input.target_rate_rad_s) -
                         static_cast<double>(input.measured_rate_rad_s);
    const double p = static_cast<double>(gains.kp_nm_s_rad) * error;
    const double change = static_cast<double>(gains.ki_nm_rad) * error * input.dt_s;
    const double i_max = gains.integral_limit_nm;
    const double capacity = input.available_delta_nm;
    double next_i = std::fmax(-i_max, std::fmin(i_max, old_i + change));

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
    const double applied = std::fmax(-capacity, std::fmin(capacity, raw));
    if (!std::isfinite(error) || !std::isfinite(p) || !std::isfinite(change) ||
        !std::isfinite(raw) || !std::isfinite(applied) ||
        std::fabs(next_i) > std::numeric_limits<float>::max() || std::fabs(applied) > std::numeric_limits<float>::max()) {
        resetYaw(state);
        return false;
    }
    state.integral_nm = static_cast<float>(next_i);
    state.active = true;
    output.delta_nm = static_cast<float>(applied);
    output.integral_nm = state.integral_nm;
    output.saturated = raw != applied;
    output.integral_frozen = freeze;
    return true;
}

} // namespace balance_math
namespace {

LegPdFeedback legFeedback(const ControlSnapshot& state) noexcept
{
    return {state.legs[0].mechanism.lengthM,
            state.legs[1].mechanism.lengthM,
            state.legs[0].mechanism.lengthRateMps,
            state.legs[1].mechanism.lengthRateMps,
            state.body.rollRad, state.body.rollRateRadS};
}

LqrState lqrState(const ControlSnapshot& state) noexcept
{
    // LQRの仮想脚θは左右平均、機体Pitchは別状態φ。
    // 脚の機体相対θにPitchを加えた鉛直側の角を使い、取り付け軸の
    // 実機符号と座標系は後続のCAD検証に委ねる。
    const auto& left = state.legs[0];
    const auto& right = state.legs[1];
    return {
        0.5f * (left.thetaVerticalRad + right.thetaVerticalRad),
        0.5f * (left.mechanism.thetaRateRadS + right.mechanism.thetaRateRadS) +
            state.body.pitchRateRadS,
        state.chassis.positionM, state.chassis.velocityMps,
        state.body.pitchRad, state.body.pitchRateRadS
    };
}

} // namespace

BalanceControl::BalanceControl(const BalanceControlConfig& config) noexcept : config_{config}
{
}

bool BalanceControl::initialize(const ControlSnapshot& snapshot) noexcept
{
    reset();
    // 装着済みなのに計算モデルが未検証なら、ARM前の初期化を拒否する。
    // 未装着では関数やverifiedの値に関係なく従来の計算を維持する。
    const auto& gas = config_.gasCylinder;
    if (gas.installed && (!gas.verified || !gas.evaluate)) {
        return false;
    }
    const auto measured = legFeedback(snapshot);
    // ARM時の固定脚長目標は左右の現測定値から取得する(MOTOR-159)。
    // Roll/脚長PDのD項は実伸縮速度だけを使うので目標の飛びを持ち込まない。
    return balance_math::captureLegReference(legReference_, measured, true);
}

std::optional<BalanceControlOutput> BalanceControl::compute(
    const ControlSnapshot& snapshot, const BalanceReference& reference) noexcept
{
    if (!legReference_.valid || snapshot.elapsed.count() <= 0) {
        reset();
        return std::nullopt;
    }

    // 参照生成器はSILの外。固定位置へ戻るか移動基準を使うかを
    // 勝手に決定しないため、callerが渡した位置・速度をそのまま使う。
    const LqrState target{
        0.0f, 0.0f, reference.positionM, reference.forwardVelocityMps, 0.0f, 0.0f
    };
    LqrTorque balance{};
    const auto state = lqrState(snapshot);
    if (!balance_math::computeLqr(config_.lqr, state, target, balance)) {
        reset();
        return std::nullopt;
    }

    const auto measured = legFeedback(snapshot);
    LegPdOutput support{};
    if (!balance_math::computeLegSupport(legReference_, config_.legPd, measured, support)) {
        reset();
        return std::nullopt;
    }

    // 倒立共通成分を先に確保し、残る左右差動容量だけをYaw PIへ渡す。
    // 既存の優先配分・積分凍結規則を再利用し、同じ飽和式を二重実装しない。
    WheelAllocation base{};
    if (!balance_math::allocateWheels(balance.wheel_total_nm, 0.0f,
                          config_.wheelTorqueLimitNm[0],
                          config_.wheelTorqueLimitNm[1], base)) {
        reset();
        return std::nullopt;
    }
    const float availableYawNm = std::min(
        config_.wheelTorqueLimitNm[0] - std::fabs(base.base_each_nm),
        config_.wheelTorqueLimitNm[1] - std::fabs(base.base_each_nm));
    const YawPiInput yawInput{
        reference.yawRateRadS, snapshot.body.yawRateRadS,
        static_cast<float>(snapshot.elapsed.count()) * 1.0e-6f,
        availableYawNm, true
    };
    YawPiOutput yaw{};
    if (!balance_math::stepYaw(yawState_, config_.yawPi, yawInput, yaw)) {
        reset();
        return std::nullopt;
    }
    WheelAllocation allocated{};
    if (!balance_math::allocateWheels(balance.wheel_total_nm, yaw.delta_nm,
                          config_.wheelTorqueLimitNm[0],
                          config_.wheelTorqueLimitNm[1], allocated)) {
        reset();
        return std::nullopt;
    }

    // LQRは両脚合計の仮想トルクを返すので、初期の左右均等配分を適用。
    // VMCでは脚長/Roll支持力と均等な仮想トルクを機構軸へ変換する。
    // モータ軸への変換・最終制限はMotorCommandで行う。
    std::array<float, 4> joints{};
    for (std::size_t leg = 0; leg < 2; ++leg) {
        std::array<float,2> torque{};
        const float forceN=leg==0?support.left_force_n:support.right_force_n;
        // StateEstimatorの解析Jをそのまま渡し、旧Cの中間構造体へコピーしない。
        if (!LegKinematics::virtualToJointTorque(snapshot.legs[leg].mechanism,
                forceN,balance.hip_total_nm*0.5f,torque)) {
            reset();
            return std::nullopt;
        }
        joints[2*leg]=torque[0];
        joints[2*leg+1]=torque[1];
    }
    // 補助力は同じ機構座標のトルクへ換算済みで受け取る。
    // VMC後に一度だけ差し引き、モータ軸変換や最終制限を重複させない。
    const auto& gas = config_.gasCylinder;
    if (gas.installed) {
        // 全軸の明示出力を要求し、書き忘れを補償ゼロと誤認しない。
        std::array<float,4> assist{};
        assist.fill(std::numeric_limits<float>::quiet_NaN());
        if (!gas.evaluate(snapshot, assist)) {
            reset();
            return std::nullopt;
        }
        for (std::size_t axis = 0; axis < joints.size(); ++axis) {
            // doubleで差を検査し、float範囲外を飽和として通さない。
            const double corrected = static_cast<double>(joints[axis]) - assist[axis];
            if (!std::isfinite(assist[axis]) || !std::isfinite(corrected) ||
                std::fabs(corrected) > std::numeric_limits<float>::max()) {
                reset();
                return std::nullopt;
            }
            joints[axis] = static_cast<float>(corrected);
        }
    }
    // 出力は機構軸の数学値であり、検証済みモータ指令には変換しない。
    return BalanceControlOutput{
        joints, {allocated.left_nm, allocated.right_nm},
        balance.wheel_total_nm, allocated.yaw_applied_nm,
        base.saturated || yaw.saturated || allocated.saturated,
        yaw.integral_frozen, true
    };
}

void BalanceControl::reset() noexcept
{
    legReference_ = {};
    balance_math::resetYaw(yawState_);
}

} // namespace balancing
