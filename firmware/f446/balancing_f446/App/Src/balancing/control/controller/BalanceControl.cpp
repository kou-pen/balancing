#include "balancing/control/controller/BalanceControl.hpp"
#include "balancing/control/estimator/balancing_leg_kinematics.h"

#include <algorithm>
#include <cmath>

namespace balancing {
namespace {

BLegPdFeedback legFeedback(const ControlSnapshot& state) noexcept
{
    return {state.legs[0].mechanism.lengthM,
            state.legs[1].mechanism.lengthM,
            state.legs[0].mechanism.lengthRateMps,
            state.legs[1].mechanism.lengthRateMps,
            state.body.rollRad, state.body.rollRateRadS};
}

BControlState lqrState(const ControlSnapshot& state) noexcept
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

BLegKinematics kinematicsForVmc(const LegEstimate& source) noexcept
{
    // 旧C11の仮想仕事変換が参照するのはJの2行だけ。
    // 他の幾何を合成したかのように見せず、必要なヤコビアンだけコピーする。
    BLegKinematics mapped{};
    mapped.jacobian.dlength_dq_m[0] = source.lengthJacobianM[0];
    mapped.jacobian.dlength_dq_m[1] = source.lengthJacobianM[1];
    mapped.jacobian.dtheta_dq[0] = source.thetaJacobian[0];
    mapped.jacobian.dtheta_dq[1] = source.thetaJacobian[1];
    return mapped;
}

} // namespace

BalanceControl::BalanceControl(const BalanceControlConfig& config) noexcept : config_{config}
{
}

bool BalanceControl::initialize(const ControlSnapshot& snapshot) noexcept
{
    reset();
    const auto measured = legFeedback(snapshot);
    // ARM時の固定脚長目標は左右の現測定値から取得する(MOTOR-159)。
    // Roll/脚長PDのD項は実伸縮速度だけを使うので目標の飛びを持ち込まない。
    return b_leg_pd_capture(&legReference_, &measured, true);
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
    const BControlState target{
        0.0f, 0.0f, reference.positionM, reference.forwardVelocityMps, 0.0f, 0.0f
    };
    BControlTorque balance{};
    const auto state = lqrState(snapshot);
    if (!b_lqr_compute(&config_.lqr, &state, &target, &balance)) {
        reset();
        return std::nullopt;
    }

    const auto measured = legFeedback(snapshot);
    BLegPdOutput support{};
    if (!b_leg_pd_compute(&legReference_, &config_.legPd, &measured, &support)) {
        reset();
        return std::nullopt;
    }

    // 倒立共通成分を先に確保し、残る左右差動容量だけをYaw PIへ渡す。
    // 旧C11の優先配分・積分凍結規則を再利用し、同じ飽和式を二重実装しない。
    BWheelAllocation base{};
    if (!b_wheel_allocate(balance.wheel_total_nm, 0.0f,
                          config_.wheelTorqueLimitNm[0],
                          config_.wheelTorqueLimitNm[1], &base)) {
        reset();
        return std::nullopt;
    }
    const float availableYawNm = std::min(
        config_.wheelTorqueLimitNm[0] - std::fabs(base.base_each_nm),
        config_.wheelTorqueLimitNm[1] - std::fabs(base.base_each_nm));
    const BYawPiInput yawInput{
        reference.yawRateRadS, snapshot.body.yawRateRadS,
        static_cast<float>(snapshot.elapsed.count()) * 1.0e-6f,
        availableYawNm, true
    };
    BYawPiOutput yaw{};
    if (!b_yaw_pi_step(&yawState_, &config_.yawPi, &yawInput, &yaw)) {
        reset();
        return std::nullopt;
    }
    BWheelAllocation allocated{};
    if (!b_wheel_allocate(balance.wheel_total_nm, yaw.delta_nm,
                          config_.wheelTorqueLimitNm[0],
                          config_.wheelTorqueLimitNm[1], &allocated)) {
        reset();
        return std::nullopt;
    }

    // LQRは両脚合計の仮想トルクを返すので、初期の左右均等配分を適用。
    // 左右差を生むのは脚長/Rollの支持力だけ。実際の機構軸・モータ軸
    // 変換、ガスシリンダ、最終モータ制限はここで扱わない。
    std::array<float, 4> joints{};
    for (std::size_t leg = 0; leg < 2; ++leg) {
        const auto jacobian = kinematicsForVmc(snapshot.legs[leg].mechanism);
        const BLegVirtualLoad virtualLoad{
            leg == 0 ? support.left_force_n : support.right_force_n,
            balance.hip_total_nm * 0.5f
        };
        BLegTorquePair torque{};
        if (!b_leg_virtual_to_joint_torque(&jacobian, &virtualLoad, &torque)) {
            reset();
            return std::nullopt;
        }
        joints[2 * leg] = torque.torque_nm[0];
        joints[2 * leg + 1] = torque.torque_nm[1];
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
    b_yaw_pi_reset(&yawState_);
}

} // namespace balancing
