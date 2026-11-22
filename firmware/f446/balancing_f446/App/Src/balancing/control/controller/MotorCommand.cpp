#include "balancing/control/controller/MotorCommand.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace balancing {
namespace {

// 以下はCANプロトコルの符号化表現範囲に限る。実機の許容値を決める根拠ではない。
constexpr float kRs06ProtocolMaxNm = 36.0f;
constexpr float kC620ProtocolMaxA = 20.0f;

bool signedDirection(float direction) noexcept
{
    return direction == 1.0f || direction == -1.0f;
}

MotorCommandPreview failedPreview(CommandStatus status) noexcept
{
    MotorCommandPreview preview{};
    preview.status = status;
    return preview;
}

} // namespace

bool MotorCommand::valid(const Rs06TorqueConversion& axis) noexcept
{
    // 設定の数値検査は構築時にまとめ、500Hz制御演算では繰り返さない。
    // メーカー表現上限内であっても、verified=falseなら常に拒否する。
    return axis.verified && std::isfinite(axis.motorRevsPerJointRev) &&
           axis.motorRevsPerJointRev > 0.0f && signedDirection(axis.motorDirection) &&
           std::isfinite(axis.maxAbsMotorTorqueNm) &&
           axis.maxAbsMotorTorqueNm > 0.0f &&
           axis.maxAbsMotorTorqueNm <= kRs06ProtocolMaxNm;
}

bool MotorCommand::valid(const C620CurrentConversion& axis) noexcept
{
    return axis.verified && std::isfinite(axis.wheelTorqueNmPerAmp) &&
           axis.wheelTorqueNmPerAmp > 0.0f && signedDirection(axis.currentDirection) &&
           std::isfinite(axis.maxAbsCurrentA) &&
           axis.maxAbsCurrentA > 0.0f && axis.maxAbsCurrentA <= kC620ProtocolMaxA;
}

MotorCommand::MotorCommand(const MotorCommandConfig& config) noexcept
    : config_{config}, configValid_{true}
{
    for (const auto& axis : config_.rs06) {
        configValid_ = configValid_ && valid(axis);
    }
    for (const auto& axis : config_.c620) {
        configValid_ = configValid_ && valid(axis);
    }
}

MotorCommandPreview MotorCommand::evaluate(const BalanceControlOutput& input) const noexcept
{
    MotorCommandPreview result{};
    if (!configValid_) {
        result.status = CommandStatus::UnverifiedConfiguration;
        return result;
    }
    if (!input.fixtureOnly) {
        // このAPIは数学fixtureの比較だけに使用する。
        // 上流のフラグを変えて非ゼロ値を「実機用」に昇格させない。
        result.status = CommandStatus::InvalidInput;
        return result;
    }

    // 一部の軸を計算し終えた後に別軸のNaN/Infを見つけても、部分指令を
    // 発行しない。戻すのはすべてゼロの局所resultのみ。
    for (const auto torque : input.jointTorqueNm) {
        if (!std::isfinite(torque)) {
            result.status = CommandStatus::InvalidInput;
            return result;
        }
    }
    for (const auto torque : input.wheelTorqueNm) {
        if (!std::isfinite(torque)) {
            result.status = CommandStatus::InvalidInput;
            return result;
        }
    }

    // 順運動学で q_joint = sign*q_motor / ratio のとき、
    // 仮想仕事の保存 τ_motor*dq_motor = τ_joint*dq_joint より
    // τ_motor = sign*τ_joint / ratio。符号を二度反転しない。
    for (std::size_t axis = 0; axis < config_.rs06.size(); ++axis) {
        const auto& calibration = config_.rs06[axis];
        const double requested =
            static_cast<double>(input.jointTorqueNm[axis]) *
            static_cast<double>(calibration.motorDirection) /
            static_cast<double>(calibration.motorRevsPerJointRev);
        if (!std::isfinite(requested) ||
            std::abs(requested) > std::numeric_limits<float>::max()) {
            // 先行軸の計算値を部分的に外へ出さない。
            return failedPreview(CommandStatus::ConversionFailed);
        }
        const double limited = std::clamp(
            requested,
            -static_cast<double>(calibration.maxAbsMotorTorqueNm),
            static_cast<double>(calibration.maxAbsMotorTorqueNm));
        result.rs06[axis] = {
            static_cast<float>(requested),
            static_cast<float>(limited),
            limited != requested
        };
    }

    // wheelTorqueNmPerAmpにはモータ定数・減速機・損失係数をまとめて含める。
    // この境界でギヤ比を別途乗除すると二重計上になる。
    for (std::size_t axis = 0; axis < config_.c620.size(); ++axis) {
        const auto& calibration = config_.c620[axis];
        const double requested =
            static_cast<double>(input.wheelTorqueNm[axis]) *
            static_cast<double>(calibration.currentDirection) /
            static_cast<double>(calibration.wheelTorqueNmPerAmp);
        if (!std::isfinite(requested) ||
            std::abs(requested) > std::numeric_limits<float>::max()) {
            return failedPreview(CommandStatus::ConversionFailed);
        }
        const double limited = std::clamp(
            requested,
            -static_cast<double>(calibration.maxAbsCurrentA),
            static_cast<double>(calibration.maxAbsCurrentA));
        result.c620[axis] = {
            static_cast<float>(requested),
            static_cast<float>(limited),
            limited != requested
        };
    }

    // 上流のYaw配分での飽和と、この層の各軸最終制限を混同しない。
    result.upstreamWheelSaturated = input.wheelSaturated;
    const auto anySaturated = [](const auto& axis) { return axis.saturated; };
    const bool saturated =
        std::any_of(result.rs06.begin(), result.rs06.end(), anySaturated) ||
        std::any_of(result.c620.begin(), result.c620.end(), anySaturated);
    result.status = saturated ? CommandStatus::Saturated : CommandStatus::Ready;
    return result;
}

} // namespace balancing
