#include "balancing/control/estimator/WheelOdometry.hpp"

#include <cmath>
#include <limits>

namespace balancing {
namespace {
constexpr int kEncoderCounts = 8192;
constexpr int kHalfTurnCounts = kEncoderCounts / 2;
constexpr double kTau = 6.2831853071795864769;
}

bool WheelOdometry::validConfig(const WheelOdometryConfig& config) noexcept
{
    // 設定は起動時に固定するため、受信周期ごとには再検査しない。
    return config.verified && std::isfinite(config.wheelRadiusM) &&
           std::isfinite(config.motorRevsPerWheelRev) &&
           config.wheelRadiusM > 0.0f && config.motorRevsPerWheelRev > 0.0f &&
           (config.direction == 1.0f || config.direction == -1.0f);
}

WheelOdometry::WheelOdometry(const std::array<WheelOdometryConfig, 2>& config) noexcept
    : config_{config},
      configValid_{{validConfig(config[0]), validConfig(config[1])}}
{
}

WheelUpdateResult WheelOdometry::accept(
    std::size_t axis, const C620FeedbackSample& sample, std::uint64_t sampledAtUs) noexcept
{
    const auto& raw = sample.report;
    if (!configValid_[axis]) {
        return WheelUpdateResult::InvalidConfiguration;
    }
    if (sample.receivedAtUs > sampledAtUs ||
        (last_[axis] && sample.receivedAtUs < last_[axis]->receivedAtUs)) {
        return WheelUpdateResult::InvalidTimestamp;
    }
    if (last_[axis] && sample.receivedAtUs == last_[axis]->receivedAtUs) {
        // 500 Hzループで同一報告を何度読んでも、位置を二重積算しない。
        return WheelUpdateResult::NoNewSample;
    }
    if (raw.encoder_raw >= kEncoderCounts) {
        return WheelUpdateResult::InvalidValue;
    }

    int counts = 0;
    if (last_[axis]) {
        counts = static_cast<int>(raw.encoder_raw) - last_[axis]->encoderRaw;
        if (counts > kHalfTurnCounts) {
            counts -= kEncoderCounts;
        } else if (counts < -kHalfTurnCounts) {
            counts += kEncoderCounts;
        }
        // 正転と逆転の両方が成り立つ半回転差では、適当な回転方向を採用しない。
        if (counts == kHalfTurnCounts || counts == -kHalfTurnCounts) {
            return WheelUpdateResult::AmbiguousDelta;
        }
    }

    const auto& config = config_[axis];
    // 旧C11のb_observation_wheelと同じ計算。位置は初回報告を0mとし、
    // 受信間のロータ回転数が半周未満という外部検証前提の下でだけ最短差を積算する。
    const double metersPerTurn =
        kTau * static_cast<double>(config.wheelRadiusM) /
        static_cast<double>(config.motorRevsPerWheelRev) *
        static_cast<double>(config.direction);
    const double positionM =
        (last_[axis] ? static_cast<double>(last_[axis]->positionM) : 0.0) +
        metersPerTurn * static_cast<double>(counts) / kEncoderCounts;
    const double velocityMps = metersPerTurn * static_cast<double>(raw.rpm) / 60.0;
    if (!std::isfinite(positionM) || !std::isfinite(velocityMps) ||
        std::abs(positionM) > std::numeric_limits<float>::max() ||
        std::abs(velocityMps) > std::numeric_limits<float>::max()) {
        return WheelUpdateResult::InvalidValue;
    }

    // 完全に計算できた軸のみ確定する。他方の観測・時刻は触らない。
    last_[axis] = WheelMotion{static_cast<float>(positionM),
                              static_cast<float>(velocityMps),
                              raw.encoder_raw, sample.receivedAtUs};
    return WheelUpdateResult::Updated;
}

std::array<WheelUpdateResult, 2> WheelOdometry::update(
    const MotorFeedbackSnapshot& motors) noexcept
{
    sampledAtUs_ = motors.sampledAtUs;
    std::array<WheelUpdateResult, 2> result{};
    for (std::size_t axis = 0; axis < result.size(); ++axis) {
        const auto& report = motors.c620[axis];
        result[axis] = report.latest
                           ? accept(axis, *report.latest, motors.sampledAtUs)
                           : WheelUpdateResult::NoSample;
        // C620の20ms鮮度はMotorServiceのみが判断する。
        // 変換に失敗した新しい報告で古い位置だけをfresh扱いしないため、
        // 最新CAN時刻と最後に採用した時刻が一致することも要求する。
        fresh_[axis] = report.fresh && last_[axis] && report.latest &&
                       last_[axis]->receivedAtUs == report.latest->receivedAtUs;
    }
    return result;
}

WheelOdometrySnapshot WheelOdometry::snapshot() const noexcept
{
    return {sampledAtUs_, {{{last_[0], fresh_[0]}, {last_[1], fresh_[1]}}}};
}

void WheelOdometry::reset() noexcept
{
    last_ = {};
    fresh_ = {};
    sampledAtUs_ = 0;
}

} // namespace balancing
