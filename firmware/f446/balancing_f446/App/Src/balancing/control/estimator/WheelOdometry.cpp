#include "balancing/control/estimator/WheelOdometry.hpp"

#include <cmath>
#include <limits>

namespace balancing {
namespace {
constexpr int kEncoderCounts = 8192;
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
    std::size_t axis, const C620AxisFeedback& report, std::uint64_t sampledAtUs) noexcept
{
    if (!configValid_[axis]) {
        return WheelUpdateResult::InvalidConfiguration;
    }
    // 原点を勝手に変更しない。resetの前後で古いコピーを受け取っても
    // 前世代の位置を新しい原点として採用しない。
    if (generationExhausted_ || report.generation != generation_) {
        return WheelUpdateResult::ReferenceMismatch;
    }
    if (report.continuity != C620ContinuityStatus::Continuous) {
        if (report.continuity != C620ContinuityStatus::Waiting &&
            report.continuity != C620ContinuityStatus::Unverified) {
            // 喪失を観測した世代では、以前の有効コピーも復活させない。
            continuityLost_[axis] = true;
        }
        return report.continuity == C620ContinuityStatus::Waiting && !report.latest
                   ? WheelUpdateResult::NoSample : WheelUpdateResult::ContinuityUnavailable;
    }
    if (continuityLost_[axis] || !report.position || !report.latest) {
        return WheelUpdateResult::ContinuityUnavailable;
    }
    const auto& sample = *report.latest;
    const auto& position = *report.position;
    const auto& raw = sample.report;
    if (position.generation != generation_ || position.sequence != sample.sequence ||
        sample.sequence == 0 || sample.sequence < lastSequence_[axis]) {
        return WheelUpdateResult::InvalidSequence;
    }
    if (sample.receivedAtUs > sampledAtUs ||
        (last_[axis] && sample.receivedAtUs < last_[axis]->receivedAtUs)) {
        return WheelUpdateResult::InvalidTimestamp;
    }
    if (raw.encoderRaw >= kEncoderCounts) {
        return WheelUpdateResult::InvalidValue;
    }
    if (last_[axis] && sample.sequence == lastSequence_[axis]) {
        // 同じ受理番号のコピーを再読しただけなら、時刻や値を変更しない。
        if (sample.receivedAtUs != last_[axis]->receivedAtUs ||
            raw.encoderRaw != last_[axis]->encoderRaw || position.counts != lastCounts_[axis]) {
            return WheelUpdateResult::InvalidSequence;
        }
        return WheelUpdateResult::NoNewSample;
    }

    const auto& config = config_[axis];
    const double metersPerTurn =
        kTau * static_cast<double>(config.wheelRadiusM) /
        static_cast<double>(config.motorRevsPerWheelRev) *
        static_cast<double>(config.direction);
    // C620Motorは受信ごとに折返しを処理済み。ここで最新値との差を
    // 再積算すると中間報告の変位を失うため、累積整数を直接換算する。
    const double positionM = metersPerTurn * static_cast<double>(position.counts) / kEncoderCounts;
    const double velocityMps = metersPerTurn * static_cast<double>(raw.rpm) / 60.0;
    if (!std::isfinite(positionM) || !std::isfinite(velocityMps) ||
        std::abs(positionM) > std::numeric_limits<float>::max() ||
        std::abs(velocityMps) > std::numeric_limits<float>::max()) {
        return WheelUpdateResult::InvalidValue;
    }

    last_[axis] = WheelMotion{static_cast<float>(positionM),
                              static_cast<float>(velocityMps),
                              raw.encoderRaw, sample.receivedAtUs};
    lastSequence_[axis] = sample.sequence;
    lastCounts_[axis] = position.counts;
    return WheelUpdateResult::Updated;
}

std::array<WheelUpdateResult, 2> WheelOdometry::update(
    const MotorFeedbackSnapshot& motors) noexcept
{
    sampledAtUs_ = motors.sampledAtUs;
    std::array<WheelUpdateResult, 2> result{};
    for (std::size_t axis = 0; axis < result.size(); ++axis) {
        const auto& report = motors.c620[axis];
        result[axis] = accept(axis, report, motors.sampledAtUs);
        // 通信鮮度はC620Motorが所有する。今回の位置照合にも成功した場合だけ
        // 物理観測を有効にし、古い診断値を制御入力へ戻さない。
        fresh_[axis] = report.fresh &&
                       (result[axis] == WheelUpdateResult::Updated ||
                        result[axis] == WheelUpdateResult::NoNewSample);
    }
    return result;
}

WheelOdometrySnapshot WheelOdometry::snapshot() const noexcept
{
    return {sampledAtUs_, {{{last_[0], fresh_[0]}, {last_[1], fresh_[1]}}}};
}

std::optional<std::uint64_t> WheelOdometry::resettableGeneration() const noexcept
{
    if (!configValid_[0] || !configValid_[1] || generationExhausted_ ||
        generation_==std::numeric_limits<std::uint64_t>::max()) return std::nullopt;
    return generation_;
}

void WheelOdometry::reset() noexcept
{
    if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
        generationExhausted_ = true;
    } else {
        ++generation_;
    }
    last_ = {};
    fresh_ = {};
    lastSequence_ = {};
    lastCounts_ = {};
    continuityLost_ = {};
    sampledAtUs_ = 0;
}

} // namespace balancing
