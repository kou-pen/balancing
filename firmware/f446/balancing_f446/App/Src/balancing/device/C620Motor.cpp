#include "balancing/device/C620Motor.hpp"

#include <limits>

namespace balancing {
namespace {
constexpr std::uint64_t feedbackTimeoutUs = 20000;
constexpr int encoderCounts = 8192;
constexpr int halfTurn = encoderCounts / 2;

constexpr bool canAdd(std::int64_t counts, int delta) noexcept
{
    return !(delta > 0 && counts > std::numeric_limits<std::int64_t>::max() - delta) &&
           !(delta < 0 && counts < std::numeric_limits<std::int64_t>::min() - delta);
}

// 実行時に到達困難な整数境界も、積算に使用する同じ式でコンパイル時検査する。
static_assert(canAdd(std::numeric_limits<std::int64_t>::max() - 1, 1));
static_assert(!canAdd(std::numeric_limits<std::int64_t>::max(), 1));
static_assert(canAdd(std::numeric_limits<std::int64_t>::min() + 1, -1));
static_assert(!canAdd(std::numeric_limits<std::int64_t>::min(), -1));
}

C620Motor::C620Motor(C620ContinuityConfig config) noexcept
    : config_{config},
      configValid_{config.verified && config.maxCaptureGapUs > 0 &&
                   config.maxCaptureGapUs <= feedbackTimeoutUs},
      continuity_{configValid_ ? C620ContinuityStatus::Waiting :
                                C620ContinuityStatus::Unverified}
{
}

void C620Motor::lose(C620ContinuityStatus reason) noexcept
{
    // 最初の喪失原因を保持する。正常フレームだけでは喪失した巻数を復元できない。
    if (continuity_ == C620ContinuityStatus::Waiting ||
        continuity_ == C620ContinuityStatus::Continuous) {
        continuity_ = reason;
    }
}

void C620Motor::invalidateContinuity() noexcept
{
    lose(C620ContinuityStatus::TransportLost);
}

void C620Motor::onFeedback(const BC620Feedback& report, std::uint64_t receivedAtUs) noexcept
{
    if (report.encoder_raw >= encoderCounts) {
        lose(C620ContinuityStatus::InvalidValue);
        return;
    }
    if (lastTimestamp_ && receivedAtUs < *lastTimestamp_) {
        lose(C620ContinuityStatus::ClockRegression);
        return;
    }
    // 明示リセット前の最後の時刻と同じ報告では新原点を作らない。
    // キュー自体の排除は上位の責務であり、時刻だけで排除を証明できない。
    if (!latest_ && lastTimestamp_ && receivedAtUs == *lastTimestamp_) {
        return;
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        lose(C620ContinuityStatus::CounterOverflow);
        return;
    }

    if (continuity_ == C620ContinuityStatus::Waiting) {
        counts_ = 0;
        continuity_ = C620ContinuityStatus::Continuous;
    } else if (continuity_ == C620ContinuityStatus::Continuous) {
        // µs取得時刻は同一IRQ内で重なる。受信順を使用し、時刻一致で捨てない。
        // 上限は半周未満の実測間変位を保証する外部条件であって、
        // 報告RPMだけから巻数を推定するものではない。
        if (receivedAtUs - latest_->receivedAtUs >= config_.maxCaptureGapUs) {
            lose(C620ContinuityStatus::GapExceeded);
        } else {
            int delta = static_cast<int>(report.encoder_raw) - latest_->report.encoder_raw;
            if (delta > halfTurn) delta -= encoderCounts;
            else if (delta < -halfTurn) delta += encoderCounts;
            if (delta == halfTurn || delta == -halfTurn) {
                lose(C620ContinuityStatus::AmbiguousDelta);
            } else if (!canAdd(counts_, delta)) {
                lose(C620ContinuityStatus::CounterOverflow);
            } else {
                counts_ += delta;
            }
        }
    }
    // 位置が失われても最新の通信診断値は保持できる。両者を混同しない。
    latest_ = C620FeedbackSample{report, receivedAtUs, ++sequence_};
    lastTimestamp_ = receivedAtUs;
}

C620AxisFeedback C620Motor::feedback(std::uint64_t nowUs) const noexcept
{
    const bool fresh = latest_ && latest_->receivedAtUs <= nowUs &&
                       nowUs - latest_->receivedAtUs < feedbackTimeoutUs;
    C620AxisFeedback result{latest_, fresh, {}, continuity_, generation_};
    if (latest_ && continuity_ == C620ContinuityStatus::Continuous) {
        result.position = C620EncoderPosition{counts_, sequence_, generation_};
    }
    return result;
}

bool C620Motor::resetReference() noexcept
{
    if (generation_ == std::numeric_limits<std::uint64_t>::max() ||
        sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        lose(C620ContinuityStatus::CounterOverflow);
        return false;
    }
    ++generation_;
    latest_.reset();
    counts_ = 0;
    continuity_ = configValid_ ? C620ContinuityStatus::Waiting :
                                 C620ContinuityStatus::Unverified;
    // 受理番号と最終時刻の高水位は保持し、旧世代と混同しない。
    return true;
}

} // namespace balancing
