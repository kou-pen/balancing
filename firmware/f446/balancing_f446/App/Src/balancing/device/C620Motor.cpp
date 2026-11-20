#include "balancing/device/C620Motor.hpp"

namespace balancing {
namespace {
constexpr std::uint64_t feedbackTimeoutUs = 20000;
}

void C620Motor::onFeedback(const BC620Feedback& report, std::uint64_t receivedAtUs) noexcept
{
    // CAN時刻はµs単位なので同時刻の報告は到着順に採用する。古い報告では現在値を巻き戻さない。
    if (!latest_ || latest_->receivedAtUs <= receivedAtUs) {
        latest_ = C620FeedbackSample{report, receivedAtUs};
    }
}

C620AxisFeedback C620Motor::feedback(std::uint64_t nowUs) const noexcept
{
    // 受信時刻から測るため、主ループで回収が遅れても報告の期限は延びない。
    const bool fresh = latest_ && latest_->receivedAtUs <= nowUs &&
                       nowUs - latest_->receivedAtUs < feedbackTimeoutUs;
    return {latest_, fresh};
}

} // namespace balancing
