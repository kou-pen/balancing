#include "balancing/device/Rs06Motor.hpp"

#include <limits>

namespace balancing {
namespace {
constexpr std::uint64_t feedbackTimeoutUs = 30000;
}

Rs06Motor::Rs06Motor(std::uint8_t motorId, std::uint8_t hostId) noexcept
    : motorId_{motorId}, hostId_{hostId}
{
}

void Rs06Motor::requestEnable() noexcept
{
    if (phase_ == Phase::StopSent) {
        advanceToken();
        phase_ = Phase::EnableQueued;
    }
}

void Rs06Motor::requestStop() noexcept
{
    if (phase_ == Phase::StopQueued || phase_ == Phase::StopSent) {
        return;
    }

    // 受付済みType 3は物理的に取り消せない。旧トークンの終端を確認後、
    // 新トークンのType 4を送り、同じ軸で有効化が停止より後に届くのを防ぐ。
    advanceToken();
    phase_ = Phase::StopQueued;
}

std::optional<Rs06TxRequest> Rs06Motor::nextRequest() const noexcept
{
    if (accepted_ || serial_ == 0 ||
        (phase_ != Phase::StopQueued && phase_ != Phase::EnableQueued)) {
        return std::nullopt;
    }
    const auto command = phase_ == Phase::StopQueued ? Rs06Command::Stop : Rs06Command::Enable;
    const auto frame = Rs06Codec::encodeControl(command, motorId_, hostId_);
    if (!frame) {
        return std::nullopt;
    }
    return Rs06TxRequest{{motorId_, serial_}, command, *frame};
}

void Rs06Motor::onTxResult(Rs06TxToken token, Rs06TxResult result, std::uint64_t atUs) noexcept
{
    if (token.motorId != motorId_ || token.serial == 0 || result == Rs06TxResult::Busy) {
        return;
    }

    if (accepted_) {
        if (token.serial != accepted_->serial || atUs < accepted_->acceptedAtUs ||
            result == Rs06TxResult::Accepted) {
            return;
        }

        accepted_.reset();
        if (token.serial == serial_) {
            completeCurrent(result, atUs);
        }
        return;
    }

    if (token.serial != serial_ ||
        (phase_ != Phase::StopQueued && phase_ != Phase::EnableQueued)) {
        return;
    }
    if (result == Rs06TxResult::Accepted) {
        accepted_ = AcceptedTx{token.serial, atUs};
    } else if (result == Rs06TxResult::Failed || result == Rs06TxResult::RetryableFailure) {
        completeCurrent(result, atUs);
    }
    // 受付記録のない完了通知では有効化・停止の送信済み判定を進めない。
}

void Rs06Motor::completeCurrent(Rs06TxResult result, std::uint64_t atUs) noexcept
{
    const bool stopping = phase_ == Phase::StopQueued;
    if (result == Rs06TxResult::Completed) {
        txCompletedAtUs_ = atUs;
        phase_ = stopping ? Phase::StopSent : Phase::EnableSent;
        if (stopping) {
            stopTxFailed_ = false;
        }
    } else if (!stopping && result == Rs06TxResult::Failed) {
        // 確定失敗はRobotSequenceへ渡し、全体期限を待たず停止に移行させる。
        phase_ = Phase::Failed;
    } else {
        stopTxFailed_ = stopTxFailed_ || (stopping && result == Rs06TxResult::Failed);
        // 終端結果を受けた試行とは番号を変え、遅延完了を次の試行に使わせない。
        advanceToken();
    }
}

bool Rs06Motor::onFeedback(const Rs06StateReport& feedback, std::uint64_t receivedAtUs) noexcept
{
    if (feedback.motorId != motorId_ || feedback.hostId != hostId_ ||
        (latest_ && receivedAtUs < latest_->receivedAtUs)) {
        return false;
    }
    latest_ = Rs06FeedbackSample{feedback, receivedAtUs};
    if (feedback.report == Rs06Report::Status) {
        // Type 24が届いた後もType 2の照合情報は必要。物理量は最新報告だけに保持する。
        latestStatus_ = StatusConfirmation{feedback.mode, feedback.faultBits, receivedAtUs};
    }
    return true;
}

Rs06MotorStatus Rs06Motor::status(std::uint64_t nowUs) const noexcept
{
    const bool fresh = feedbackFresh(nowUs);
    const bool fault = hasReportAt(nowUs) && latest_->report.faultBits != 0;
    const auto confirms = [&](Rs06Mode mode) {
        // Type 24の定期報告は鮮度・故障監視に使う。Tx後の状態遷移は
        // Type 2で確認し、より新しい報告のモードや故障とも一致させる。
        return fresh && !fault && latest_->report.mode == mode && latestStatus_ &&
               latestStatus_->receivedAtUs > txCompletedAtUs_ &&
               latestStatus_->receivedAtUs <= nowUs &&
               nowUs - latestStatus_->receivedAtUs < feedbackTimeoutUs &&
               latestStatus_->mode == mode && latestStatus_->faultBits == 0;
    };
    auto enabled = Rs06EnableStatus::Pending;
    if (phase_ == Phase::Failed || serial_ == 0) {
        enabled = Rs06EnableStatus::Failed;
    } else if (phase_ == Phase::EnableSent && confirms(Rs06Mode::Running)) {
        enabled = Rs06EnableStatus::Confirmed;
    }
    const bool stoppedSent = phase_ == Phase::StopSent && txCompletedAtUs_ <= nowUs;
    return {enabled, phase_ == Phase::StopQueued ||
                         (phase_ == Phase::StopSent && !stoppedSent),
            stoppedSent && confirms(Rs06Mode::Reset), fresh, fault, stopTxFailed_ || serial_ == 0};
}

Rs06AxisFeedback Rs06Motor::feedback(std::uint64_t nowUs) const noexcept
{
    return {latest_, feedbackFresh(nowUs)};
}

bool Rs06Motor::hasReportAt(std::uint64_t nowUs) const noexcept
{
    return latest_ && latest_->receivedAtUs <= nowUs;
}

bool Rs06Motor::feedbackFresh(std::uint64_t nowUs) const noexcept
{
    // ARM判定と周期スナップショットが同じ受信時刻と期限を使う。30 msちょうどで期限切れ。
    return hasReportAt(nowUs) && nowUs - latest_->receivedAtUs < feedbackTimeoutUs;
}

void Rs06Motor::advanceToken() noexcept
{
    // 番号を再利用せず、枯渇時は送信候補を出さずに故障判定にする。
    if (serial_ != 0) {
        serial_ = serial_ == std::numeric_limits<std::uint64_t>::max() ? 0 : serial_ + 1;
    }
}

} // namespace balancing
