#include "balancing/app/MotorService.hpp"

#include <algorithm>

namespace balancing {
namespace {

bool validMotorIds(const std::array<std::uint8_t, 4>& ids) noexcept
{
    auto sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    return sorted.front() != 0 && std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end();
}

} // namespace

MotorService::MotorService(const Rs06BusConfig& config) noexcept
    : config_{config},
      rs06_{{{config.motorIds[0], config.hostId}, {config.motorIds[1], config.hostId},
             {config.motorIds[2], config.hostId}, {config.motorIds[3], config.hostId}}},
      configured_{validMotorIds(config.motorIds)}
{
}

void MotorService::applySequence(const RobotSequenceResult& sequence, std::uint64_t nowUs) noexcept
{
    if (sequence.desiredMotorState == DesiredMotorState::StopAll ||
        (armActive_ && sequence.armSession != armSession_)) {
        // 継続的なStopAllで送信済み軸を再キューしない。軸自身が完了を保持する。
        for (auto& motor : rs06_) {
            motor.requestStop();
        }
        armActive_ = false;
        return;
    }

    if (sequence.desiredMotorState != DesiredMotorState::EnableRs06 || armActive_ ||
        sequence.armSession <= armSession_ || !configured_ || can1BusOff_ ||
        rs06ArmFeedback(nowUs).stopPending) {
        return;
    }

    // 停止送信を全軸で終えた後にのみ、新しいセッションのType 3を組み立てる。
    armSession_ = sequence.armSession;
    armActive_ = true;
    for (auto& motor : rs06_) {
        motor.requestEnable();
    }
}

std::array<std::optional<Rs06TxRequest>, 4> MotorService::pendingRs06Requests() const noexcept
{
    std::array<std::optional<Rs06TxRequest>, 4> requests{};
    if (configured_ && !can1BusOff_) {
        for (std::size_t i = 0; i < rs06_.size(); ++i) {
            requests[i] = rs06_[i].nextRequest();
        }
    }
    return requests;
}

void MotorService::onRs06TxResult(
    Rs06TxToken token, Rs06TxResult result, std::uint64_t atUs) noexcept
{
    if (auto* motor = findMotor(token.motorId)) {
        motor->onTxResult(token, result, atUs);
        if (motor->status(atUs).stopTxFailed) {
            pendingFaults_ = pendingFaults_ | FaultFlags::ExternalFault;
        }
    }
}

void MotorService::onRs06Frame(const BMotorCanFrame& frame, std::uint64_t receivedAtUs) noexcept
{
    if (const auto report = Rs06Codec::decodeFeedback(frame)) {
        if (auto* motor = findMotor(report->motorId)) {
            if (motor->onFeedback(*report, receivedAtUs) && report->faultBits != 0) {
                pendingFaults_ = pendingFaults_ | FaultFlags::MotorFault;
            }
        }
    }
}

void MotorService::onC620Frame(const BMotorCanFrame& frame, std::uint64_t receivedAtUs) noexcept
{
    BC620Feedback report{};
    // 共通C11デコーダはID1〜4を扱う。本機の観測対象は先頭の2車輪だけ。
    if (b_c620_parse_feedback(&frame, &report) && report.motor_id <= c620_.size()) {
        c620_[report.motor_id - 1u].onFeedback(report, receivedAtUs);
    }
}

MotorFeedbackSnapshot MotorService::feedback(std::uint64_t nowUs) const noexcept
{
    // 各軸の受信時刻は違うが、期限は全軸とも同じ周期時刻で評価する。
    return {nowUs,
            {rs06_[0].feedback(nowUs), rs06_[1].feedback(nowUs),
             rs06_[2].feedback(nowUs), rs06_[3].feedback(nowUs)},
            {c620_[0].feedback(nowUs), c620_[1].feedback(nowUs)}};
}

void MotorService::setCan1BusOff(bool busOff) noexcept
{
    // バスオフ中は候補の投入を止める。ハードウェアが占有中のトークンは、
    // ポートの完了・失敗通知を待って解放する。
    can1BusOff_ = busOff;
    if (busOff) {
        pendingFaults_ = pendingFaults_ | FaultFlags::BusOff;
    }
}

void MotorService::reportCan1RxLost() noexcept
{
    // 次の受信が正常でも、欠落した報告の内容は分からない。制御周期まで異常を保持する。
    pendingFaults_ = pendingFaults_ | FaultFlags::ExternalFault;
}

std::array<Rs06MotorStatus, 4> MotorService::rs06Status(std::uint64_t nowUs) const noexcept
{
    return {rs06_[0].status(nowUs), rs06_[1].status(nowUs),
            rs06_[2].status(nowUs), rs06_[3].status(nowUs)};
}

Rs06ArmFeedback MotorService::rs06ArmFeedback(std::uint64_t nowUs) const noexcept
{
    Rs06ArmFeedback feedback{};
    feedback.session = armActive_ ? armSession_ : 0;
    const auto statuses = rs06Status(nowUs);
    for (std::size_t i = 0; i < statuses.size(); ++i) {
        const auto& status = statuses[i];
        feedback.stopPending = feedback.stopPending || status.stopPending;
        if (status.enable == Rs06EnableStatus::Confirmed) {
            feedback.axes[i] = Rs06ArmStatus::Confirmed;
        } else if (status.enable == Rs06EnableStatus::Failed) {
            feedback.axes[i] = Rs06ArmStatus::Failed;
        }
    }
    return feedback;
}

FaultFlags MotorService::takeObservedFaults(std::uint64_t nowUs) noexcept
{
    // 1周期の間に正常フレームや復旧通知が来ても、発生した異常を渡す。
    auto flags = pendingFaults_;
    pendingFaults_ = FaultFlags::None;
    if (can1BusOff_) {
        flags = flags | FaultFlags::BusOff;
    }
    if (!configured_) {
        flags = flags | FaultFlags::ExternalFault;
    }
    for (const auto& status : rs06Status(nowUs)) {
        if (status.motorFault) {
            flags = flags | FaultFlags::MotorFault;
        }
        if (status.stopTxFailed) {
            flags = flags | FaultFlags::ExternalFault;
        }
    }
    return flags;
}

Rs06Motor* MotorService::findMotor(std::uint8_t id) noexcept
{
    const auto motor = std::find(config_.motorIds.begin(), config_.motorIds.end(), id);
    if (!configured_ || motor == config_.motorIds.end()) {
        return nullptr;
    }
    return &rs06_[static_cast<std::size_t>(motor - config_.motorIds.begin())];
}

} // namespace balancing
