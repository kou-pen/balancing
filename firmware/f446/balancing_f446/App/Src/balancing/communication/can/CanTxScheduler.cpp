#include "balancing/communication/can/CanTxScheduler.hpp"

#include "balancing/communication/can/CanPort.hpp"

#include <algorithm>

namespace balancing {

CanTxScheduler::CanTxScheduler(CanPort& port) noexcept : port_{port}
{
}

CanTxSubmitResult CanTxScheduler::trySubmit(const CanFrame& frame, TxToken token,
                                            std::uint64_t nowUs) noexcept
{
    // 受付済みの通常イベントも優先し、後から直送した小さいIDが追い越すのを防ぐ。
    if (eventCount_ != 0 || hasEventInFlight() || hasQueuedPeriodic(token)) {
        return CanTxSubmitResult::Busy;
    }
    return submit(frame, {token, nowUs});
}

bool CanTxScheduler::isAccepted(TxToken token) const noexcept
{
    const auto sameToken = [token](const AcceptedTx& entry) { return entry.token == token; };
    return std::any_of(accepted_.begin(), accepted_.end(), sameToken);
}

CanTxSubmitResult CanTxScheduler::submit(const CanFrame& frame, AcceptedTx accepted) noexcept
{
    const auto token = accepted.token;
    // serial=0を空き枠の印として使うため、ポートへ触る前に無効なキーを除外する。
    if (token.serial == 0u) {
        return CanTxSubmitResult::Invalid;
    }
    if (isAccepted(token)) {
        return CanTxSubmitResult::Busy;
    }

    const auto available = std::find_if(accepted_.begin(), accepted_.end(),
                                        [](const AcceptedTx& entry) { return entry.token.serial == 0u; });
    if (available == accepted_.end()) {
        return CanTxSubmitResult::Busy;
    }

    // IRQが先行しても CanPort 内にイベントを保存するだけ。主ループで照合するまでに
    // この記録が完了するため、レジスタ操作以外へIRQマスク区間を広げる必要はない。
    const auto result = port_.trySend(frame, token, accepted.acceptedAtUs);
    if (result == CanTxSubmitResult::Accepted) {
        *available = accepted;
    }
    return result;
}

PeriodicBatchResult CanTxScheduler::replacePeriodic(const PeriodicTxBatch& batch,
                                                       std::uint64_t nowUs) noexcept
{
    if (batch.generation == 0u) {
        return PeriodicBatchResult::Invalid;
    }
    if (batch.generation <= periodic_.generation) {
        return PeriodicBatchResult::StaleGeneration;
    }

    // 最後の対象が不正でも、旧世代の一部だけを消さないよう、集合全体を先に検証する。
    for (auto entry = batch.requests.begin(); entry != batch.requests.end(); ++entry) {
        if (!*entry) {
            continue;
        }
        const auto& request = **entry;
        const auto token = request.token;
        const auto sameToken = [token](const auto& other) { return other && other->token == token; };
        if (token.serial == 0u || !request.frame.validForTransmit() || isAccepted(token) ||
            std::any_of(batch.requests.begin(), entry, sameToken) ||
            hasQueuedPeriodic(token) || hasQueuedEvent(token)) {
            return PeriodicBatchResult::Invalid;
        }
    }

    if (nowUs >= batch.expiresAtUs) {
        // 新世代がすでに期限切れでも、古い指令へ戻らないように世代を進める。
        periodic_ = {batch.generation, batch.expiresAtUs, {}};
        return PeriodicBatchResult::Expired;
    }
    periodic_ = batch;
    return PeriodicBatchResult::Queued;
}

EventEnqueueResult CanTxScheduler::enqueueEvent(const EventTxRequest& request, std::uint64_t nowUs) noexcept
{
    const auto token = request.token;
    if (token.serial == 0 || !request.frame.validForTransmit() || isAccepted(token) ||
        hasQueuedPeriodic(token) || hasQueuedEvent(token)) {
        return EventEnqueueResult::Invalid;
    }
    if (nowUs >= request.expiresAtUs) {
        return EventEnqueueResult::Expired;
    }
    if (eventCount_ == eventCapacity_) {
        return EventEnqueueResult::QueueFull;
    }

    events_[(eventHead_ + eventCount_) % eventCapacity_] = request;
    ++eventCount_;
    return EventEnqueueResult::Queued;
}

ScheduledTxProgress CanTxScheduler::service(std::uint64_t nowUs) noexcept
{
    if (eventCount_ == 0) {
        // 通常イベントは終端まで優先する。後から投入するフレームとのID優先度による逆転を防ぐ。
        if (hasEventInFlight()) {
            return {ScheduledTxResult::Busy, {}};
        }
        return servicePeriodic(nowUs);
    }

    const auto& request = events_[eventHead_];
    const auto token = request.token;
    if (nowUs >= request.expiresAtUs) {
        // ポートの状態に関係なく、未受付の先頭要求は期限に達した時点で破棄できる。
        popEvent();
        return {ScheduledTxResult::Expired, token};
    }
    // TXFPは無効。次の通常イベントのTXRQは前のイベントの終端を回収した後に立てる。
    if (hasEventInFlight()) {
        return {ScheduledTxResult::Busy, {}};
    }

    const auto result = submitScheduled(request.frame, {token, nowUs, 0, true});
    if (result.result == ScheduledTxResult::Accepted || result.result == ScheduledTxResult::Invalid) {
        popEvent();
    }
    return result;
}

void CanTxScheduler::popEvent() noexcept
{
    eventHead_ = (eventHead_ + 1) % eventCapacity_;
    --eventCount_;
}

bool CanTxScheduler::hasEventInFlight() const noexcept
{
    return std::any_of(accepted_.begin(), accepted_.end(),
                       [](const AcceptedTx& tx) { return tx.normalEvent; });
}

bool CanTxScheduler::hasQueuedEvent(TxToken token) const noexcept
{
    for (std::size_t i = 0; i < eventCount_; ++i) {
        if (events_[(eventHead_ + i) % eventCapacity_].token == token) {
            return true;
        }
    }
    return false;
}

bool CanTxScheduler::hasQueuedPeriodic(TxToken token) const noexcept
{
    const auto matches = [token](const auto& request) { return request && request->token == token; };
    return std::any_of(periodic_.requests.begin(), periodic_.requests.end(), matches);
}

ScheduledTxProgress CanTxScheduler::submitScheduled(const CanFrame& frame, AcceptedTx accepted) noexcept
{
    switch (submit(frame, accepted)) {
    case CanTxSubmitResult::Accepted:
        return {ScheduledTxResult::Accepted, accepted.token};
    case CanTxSubmitResult::Busy:
        return {ScheduledTxResult::Busy, {}};
    case CanTxSubmitResult::NotReady:
        return {ScheduledTxResult::NotReady, {}};
    case CanTxSubmitResult::Invalid:
        // 入力検証済みの要求でも、ポートが拒否したら要求元へ返して待ちから外す。
        return {ScheduledTxResult::Invalid, accepted.token};
    }
    return {ScheduledTxResult::Invalid, accepted.token};
}

ScheduledTxProgress CanTxScheduler::servicePeriodic(std::uint64_t nowUs) noexcept
{
    const auto slot = std::find_if(periodic_.requests.begin(), periodic_.requests.end(),
                                   [](const auto& entry) { return entry.has_value(); });
    if (slot == periodic_.requests.end()) {
        return {};
    }

    const auto& request = **slot;
    const auto token = request.token;
    // 1件ずつトークンを返すことで、メールボックスが満杯でも期限切れを要求元へ通知できる。
    if (nowUs >= periodic_.expiresAtUs) {
        slot->reset();
        return {ScheduledTxResult::Expired, token};
    }

    // bxCANに受理された旧フレームの送信順は上位から変えられない。
    // 旧周期の終端を回収するまでは新周期を重ねず、待ちの期限だけを維持する。
    const auto olderPeriodic = [this](const AcceptedTx& tx) {
        return tx.periodicGeneration != 0u && tx.periodicGeneration < periodic_.generation;
    };
    if (std::any_of(accepted_.begin(), accepted_.end(), olderPeriodic)) {
        return {ScheduledTxResult::Busy, {}};
    }

    const auto progress = submitScheduled(request.frame, {token, nowUs, periodic_.generation});
    if (progress.result == ScheduledTxResult::Accepted || progress.result == ScheduledTxResult::Invalid) {
        slot->reset();
    }
    return progress;
}

bool CanTxScheduler::consumeTerminal(TxToken token, std::uint64_t atUs) noexcept
{
    if (token.serial == 0u) {
        return false;
    }
    const auto matches = [token](const AcceptedTx& entry) { return entry.token == token; };
    const auto accepted = std::find_if(accepted_.begin(), accepted_.end(), matches);
    // 同じµsの受付と完了はあり得る。受付より前の終端が枠を解放すると、次の正常な
    // 終端を取りこぼすため、時刻が逆行したイベントでは記録を消さない。
    if (accepted == accepted_.end() || atUs < accepted->acceptedAtUs) {
        return false;
    }

    // 送信失敗もメールボックスの終端。結果ごとの動作は通知を受けた要求元が決める。
    *accepted = {};
    return true;
}

} // namespace balancing
