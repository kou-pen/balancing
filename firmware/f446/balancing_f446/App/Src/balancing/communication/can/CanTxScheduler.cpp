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
    // 直送も停止の優先順位を守る。通常イベントでFIFOがすでに空でも、その終端まで
    // 次のフレームは待つ。先に投入すると小さいCAN IDが通常イベントを追い越し得る。
    if (hasStop() || eventCount_ != 0 || hasEventInFlight() || hasQueuedPeriodic(token)) {
        return CanTxSubmitResult::Busy;
    }
    return submit(frame, {token, nowUs});
}

bool CanTxScheduler::isAccepted(TxToken token) const noexcept
{
    const auto sameToken = [token](const AcceptedTx& entry) { return entry.token == token; };
    return std::any_of(accepted_.begin(), accepted_.end(), sameToken);
}

bool CanTxScheduler::isKnownToken(TxToken token) const noexcept
{
    // 送信結果はバス・トークンで配送するため、同じバスの待機列が違ってもキーは共有する。
    // 停止が失敗した直後のキーも保持し、遅い終端を再試行の結果として使わない。
    const auto sameToken = [token](const StopSlot& slot) {
        return slot.state != StopState::Idle && slot.token == token;
    };
    return isAccepted(token) || hasQueuedPeriodic(token) || hasQueuedEvent(token) ||
           std::any_of(stops_.begin(), stops_.end(), sameToken);
}

bool CanTxScheduler::hasStop() const noexcept
{
    return std::any_of(stops_.begin(), stops_.end(),
                       [](const StopSlot& slot) { return slot.state != StopState::Idle; });
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
        if (token.serial == 0u || !request.frame.validForTransmit() || isKnownToken(token) ||
            std::any_of(batch.requests.begin(), entry, sameToken)) {
            return PeriodicBatchResult::Invalid;
        }
    }

    const auto stopping = hasStop();
    if (stopping || nowUs >= batch.expiresAtUs) {
        // 停止中や期限切れのフレームは保持しない。世代だけを進め、停止完了後に
        // 呼出側が停止中の同じフレームを再登録しても復活しないようにする。
        periodic_ = {batch.generation, batch.expiresAtUs, {}};
        return stopping ? PeriodicBatchResult::StopPending : PeriodicBatchResult::Expired;
    }
    periodic_ = batch;
    return PeriodicBatchResult::Queued;
}

EventEnqueueResult CanTxScheduler::enqueueEvent(const EventTxRequest& request, std::uint64_t nowUs) noexcept
{
    const auto token = request.token;
    // 終端の照合キーは同じバスの全送信経路で共有する。待機中も含めて重複を防ぐと、
    // 周期フレームと通常イベントのどちらの結果かを要求元が特定できる。
    if (token.serial == 0 || !request.frame.validForTransmit() || isKnownToken(token)) {
        return EventEnqueueResult::Invalid;
    }
    if (nowUs >= request.expiresAtUs) {
        return EventEnqueueResult::Expired;
    }
    // 満杯なら登録元に再試行の判断を返す。先に登録された操作を上書きすると順序が失われる。
    if (eventCount_ == eventCapacity_) {
        return EventEnqueueResult::QueueFull;
    }

    // 空き確認後に末尾を求めるので、記憶領域が0件でも剰余演算やアクセスを行わない。
    events_[(eventHead_ + eventCount_) % eventCapacity_] = request;
    ++eventCount_;
    return EventEnqueueResult::Queued;
}

StopEnqueueResult CanTxScheduler::enqueueStop(std::size_t target, const CanFrame& frame,
                                               TxToken token) noexcept
{
    if (target >= stops_.size() || token.serial == 0u || !frame.validForTransmit() || isKnownToken(token)) {
        return StopEnqueueResult::Invalid;
    }
    auto& stop = stops_[target];
    // InFlightのキーを上書きすると、古い終端が新しい要求を解除したり、
    // 対応先が分からなくなったりする。終端後の再試行だけが同じ枠を再登録できる。
    if (stop.state == StopState::Pending || stop.state == StopState::InFlight) {
        return StopEnqueueResult::TargetBusy;
    }
    stop = {frame, token, StopState::Pending};
    // 受付済みの送信はaccepted_に残る。ここで取り消せるのはソフトウェアの待ちだけ。
    // 周期フレームは停止完了後のモータ出力に使わないよう、世代を保持したまま取り除く。
    periodic_.requests = {};
    return StopEnqueueResult::Queued;
}

ScheduledTxProgress CanTxScheduler::service(std::uint64_t nowUs) noexcept
{
    if (hasStop()) {
        return serviceStop(nowUs);
    }
    if (eventCount_ == 0) {
        // 受付時点でFIFOの枠は返すため、FIFOが空でもメールボックスに通常イベントが残り得る。
        // 周期フレームを投入できるのは、その送信結果を回収して照合表を空けた後。
        if (hasEventInFlight()) {
            return {ScheduledTxResult::Busy, {}};
        }
        return servicePeriodic(nowUs);
    }

    const auto& request = events_[eventHead_];
    const auto token = request.token;
    if (nowUs >= request.expiresAtUs) {
        // 期限はソフトウェア内の待ちに対して評価する。先行送信やポート復旧を待ってから
        // 判定すると、すでに無効な操作がFIFOの先頭に居続ける。
        popEvent();
        return {ScheduledTxResult::Expired, token};
    }
    // TXFPは無効。次の通常イベントのTXRQは前のイベントの終端を回収した後に立てる。
    if (hasEventInFlight()) {
        return {ScheduledTxResult::Busy, {}};
    }

    // 受付済みの要求はsubmit()が照合表へ移す。起動前や空きなしで未受付なら
    // 同じ先頭を残し、次の呼び出しで期限を再確認する。
    const auto result = submitScheduled(request.frame, {token, nowUs, 0, true});
    if (result.result == ScheduledTxResult::Accepted || result.result == ScheduledTxResult::Invalid) {
        popEvent();
    }
    return result;
}

ScheduledTxProgress CanTxScheduler::serviceStop(std::uint64_t nowUs) noexcept
{
    // 別の軸が終端待ち・再試行待ちでも、まだ受付されていない停止先を進められる。
    // 空き枠は通常イベントの終端を待たず利用する。既にHWにあるフレームの順序は変えない。
    const auto pending = std::find_if(stops_.begin(), stops_.end(),
                                      [](const StopSlot& slot) { return slot.state == StopState::Pending; });
    if (pending == stops_.end()) {
        return {ScheduledTxResult::Busy, {}};
    }
    // フレームとキーは登録時に検証済み。再試行待ちへの遷移は送信終端で扱う。
    const auto progress = submitScheduled(pending->frame, {pending->token, nowUs});
    if (progress.result == ScheduledTxResult::Accepted) {
        pending->state = StopState::InFlight;
    }
    return progress;
}

void CanTxScheduler::popEvent() noexcept
{
    // 先頭だけを進める。途中の要素を移動させず、循環しても登録順を維持できる。
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
    // 使用中の範囲だけを調べる。取り出し済み領域の値は循環用の空きとして扱う。
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

bool CanTxScheduler::consumeTerminal(TxToken token, CanTxOutcome outcome, std::uint64_t atUs) noexcept
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

    // 停止の送信失敗もメールボックスの終端になるが、停止要求は残す。要求元が新しい
    // キーで再試行する前に通常送信が再開すると、停止できなかった対象へ指令を出し得る。
    const auto sameStop = [token](const StopSlot& slot) {
        return slot.state == StopState::InFlight && slot.token == token;
    };
    const auto stop = std::find_if(stops_.begin(), stops_.end(), sameStop);
    if (stop != stops_.end()) {
        if (outcome == CanTxOutcome::Sent) {
            *stop = {};
        } else {
            stop->state = StopState::RetryRequired;
        }
    }
    // 配送前に枠を返す。CanRouterの通知を受けた要求元が、その場で再登録できる。
    *accepted = {};
    return true;
}

} // namespace balancing
