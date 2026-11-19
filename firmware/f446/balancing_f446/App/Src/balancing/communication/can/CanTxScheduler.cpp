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
    // 直送要求に周期送信待ちと同じトークンを使うと、受付元と終端の対応を失う。
    const auto queuedToken = [token](const auto& request) { return request && request->token == token; };
    if (std::any_of(periodic_.requests.begin(), periodic_.requests.end(), queuedToken)) {
        return CanTxSubmitResult::Busy;
    }
    return submit(frame, token, nowUs, 0);
}

bool CanTxScheduler::isAccepted(TxToken token) const noexcept
{
    const auto sameToken = [token](const AcceptedTx& entry) { return entry.token == token; };
    return std::any_of(accepted_.begin(), accepted_.end(), sameToken);
}

CanTxSubmitResult CanTxScheduler::submit(const CanFrame& frame, TxToken token, std::uint64_t nowUs,
                                         std::uint64_t periodicGeneration) noexcept
{
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
    const auto result = port_.trySend(frame, token, nowUs);
    if (result == CanTxSubmitResult::Accepted) {
        *available = {token, nowUs, periodicGeneration};
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
            std::any_of(periodic_.requests.begin(), periodic_.requests.end(), sameToken)) {
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

PeriodicTxProgress CanTxScheduler::servicePeriodic(std::uint64_t nowUs) noexcept
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
        return {PeriodicTxResult::Expired, token};
    }

    // bxCANに受理された旧フレームの送信順は上位から変えられない。
    // 旧周期の終端を回収するまでは新周期を重ねず、待ちの期限だけを維持する。
    const auto olderPeriodic = [this](const AcceptedTx& tx) {
        return tx.periodicGeneration != 0u && tx.periodicGeneration < periodic_.generation;
    };
    if (std::any_of(accepted_.begin(), accepted_.end(), olderPeriodic)) {
        return {PeriodicTxResult::Busy, {}};
    }

    switch (submit(request.frame, token, nowUs, periodic_.generation)) {
    case CanTxSubmitResult::Accepted:
        slot->reset();
        return {PeriodicTxResult::Accepted, token};
    case CanTxSubmitResult::Busy:
        return {PeriodicTxResult::Busy, {}};
    case CanTxSubmitResult::NotReady:
        return {PeriodicTxResult::NotReady, {}};
    case CanTxSubmitResult::Invalid:
        break;
    }

    // 受理前の検証とポートの検証が食い違った場合は、この要求を再試行せず呼出側へ返す。
    slot->reset();
    return {PeriodicTxResult::Invalid, token};
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
