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
    // serial=0を空き枠の印として使うため、ポートへ触る前に無効なキーを除外する。
    if (token.serial == 0u) {
        return CanTxSubmitResult::Invalid;
    }
    const auto sameToken = [token](const AcceptedTx& entry) { return entry.token == token; };
    if (std::any_of(accepted_.begin(), accepted_.end(), sameToken)) {
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
        *available = {token, nowUs};
    }
    return result;
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
