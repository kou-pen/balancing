#include "balancing/device/F446Can1Port.hpp"

namespace balancing {
namespace {

class InterruptGuard {
public:
    InterruptGuard() noexcept : previous_{__get_PRIMASK()} { __disable_irq(); }
    ~InterruptGuard() { if (previous_ == 0u) __enable_irq(); }

private:
    std::uint32_t previous_;
};

constexpr std::array<std::uint32_t, 3> completeFlags{CAN_FLAG_RQCP0, CAN_FLAG_RQCP1, CAN_FLAG_RQCP2};
constexpr std::uint32_t completeBit(std::size_t index) noexcept
{
    return CAN_TSR_RQCP0 << (8u * index);
}

constexpr std::uint32_t emptyBit(std::size_t index) noexcept
{
    return CAN_TSR_TME0 << index;
}

constexpr std::uint32_t abortBit(std::size_t index) noexcept
{
    return CAN_TSR_ABRQ0 << (8u * index);
}

std::uint32_t packWord(const std::uint8_t* bytes) noexcept
{
    // bxCANのTDLR/TDHRは小さいバイト番号を下位ビットに配置する。
    return std::uint32_t{bytes[0]} | (std::uint32_t{bytes[1]} << 8u) |
           (std::uint32_t{bytes[2]} << 16u) | (std::uint32_t{bytes[3]} << 24u);
}

Rs06TxResult terminalResult(std::uint32_t status, std::size_t index) noexcept
{
    status >>= 8u * index;
    if ((status & CAN_TSR_TXOK0) != 0u) {
        return Rs06TxResult::Completed;
    }
    // ALSTと中止は再試行可能。TERRはARM期限を待たず上位へ通知する。
    return (status & CAN_TSR_TERR0) != 0u ? Rs06TxResult::Failed
                                         : Rs06TxResult::RetryableFailure;
}

} // namespace

F446Can1Port::F446Can1Port(CAN_HandleTypeDef& handle) noexcept : handle_{handle}
{
}

Rs06TxResult F446Can1Port::trySend(const Rs06TxRequest& request, std::uint64_t nowUs) noexcept
{
    const auto& frame = request.frame;
    const auto idLimit = frame.extended ? 0x1fffffffu : 0x7ffu;
    if (request.token.serial == 0 || request.token.motorId == 0 ||
        frame.dlc > 8 || frame.id > idLimit || handle_.Instance == nullptr) {
        return Rs06TxResult::Failed;
    }

    InterruptGuard lock;
    auto& can = *handle_.Instance;
    const auto status = can.TSR;
    observeBus(status, nowUs);
    if (busOff_ || pendingBusOff_ || busOffDelivered_ || rxLostAtUs_) {
        return Rs06TxResult::Busy;
    }
    if (HAL_CAN_GetState(&handle_) != HAL_CAN_STATE_LISTENING) {
        return Rs06TxResult::Failed;
    }

    for (const auto& box : mailboxes_) {
        if (box.token && box.token->motorId == request.token.motorId &&
            box.token->serial == request.token.serial) {
            return Rs06TxResult::Busy;
        }
    }
    for (std::size_t i = 0; i < mailboxes_.size(); ++i) {
        auto& box = mailboxes_[i];
        if (box.token || (status & emptyBit(i)) == 0u || (status & completeBit(i)) != 0u) {
            continue;
        }

        // HALの自動メールボックス選択は、別枠が直後に空くと選択が変わりうる。
        // 先に占有を決めた枠だけを書き、トークン保存後にTXRQを立てる。
        auto& tx = can.sTxMailBox[i];
        const auto id = frame.extended ? (frame.id << 3u) | CAN_TI0R_IDE : frame.id << 21u;
        tx.TIR = id;
        tx.TDTR = frame.dlc;
        tx.TDLR = packWord(frame.data);
        tx.TDHR = packWord(frame.data + 4);
        box.token = request.token;
        tx.TIR = id | CAN_TI0R_TXRQ;
        return Rs06TxResult::Accepted;
    }
    return Rs06TxResult::Busy;
}

bool F446Can1Port::requestAbort(Rs06TxToken token) noexcept
{
    if (handle_.Instance == nullptr) {
        return false;
    }
    InterruptGuard lock;
    auto& can = *handle_.Instance;
    const auto status = can.TSR;
    for (std::size_t i = 0; i < mailboxes_.size(); ++i) {
        const auto& box = mailboxes_[i];
        if (box.token && !box.terminalQueued && box.token->motorId == token.motorId &&
            box.token->serial == token.serial && (status & (completeBit(i) | emptyBit(i))) == 0u) {
            // TSRの完了ビットはW1Cなのでread-modify-writeを避ける。
            can.TSR = abortBit(i);
            return true;
        }
    }
    return false;
}

void F446Can1Port::observeBus(std::uint32_t txStatus, std::uint64_t atUs) noexcept
{
    auto& can = *handle_.Instance;
    const bool off = (can.ESR & CAN_ESR_BOFF) != 0u;
    if (off && !busOff_) {
        // 主ループが読む前に復旧しても、最初の故障イベントを保持する。
        if (!pendingBusOff_) {
            pendingBusOff_ = atUs;
        }
        std::uint32_t aborts = 0u;
        for (std::size_t i = 0; i < mailboxes_.size(); ++i) {
            if (mailboxes_[i].token && !mailboxes_[i].terminalQueued &&
                (txStatus & (completeBit(i) | emptyBit(i))) == 0u) {
                aborts |= abortBit(i);
            }
        }
        if (aborts != 0u) {
            can.TSR = aborts;
        }
    } else if (!off && busOff_) {
        busRecoveredAtUs_ = atUs;
    }
    busOff_ = off;
}

void F446Can1Port::captureInterrupt(std::uint64_t atUs) noexcept
{
    if (handle_.Instance == nullptr) {
        return;
    }
    InterruptGuard lock;
    // 先にERRIを確認・クリアし、この処理中に発生した新しいエラーの通知を残す。
    if (__HAL_CAN_GET_FLAG(&handle_, CAN_FLAG_ERRI) != 0u) {
        __HAL_CAN_CLEAR_FLAG(&handle_, CAN_FLAG_ERRI);
    }
    const auto status = handle_.Instance->TSR;
    observeBus(status, atUs);

    // 先に確保した同時刻のRXはTxより前と同様に扱う。FIFO内の古い
    // フレームが割込みの処理順だけで有効化応答になるのを避ける。
    captureRx(atUs);
    for (std::size_t i = 0; i < mailboxes_.size(); ++i) {
        if ((status & completeBit(i)) == 0u) {
            continue;
        }
        __HAL_CAN_CLEAR_FLAG(&handle_, completeFlags[i]);
        auto& box = mailboxes_[i];
        if (box.token && !box.terminalQueued) {
            Can1Event event{Can1EventKind::Tx, atUs};
            event.token = *box.token;
            event.txResult = terminalResult(status, i);
            event.mailbox = static_cast<std::uint8_t>(i);
            queueEvent(event);
            box.terminalQueued = true;
        }
    }
}

void F446Can1Port::captureRx(std::uint64_t atUs) noexcept
{
    if (__HAL_CAN_GET_FLAG(&handle_, CAN_FLAG_FOV0) != 0u) {
        __HAL_CAN_CLEAR_FLAG(&handle_, CAN_FLAG_FOV0);
        markRxLost(atUs);
    }
    // bxCAN FIFO0は3枠。増え続ける受信で割込み処理を長引かせない。
    auto& can = *handle_.Instance;
    const std::uint32_t count = can.RF0R & CAN_RF0R_FMP0;
    for (std::uint32_t i = 0; i < count; ++i) {
        const auto& rx = can.sFIFOMailBox[0];
        const auto id = rx.RIR;
        const auto dlc = rx.RDTR & CAN_RDT0R_DLC;
        const auto low = rx.RDLR;
        const auto high = rx.RDHR;
        // RFOM0だけを書き、同時に立ったFIFOオーバーランを消さない。
        // HALはDLCが8を超えても8へ正規化するため、検証には生のDLCを使う。
        can.RF0R = CAN_RF0R_RFOM0;
        if ((id & CAN_RI0R_RTR) != 0u || dlc > 8u) {
            continue;
        }
        if (size_ >= queueCapacity - mailboxCount) {
            // 受信を捨てても各メールボックスの終端結果用の容量を残す。
            markRxLost(atUs);
            continue;
        }
        Can1Event event{Can1EventKind::Rx, atUs};
        event.frame.extended = (id & CAN_RI0R_IDE) != 0u;
        event.frame.id = event.frame.extended ? id >> 3u : id >> 21u;
        event.frame.dlc = static_cast<std::uint8_t>(dlc);
        for (std::size_t b = 0; b < 4; ++b) {
            event.frame.data[b] = static_cast<std::uint8_t>(low >> (8u * b));
            event.frame.data[b + 4] = static_cast<std::uint8_t>(high >> (8u * b));
        }
        queueEvent(event);
    }
}

bool F446Can1Port::popEvent(Can1Event& event) noexcept
{
    InterruptGuard lock;
    if (pendingBusOff_) {
        event = {Can1EventKind::BusOff, *pendingBusOff_};
        pendingBusOff_.reset();
        busOffDelivered_ = true;
    } else if (!busOff_ && busOffDelivered_) {
        event = {Can1EventKind::BusRecovered, busRecoveredAtUs_};
        busOffDelivered_ = false;
    } else if (rxLostAtUs_) {
        event = {Can1EventKind::RxLost, *rxLostAtUs_};
        rxLostAtUs_.reset();
    } else if (size_ != 0) {
        event = queue_[read_];
        read_ = (read_ + 1) % queueCapacity;
        --size_;
        if (event.kind == Can1EventKind::Tx) {
            // ハードウェア上は空きでも、終端イベントを渡すまでトークンを占有する。
            mailboxes_[event.mailbox] = {};
        }
    } else {
        return false;
    }
    return true;
}

void F446Can1Port::queueEvent(const Can1Event& event) noexcept
{
    queue_[(read_ + size_) % queueCapacity] = event;
    ++size_;
}

void F446Can1Port::markRxLost(std::uint64_t atUs) noexcept
{
    if (!rxLostAtUs_) {
        rxLostAtUs_ = atUs;
    }
}

} // namespace balancing
