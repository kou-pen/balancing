#include "balancing/communication/can/CanPort.hpp"

namespace balancing {
namespace {

class InterruptGuard {
public:
    InterruptGuard() noexcept : previous_{__get_PRIMASK()} { __disable_irq(); }
    ~InterruptGuard() { if (previous_ == 0u) __enable_irq(); }

private:
    std::uint32_t previous_;
};

constexpr std::array<std::uint32_t, 3> kCompleteFlags{CAN_FLAG_RQCP0, CAN_FLAG_RQCP1, CAN_FLAG_RQCP2};
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

CanTxOutcome terminalOutcome(std::uint32_t status, std::size_t index) noexcept
{
    status >>= 8u * index;
    if ((status & CAN_TSR_TXOK0) != 0u) {
        return CanTxOutcome::Sent;
    }
    // bxCANが区別する終端理由を残す。再試行の可否は要求を所有する上位が決める。
    if ((status & CAN_TSR_TERR0) != 0u) {
        return CanTxOutcome::TransmitError;
    }
    return (status & CAN_TSR_ALST0) != 0u ? CanTxOutcome::ArbitrationLost
                                         : CanTxOutcome::NotTransmitted;
}

} // namespace

CanPort::CanPort(CAN_HandleTypeDef& handle) noexcept : handle_{handle}
{
}

CanTxSubmitResult CanPort::trySend(const CanFrame& frame, TxToken token, std::uint64_t nowUs) noexcept
{
    if (token.serial == 0 || !frame.validForTransmit()) {
        return CanTxSubmitResult::Invalid;
    }
    if (handle_.Instance == nullptr) {
        return CanTxSubmitResult::NotReady;
    }

    InterruptGuard lock;
    auto& can = *handle_.Instance;
    const auto status = can.TSR;
    observeBus(status, nowUs);
    if (busOff_ || pendingBusOff_ || busOffDelivered_ || rxLostAtUs_) {
        return CanTxSubmitResult::Busy;
    }
    if (HAL_CAN_GetState(&handle_) != HAL_CAN_STATE_LISTENING) {
        return CanTxSubmitResult::NotReady;
    }

    for (const auto& box : mailboxes_) {
        if (box.token && *box.token == token) {
            return CanTxSubmitResult::Busy;
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
        tx.TDLR = packWord(frame.data.data());
        tx.TDHR = packWord(frame.data.data() + 4);
        box.token = token;
        tx.TIR = id | CAN_TI0R_TXRQ;
        return CanTxSubmitResult::Accepted;
    }
    return CanTxSubmitResult::Busy;
}

bool CanPort::requestAbort(TxToken token) noexcept
{
    if (handle_.Instance == nullptr) {
        return false;
    }
    InterruptGuard lock;
    auto& can = *handle_.Instance;
    const auto status = can.TSR;
    for (std::size_t i = 0; i < mailboxes_.size(); ++i) {
        const auto& box = mailboxes_[i];
        if (box.token && !box.terminalQueued && *box.token == token && (status & (completeBit(i) | emptyBit(i))) == 0u) {
            // TSRの完了ビットはW1Cなのでread-modify-writeを避ける。
            can.TSR = abortBit(i);
            return true;
        }
    }
    return false;
}

void CanPort::observeBus(std::uint32_t txStatus, std::uint64_t atUs) noexcept
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

void CanPort::captureInterrupt(std::uint64_t atUs) noexcept
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

    // FIFO内の到着順とTX終端の先後はここでは確定できない。
    // 同じIRQで観測したイベントには同じ採取時刻を付けて上位へ渡す。
    captureRx(atUs);
    for (std::size_t i = 0; i < mailboxes_.size(); ++i) {
        if ((status & completeBit(i)) == 0u) {
            continue;
        }
        __HAL_CAN_CLEAR_FLAG(&handle_, kCompleteFlags[i]);
        auto& box = mailboxes_[i];
        if (box.token && !box.terminalQueued) {
            CanEvent event{CanEventKind::Tx, atUs};
            event.token = *box.token;
            event.outcome = terminalOutcome(status, i);
            event.mailbox = static_cast<std::uint8_t>(i);
            queueEvent(event);
            box.terminalQueued = true;
        }
    }
}

void CanPort::captureRx(std::uint64_t atUs) noexcept
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
        if (size_ >= kQueueCapacity - kMailboxCount) {
            // 受信を捨てても各メールボックスの終端結果用の容量を残す。
            markRxLost(atUs);
            continue;
        }
        CanEvent event{CanEventKind::Rx, atUs};
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

bool CanPort::popEvent(CanEvent& event) noexcept
{
    InterruptGuard lock;
    if (pendingBusOff_) {
        event = {CanEventKind::BusOff, *pendingBusOff_};
        pendingBusOff_.reset();
        busOffDelivered_ = true;
    } else if (!busOff_ && busOffDelivered_) {
        event = {CanEventKind::BusRecovered, busRecoveredAtUs_};
        busOffDelivered_ = false;
    } else if (rxLostAtUs_) {
        event = {CanEventKind::RxLost, *rxLostAtUs_};
        rxLostAtUs_.reset();
    } else if (size_ != 0) {
        event = queue_[read_];
        read_ = (read_ + 1) % kQueueCapacity;
        --size_;
        if (event.kind == CanEventKind::Tx) {
            // ハードウェア上は空きでも、終端イベントを渡すまでトークンを占有する。
            mailboxes_[event.mailbox] = {};
        }
    } else {
        return false;
    }
    return true;
}

void CanPort::queueEvent(const CanEvent& event) noexcept
{
    queue_[(read_ + size_) % kQueueCapacity] = event;
    ++size_;
}

void CanPort::markRxLost(std::uint64_t atUs) noexcept
{
    if (!rxLostAtUs_) {
        rxLostAtUs_ = atUs;
    }
}

} // namespace balancing
