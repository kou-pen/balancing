#include "balancing_f446_can1_bridge.h"

#include "balancing/app/CanEventPoll.hpp"
#include "balancing/communication/can/CanPort.hpp"
#include "balancing/communication/can/CanRouter.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace {

constexpr std::uint32_t notifications = CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING |
                                        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR;
constexpr std::uint32_t microsecondsPerSecond = 1000000u;
constexpr std::uint32_t extendedDataMask = (0x1fu << 27u) | CAN_RI0R_IDE | CAN_RI0R_RTR;

bool installFilters(CAN_HandleTypeDef& handle)
{
    CAN_FilterTypeDef filter{};
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14u;
    filter.FilterMaskIdHigh = extendedDataMask >> 16u;
    filter.FilterMaskIdLow = extendedDataMask & 0xffffu;

    // Type 2とType 24だけをFIFO0へ入れる。実機未確認の軸ID・主機IDは設定しない。
    constexpr std::array<std::uint32_t, 2> reportTypes{2u, 24u};
    for (const auto type : reportTypes) {
        filter.FilterIdHigh = type << 11u;
        filter.FilterIdLow = CAN_RI0R_IDE;
        if (HAL_CAN_ConfigFilter(&handle, &filter) != HAL_OK) {
            return false;
        }
        ++filter.FilterBank;
    }
    return true;
}

class Can1Runtime final : private balancing::CanEventSink {
public:
    Can1Runtime(CAN_HandleTypeDef& handle, std::uint32_t cyclesPerUs) noexcept
        : port{handle}, handle{handle}, cyclesPerUs_{cyclesPerUs}, lastCycle_{DWT->CYCCNT}
    {
    }

    std::uint64_t sampleClock() noexcept
    {
        const auto irqMask = __get_PRIMASK();
        __disable_irq();
        const auto count = DWT->CYCCNT;
        // 500 Hzの主ループでも読んで、割込みが来ない間のCYCCNT周回をつなぐ。
        // 32 bitの差分を取ってから足すことで、周回境界でも時刻を巻き戻さない。
        accumulated_ += static_cast<std::uint32_t>(count - lastCycle_);
        lastCycle_ = count;
        const auto cycles = accumulated_;
        __set_PRIMASK(irqMask);
        return cycles / cyclesPerUs_;
    }

    void service() noexcept
    {
        sampleClock();
        balancing::CanRouter router{*this};
        // CAN1の観測経路を主ループの共通回収に通す。CAN2の起動・接続は別工程。
        balancing::pollCanEvents(&port, nullptr, router);
    }

    balancing::CanPort port;
    CAN_HandleTypeDef& handle;
    BF446Can1Snapshot snapshot{};

private:
    void onCanReceive(balancing::CanBus, const balancing::CanFrame&, std::uint64_t atUs) noexcept override
    {
        ++snapshot.rx_frames;
        snapshot.last_rx_us = atUs;
    }

    void onCanTransmit(balancing::CanBus, balancing::TxToken, balancing::CanTxOutcome,
                       std::uint64_t) noexcept override
    {
        // 受信監視では送信を登録しない。送信終端の照合は送信サービスを接続する工程で行う。
    }

    void onCanPortStatus(balancing::CanBus, balancing::CanEventKind kind, std::uint64_t) noexcept override
    {
        switch (kind) {
        case balancing::CanEventKind::RxLost:
            ++snapshot.rx_loss_events;
            break;
        case balancing::CanEventKind::BusOff:
            ++snapshot.bus_off_events;
            snapshot.bus_off = true;
            break;
        case balancing::CanEventKind::BusRecovered:
            snapshot.bus_off = false;
            break;
        default:
            break;
        }
    }

    std::uint32_t cyclesPerUs_;
    std::uint32_t lastCycle_;
    std::uint64_t accumulated_{0};
};

std::optional<Can1Runtime> runtime;

} // namespace

extern "C" bool balancing_f446_can1_start(CAN_HandleTypeDef* handle)
{
    if (runtime) {
        return &runtime->handle == handle;
    }
    if (handle == nullptr || handle->Instance != CAN1 ||
        HAL_CAN_GetState(handle) != HAL_CAN_STATE_READY ||
        SystemCoreClock == 0u || SystemCoreClock % microsecondsPerSecond != 0u ||
        (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u) {
        return false;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u ||
        !installFilters(*handle) || HAL_CAN_Start(handle) != HAL_OK) {
        return false;
    }

    // 通知を有効にする前に固定領域を構築する。ISRは主ループの集計に踏み込まない。
    runtime.emplace(*handle, SystemCoreClock / microsecondsPerSecond);
    if (HAL_CAN_ActivateNotification(handle, notifications) != HAL_OK) {
        __HAL_CAN_DISABLE_IT(handle, notifications);
        HAL_CAN_Stop(handle);
        runtime.reset();
        return false;
    }
    return true;
}

extern "C" bool balancing_f446_can1_capture_irq(void)
{
    if (!runtime) {
        return false;
    }
    runtime->port.captureInterrupt(runtime->sampleClock());
    return true;
}

extern "C" void balancing_f446_can1_service(void)
{
    if (runtime) {
        runtime->service();
    }
}

extern "C" bool balancing_f446_can1_snapshot(BF446Can1Snapshot* snapshot)
{
    if (!runtime || snapshot == nullptr) {
        return false;
    }
    *snapshot = runtime->snapshot;
    return true;
}
