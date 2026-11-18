#include "balancing_f446_can1_bridge.h"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

namespace {

constexpr auto notifications = CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING |
                               CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR;
CAN_TypeDef can1{};
TestDwtRegisters dwt{};
TestCoreDebugRegisters debug{};
std::uint32_t primask{};
std::uint32_t enabledNotifications{};
std::array<CAN_FilterTypeDef, 2> installedFilters{};
std::size_t filterCount{};
std::size_t startCount{};
std::size_t stopCount{};
bool filterFails{};
bool startFails{};
bool irqEnableFails{};

void pushReceive()
{
    // Type 2の運転モード報告。実機確認前のためfixtureのノードIDを使う。
    constexpr std::uint32_t frameId = (2u << 24u) | (2u << 22u) | (11u << 8u) | 0x5au;
    can1.sFIFOMailBox[0] = {(frameId << 3u) | CAN_RI0R_IDE, 8u, 0u, 0u};
    can1.RF0R.count = 1u;
}

} // namespace

TestDwtRegisters* DWT = &dwt;
TestCoreDebugRegisters* CoreDebug = &debug;
CAN_TypeDef* CAN1 = &can1;
std::uint32_t SystemCoreClock = 168000000u;

std::uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1u; }
void __enable_irq() { primask = 0u; }
void __set_PRIMASK(std::uint32_t value) { primask = value; }

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* handle, const CAN_FilterTypeDef* filter)
{
    assert(handle->Instance == CAN1 && handle->State == HAL_CAN_STATE_READY);
    assert(filter->FilterBank < installedFilters.size());
    installedFilters[filter->FilterBank] = *filter;
    ++filterCount;
    return filterFails && filter->FilterBank == 1u ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* handle)
{
    assert(handle->State == HAL_CAN_STATE_READY && enabledNotifications == 0u);
    ++startCount;
    if (startFails) {
        return HAL_ERROR;
    }
    handle->State = HAL_CAN_STATE_LISTENING;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* handle)
{
    assert(enabledNotifications == 0u);
    ++stopCount;
    handle->State = HAL_CAN_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* handle, std::uint32_t value)
{
    assert(handle->State == HAL_CAN_STATE_LISTENING && value == notifications);
    enabledNotifications = value;
    if (irqEnableFails) {
        return HAL_ERROR;
    }

    // CANが通知有効化直後に割り込む場合にも、ポートは構築済み。
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    return HAL_OK;
}

void __HAL_CAN_DISABLE_IT(CAN_HandleTypeDef*, std::uint32_t value)
{
    enabledNotifications &= ~value;
}

int main()
{
    CAN_TypeDef can2{};
    CAN_HandleTypeDef wrongBus{&can2, HAL_CAN_STATE_READY};
    CAN_HandleTypeDef handle{CAN1, HAL_CAN_STATE_READY};
    BF446Can1Snapshot snapshot{};
    assert(!balancing_f446_can1_start(nullptr));
    assert(!balancing_f446_can1_start(&wrongBus));
    assert(!balancing_f446_can1_capture_irq());
    assert(!balancing_f446_can1_snapshot(&snapshot));
    balancing_f446_can1_service();

    DWT->CTRL = DWT_CTRL_NOCYCCNT_Msk;
    assert(!balancing_f446_can1_start(&handle));
    DWT->CTRL = 0u;
    assert(filterCount == 0u && startCount == 0u);
    filterFails = true;
    assert(!balancing_f446_can1_start(&handle) && startCount == 0u);
    filterFails = false;
    startFails = true;
    assert(!balancing_f446_can1_start(&handle) && enabledNotifications == 0u);
    startFails = false;
    irqEnableFails = true;
    assert(!balancing_f446_can1_start(&handle) && stopCount == 1u);
    assert(!balancing_f446_can1_capture_irq() && enabledNotifications == 0u);

    irqEnableFails = false;
    DWT->CYCCNT = 0xfffffff0u;
    assert(balancing_f446_can1_start(&handle));
    assert((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
    assert((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u);
    const auto count = startCount;
    assert(balancing_f446_can1_start(&handle) && startCount == count);
    assert(!balancing_f446_can1_start(&wrongBus));
    assert(!balancing_f446_can1_snapshot(nullptr));
    for (std::size_t bank = 0; bank < installedFilters.size(); ++bank) {
        const auto& f = installedFilters[bank];
        assert(f.FilterBank == bank && f.SlaveStartFilterBank == 14u);
        assert(f.FilterMode == CAN_FILTERMODE_IDMASK && f.FilterScale == CAN_FILTERSCALE_32BIT);
        assert(f.FilterActivation == ENABLE && f.FilterFIFOAssignment == CAN_FILTER_FIFO0);
        assert(f.FilterIdHigh == ((bank == 0 ? 2u : 24u) << 11u));
        assert(f.FilterIdLow == CAN_RI0R_IDE && f.FilterMaskIdHigh == 0xf800u);
        assert(f.FilterMaskIdLow == (CAN_RI0R_IDE | CAN_RI0R_RTR));
    }

    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 0u);
    balancing_f446_can1_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 1u);
    assert(snapshot.last_rx_us == 0u);

    DWT->CYCCNT = 0x98u;  // 起動時のカウンタから168サイクル、ちょうど1 us。
    can1.ESR = CAN_ESR_BOFF;
    can1.RF0R.flags = CAN_RF0R_FOVR0;
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 1u);
    balancing_f446_can1_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 2u);
    assert(snapshot.last_rx_us == 1u && snapshot.rx_loss_events == 1u);
    assert(snapshot.bus_off && snapshot.bus_off_events == 1u);

    DWT->CYCCNT = 0xffffff00u;
    balancing_f446_can1_service();  // IRQがない区間を先にサンプルする。
    DWT->CYCCNT = 0x100u;
    can1.ESR = 0;
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    balancing_f446_can1_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 3u);
    assert(!snapshot.bus_off && snapshot.bus_off_events == 1u);
    assert(snapshot.last_rx_us == ((std::uint64_t{1} << 32u) + 0x110u) / 168u);
    for (const auto& mailbox : can1.sTxMailBox) {
        assert((mailbox.TIR & CAN_TI0R_TXRQ) == 0u);
    }
    assert(primask == 0u);
    std::puts("PASS: F446 CAN1 boot, IRQ routing, receive-only monitor and clock wrap");
}
