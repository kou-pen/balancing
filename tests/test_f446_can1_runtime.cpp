#include "balancing_f446_can1_bridge.h"
#include "balancing_app_bridge.h"
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/device/ImuObservation.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr auto notifications = CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING |
                               CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR;
CAN_TypeDef can1{};
CAN_TypeDef can2{};
TestDwtRegisters dwt{};
TestCoreDebugRegisters debug{};
std::uint32_t primask{};
std::array<std::uint32_t, 2> enabledNotifications{};
std::array<CAN_FilterTypeDef, 28> installedFilters{};
std::size_t filterCount{};
std::size_t startCount{};
std::size_t stopCount{};
bool filterFails{};
int startFailsBus = -1;
bool startTimeout{};
int irqEnableFailsBus = -1;

void pushReceive(unsigned motor = 11, unsigned host = 0x5a)
{
    // Type 2の運転モード報告。実機確認前のためfixtureのノードIDを使う。
    const std::uint32_t frameId = (2u << 24u) | (2u << 22u) | (motor << 8u) | host;
    can1.sFIFOMailBox[0] = {(frameId << 3u) | CAN_RI0R_IDE, 8u, 0u, 0u};
    can1.RF0R.count = 1u;
}

void pushWheel(std::uint32_t id, std::uint32_t dlc = 8u, unsigned raw = 0)
{
    can2.sFIFOMailBox[0] = {id << 21u, dlc, (raw >> 8u) | ((raw & 0xffu) << 8u), 0u};
    can2.RF0R.count = 1u;
}

std::size_t busIndex(const CAN_HandleTypeDef* handle)
{
    assert(handle->Instance == &can1 || handle->Instance == &can2);
    return handle->Instance == &can1 ? 0u : 1u;
}

} // namespace

TestDwtRegisters* DWT = &dwt;
TestCoreDebugRegisters* CoreDebug = &debug;
CAN_TypeDef* CAN1 = &can1;
CAN_TypeDef* CAN2 = &can2;
std::uint32_t SystemCoreClock = 168000000u;

std::uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1u; }
void __enable_irq() { primask = 0u; }
void __set_PRIMASK(std::uint32_t value) { primask = value; }

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* handle, const CAN_FilterTypeDef* filter)
{
    assert(handle->State == HAL_CAN_STATE_READY);
    assert((filter->FilterBank < 14u) == (busIndex(handle) == 0u));
    assert(filter->FilterBank < installedFilters.size());
    installedFilters[filter->FilterBank] = *filter;
    ++filterCount;
    return filterFails && filter->FilterBank == 15u ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* handle)
{
    assert(handle->State == HAL_CAN_STATE_READY && enabledNotifications[busIndex(handle)] == 0u);
    assert(installedFilters[15].FilterActivation == ENABLE);
    ++startCount;
    if (static_cast<int>(busIndex(handle)) == startFailsBus) {
        if (startTimeout) handle->State = HAL_CAN_STATE_ERROR;
        return HAL_ERROR;
    }
    handle->State = HAL_CAN_STATE_LISTENING;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* handle)
{
    assert(enabledNotifications[busIndex(handle)] == 0u);
    ++stopCount;
    if (handle->State != HAL_CAN_STATE_LISTENING) return HAL_ERROR;
    handle->State = HAL_CAN_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* handle, std::uint32_t value)
{
    assert(handle->State == HAL_CAN_STATE_LISTENING && value == notifications);
    enabledNotifications[busIndex(handle)] = value;
    if (static_cast<int>(busIndex(handle)) == irqEnableFailsBus) {
        return HAL_ERROR;
    }

    // CANが通知有効化直後に割り込む場合にも、ポートは構築済み。
    BF446Can1Snapshot unpublished1{};
    BF446Can2Snapshot unpublished2{};
    assert(!balancing_f446_can1_snapshot(&unpublished1));
    assert(!balancing_f446_can2_snapshot(&unpublished2));
    if (busIndex(handle) == 0u) {
        pushReceive();
        assert(balancing_f446_can1_capture_irq());
    } else {
        pushWheel(0x201u);
        assert(balancing_f446_can2_capture_irq());
    }
    return HAL_OK;
}

void __HAL_CAN_DISABLE_IT(CAN_HandleTypeDef* handle, std::uint32_t value)
{
    enabledNotifications[busIndex(handle)] &= ~value;
}

balancing::ImuAttitude balancing::imuAttitude(std::uint64_t) noexcept { return {}; }

int main(int argc, char** argv)
{
    CAN_HandleTypeDef handle{CAN1, HAL_CAN_STATE_READY};
    CAN_HandleTypeDef handle2{CAN2, HAL_CAN_STATE_READY};
    BF446Can1Snapshot snapshot{};
    BF446Can2Snapshot wheels{};
    if (argc == 2 && std::strcmp(argv[1], "--rollback-failure") == 0) {
        // 実HALのStartタイムアウトはERRORへ遷移し、Stopが拒否される。
        // 復帰不能のCAN2を正常と公開せず、CAN1は停止し、再起動も拒否する。
        startFailsBus = 1;
        startTimeout = true;
        assert(!balancing_f446_can_start(&handle, &handle2));
        assert(handle.State == HAL_CAN_STATE_READY);
        assert(handle2.State == HAL_CAN_STATE_ERROR);
        assert(stopCount == 2u);
        assert(!balancing_f446_can1_snapshot(&snapshot));
        assert(!balancing_f446_can2_snapshot(&wheels));
        assert(!balancing_f446_can1_capture_irq() && !balancing_f446_can2_capture_irq());
        assert(enabledNotifications[0] == 0u && enabledNotifications[1] == 0u);
        // 外からREADYへ変更されても、不明なハード状態で起動し直さない。
        handle2.State = HAL_CAN_STATE_READY;
        startFailsBus = -1;
        const auto attempts = startCount;
        assert(!balancing_f446_can_start(&handle, &handle2));
        assert(startCount == attempts);
        std::puts("PASS: failed CAN2 rollback is latched");
        return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "--motor-observation") == 0) {
        using namespace balancing;
        MotorObservationConfig config{{{11,12,13,14},0x5a},true,{{{4000,true},{4000,true}}}};
        assert(!motorObservation() && !observationClockUs());
        assert(startCanObservation(&handle,&handle2,config));
        assert(!motorObservation());
        for (unsigned id = 12; id <= 14; ++id) {
            DWT->CYCCNT += 168;
            pushReceive(id);
            assert(balancing_f446_can1_capture_irq());
        }
        pushWheel(0x202);
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        auto snap = motorObservation();
        assert(snap && snap->rs06MappingVerified && snap->motors.sampledAtUs == 3);
        for (const auto& axis : snap->motors.rs06) assert(axis.latest && axis.fresh);
        for (const auto& axis : snap->motors.c620) assert(axis.latest && axis.fresh && axis.position);
        DWT->CYCCNT += 168000;
        pushWheel(0x201,8,3000);
        assert(balancing_f446_can2_capture_irq());
        pushWheel(0x201,8,6000);
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        snap = motorObservation();
        assert(snap->motors.c620[0].position->counts == 6000);
        assert(snap->motors.c620[0].latest->sequence == 3);
        DWT->CYCCNT += 168;
        pushReceive(11,0x33); // 異なる主機IDは既存報告を上書きしない
        assert(balancing_f446_can1_capture_irq());
        can2.ESR = CAN_ESR_BOFF;
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        const auto failed = motorObservation();
        assert(failed->can2BusOff && failed->observedFaults == FaultFlags::BusOff);
        assert(failed->motors.rs06[0].latest->receivedAtUs == 0);
        assert(!failed->motors.c620[0].position && !failed->motors.c620[1].position);
        assert(motorObservation()->observedFaults == FaultFlags::BusOff);
        DWT->CYCCNT += 168;
        can2.ESR = 0;
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        snap = motorObservation();
        assert(!snap->can2BusOff && snap->observedFaults == FaultFlags::BusOff);
        assert(!acknowledgeMotorObservation(failed->generation));
        assert(acknowledgeMotorObservation(snap->generation));
        DWT->CYCCNT += 168u*30000u;
        balancing_f446_can_service();
        snap = motorObservation();
        assert(snap->observedFaults == FaultFlags::None);
        for (const auto& axis : snap->motors.rs06) assert(axis.latest && !axis.fresh);
        for (const auto& axis : snap->motors.c620) assert(axis.latest && !axis.fresh && !axis.position);
        for (const auto* bus : {CAN1,CAN2}) {
            for (const auto& mailbox : bus->sTxMailBox) assert((mailbox.TIR & CAN_TI0R_TXRQ)==0);
        }
        std::puts("PASS: real runtime receive-only six-axis observation");
        return 0;
    }

    assert(!balancing_f446_can_start(nullptr, &handle2));
    assert(!balancing_f446_can_start(&handle2, &handle));
    assert(!balancing_f446_can1_capture_irq());
    assert(!balancing_f446_can2_capture_irq());
    assert(!balancing_f446_can1_snapshot(&snapshot));
    assert(!balancing_f446_can2_snapshot(&wheels));
    balancing_f446_can_service();

    DWT->CTRL = DWT_CTRL_NOCYCCNT_Msk;
    assert(!balancing_f446_can_start(&handle, &handle2));
    DWT->CTRL = 0u;
    assert(filterCount == 0u && startCount == 0u);
    filterFails = true;
    assert(!balancing_f446_can_start(&handle, &handle2) && startCount == 0u);
    filterFails = false;
    // CAN2の起動が失敗したら、先に成功したCAN1も停止する。
    startFailsBus = 1;
    assert(!balancing_f446_can_start(&handle, &handle2));
    assert(stopCount == 1u && handle.State == HAL_CAN_STATE_READY);
    startFailsBus = -1;
    irqEnableFailsBus = 1;
    assert(!balancing_f446_can_start(&handle, &handle2) && stopCount == 3u);
    assert(!balancing_f446_can1_capture_irq() && !balancing_f446_can2_capture_irq());
    assert(enabledNotifications[0] == 0u && enabledNotifications[1] == 0u);
    assert(!balancing_f446_can2_snapshot(&wheels));

    irqEnableFailsBus = -1;
    DWT->CYCCNT = 0xfffffff0u;
    assert(balancing_f446_can_start(&handle, &handle2));
    assert((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
    assert((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u);
    const auto count = startCount;
    assert(balancing_f446_can_start(&handle, &handle2) && startCount == count);
    assert(!balancing_f446_can_start(&handle2, &handle));
    assert(!balancing_f446_can1_snapshot(nullptr));
    assert(!balancing_f446_can2_snapshot(nullptr));
    for (std::size_t bank : {0u, 1u, 14u, 15u}) {
        const auto& f = installedFilters[bank];
        assert(f.FilterBank == bank && f.SlaveStartFilterBank == 14u);
        assert(f.FilterMode == CAN_FILTERMODE_IDMASK && f.FilterScale == CAN_FILTERSCALE_32BIT);
        assert(f.FilterActivation == ENABLE && f.FilterFIFOAssignment == CAN_FILTER_FIFO0);
        if (bank < 14u) {
            assert(f.FilterIdHigh == ((bank == 0 ? 2u : 24u) << 11u));
            assert(f.FilterIdLow == CAN_RI0R_IDE && f.FilterMaskIdHigh == 0xf800u);
        } else {
            assert(f.FilterIdHigh == ((bank == 14u ? 0x201u : 0x202u) << 5u));
            assert(f.FilterIdLow == 0u && f.FilterMaskIdHigh == 0xffe0u);
        }
        assert(f.FilterMaskIdLow == (CAN_RI0R_IDE | CAN_RI0R_RTR));
    }

    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 0u);
    assert(!snapshot.sampled);
    assert(balancing_f446_can2_snapshot(&wheels) && !wheels.sampled);
    balancing_app_init();
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 1u);
    assert(snapshot.last_rx_us == 0u);
    assert(snapshot.sampled && snapshot.sampled_at_us == 0u);
    const auto unconfigured = balancing::motorObservation();
    assert(unconfigured && !unconfigured->rs06MappingVerified);
    assert(unconfigured->observedFaults == balancing::FaultFlags::None);
    for (const auto& axis : unconfigured->motors.rs06) assert(!axis.latest);
    assert(unconfigured->motors.c620[0].latest && !unconfigured->motors.c620[0].position);
    assert(balancing_f446_can2_snapshot(&wheels) && wheels.sampled);
    assert(wheels.left_frames == 1u && wheels.right_frames == 0u);
    assert(wheels.sampled_at_us == snapshot.sampled_at_us);
    balancing_app_control_tick();
    BF446AppTimingSnapshot timing{};
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && !timing.elapsed_valid);
    const auto initialCycle=balancing::applicationObservation();
    assert(initialCycle->motors && initialCycle->cycle);
    assert(initialCycle->cycle->estimateError==balancing::StateEstimateError::InvalidPeriod);
    assert(initialCycle->cycle->sequence.state==balancing::RobotState::Disarmed);

    DWT->CYCCNT = 0x98u;  // 起動時のカウンタから168サイクル、ちょうど1 us。
    can1.ESR = CAN_ESR_BOFF;
    can1.RF0R.flags = CAN_RF0R_FOVR0;
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 1u);
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 2u);
    assert(snapshot.last_rx_us == 1u && snapshot.rx_loss_events == 1u);
    assert(snapshot.bus_off && snapshot.bus_off_events == 1u);
    assert(balancing_f446_can2_snapshot(&wheels));
    assert(!wheels.bus_off && wheels.rx_loss_events == 0u);
    assert(snapshot.sampled_at_us == snapshot.last_rx_us);
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 1u);
    const auto faultCycle=balancing::applicationObservation();
    assert(faultCycle->cycle->sequence.state==balancing::RobotState::Fault);
    assert(faultCycle->fault && faultCycle->fault->primaryCause==balancing::FaultFlags::BusOff);
    assert(!faultCycle->cycle->motorCommandPreview);

    DWT->CYCCNT = 0xffffff00u;
    balancing_f446_can_service();  // IRQがない区間を先にサンプルする。
    DWT->CYCCNT = 0x100u;
    can1.ESR = 0;
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 3u);
    assert(!snapshot.bus_off && snapshot.bus_off_events == 1u);
    assert(snapshot.last_rx_us == ((std::uint64_t{1} << 32u) + 0x110u) / 168u);
    assert(snapshot.sampled_at_us == snapshot.last_rx_us);
    const auto afterWrap = snapshot.sampled_at_us;
    // CAN2のIRQも同じ時計を読む。短いフレームは分類件数に含めない。
    DWT->CYCCNT += 168u;
    can2.ESR = CAN_ESR_BOFF;
    can2.RF0R.flags = CAN_RF0R_FOVR0;
    pushWheel(0x202u);
    primask = 1u;
    assert(balancing_f446_can2_capture_irq());
    assert(primask == 1u);
    primask = 0u;
    pushWheel(0x201u, 7u);
    assert(balancing_f446_can2_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can2_snapshot(&wheels));
    assert(wheels.rx_frames == 3u && wheels.left_frames == 1u && wheels.right_frames == 1u);
    assert(wheels.bus_off && wheels.bus_off_events == 1u && wheels.rx_loss_events == 1u);
    assert(wheels.last_rx_us == afterWrap + 1u);
    assert(balancing_f446_can1_snapshot(&snapshot));
    assert(!snapshot.bus_off && snapshot.rx_loss_events == 1u);
    assert(snapshot.sampled_at_us == wheels.sampled_at_us);
    can2.ESR = 0;
    assert(balancing_f446_can2_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can2_snapshot(&wheels) && !wheels.bus_off);
    assert(wheels.bus_off_events == 1u);
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && timing.sampled_at_us == afterWrap + 1u);
    assert(timing.elapsed_valid && timing.elapsed_us == afterWrap);

    // IRQなしでもserviceの時刻は進む。最後のRX時刻は変更しない。
    DWT->CYCCNT += 168u * 2000u;
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot));
    assert(snapshot.last_rx_us == afterWrap && snapshot.sampled_at_us == afterWrap + 2001u);
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 2000u);
    for (const auto* bus : {CAN1, CAN2}) {
        for (const auto& mailbox : bus->sTxMailBox) {
            assert((mailbox.TIR & CAN_TI0R_TXRQ) == 0u);
        }
    }
    // 一方のキューを溢れさせても他方の新規受信は回収できる。
    for (unsigned i = 0; i < 40u; ++i) {
        pushReceive();
        assert(balancing_f446_can1_capture_irq());
    }
    pushWheel(0x202u);
    assert(balancing_f446_can2_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can2_snapshot(&wheels) && wheels.right_frames == 2u);
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_loss_events > 1u);
    assert(wheels.rx_loss_events == 1u);
    assert(wheels.sampled_at_us == snapshot.sampled_at_us);
    assert(primask == 0u);
    std::puts("PASS: F446 dual CAN boot, IRQ routing, independent monitors and shared clock wrap");
}
