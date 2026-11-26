#include "balancing_f446_can1_bridge.h"

#include "balancing/communication/can/CanRouter.hpp"
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/app/DriveRuntime.hpp"
#include "balancing/app/MotorCanLink.hpp"
#include "balancing/communication/can/CanPort.hpp"
#include "balancing/communication/can/Can2FilterPlan.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace {

constexpr std::uint32_t notifications = CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING |
                                        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR;
constexpr std::uint32_t microsecondsPerSecond = 1000000u;
constexpr std::uint32_t extendedDataMask = (0x1fu << 27u) | CAN_RI0R_IDE | CAN_RI0R_RTR;

bool installCan1Filters(CAN_HandleTypeDef& handle)
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

bool installCan2Filters(CAN_HandleTypeDef& handle,bool selfDefinedCan2)
{
    // 既定は車輪のみ。明示設定済みの非駆動試験でだけ自作通信を追加する。
    const auto plan=balancing::makeCan2FilterPlan(selfDefinedCan2);
    CAN_FilterTypeDef filter{};
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.SlaveStartFilterBank = 14u;

    // CAN2の全所有バンクを明示設定する。旧設定や起動再試行で
    // 残ったフィルタを有効のままにしない。CAN1の0～13には触れない。
    for (std::size_t i=0;i<14;++i) {
        filter.FilterBank=14u+static_cast<std::uint32_t>(i);
        const bool active=i<plan.count;
        const auto word=active ? plan.entries[i] : balancing::Can2FilterWord{};
        filter.FilterActivation=active ? ENABLE : DISABLE;
        filter.FilterIdHigh=word.first >> 16u;
        filter.FilterIdLow=word.first & 0xffffu;
        // HALの名前はMaskだが、32bit ListではFR2に置く第2のID。
        filter.FilterMaskIdHigh=word.second >> 16u;
        filter.FilterMaskIdLow=word.second & 0xffffu;
        if (HAL_CAN_ConfigFilter(&handle,&filter)!=HAL_OK) return false;
    }
    return true;
}

class CanRuntime final : private balancing::CanEventSink {
public:
    CanRuntime(CAN_HandleTypeDef& can1, CAN_HandleTypeDef& can2, std::uint32_t cyclesPerUs,
               const balancing::MotorObservationConfig& config,bool selfDefinedCan2) noexcept
        : port1{can1}, port2{can2}, handle1{can1}, handle2{can2},
          motors_{config.rs06MappingVerified ? config.rs06 : balancing::Rs06BusConfig{},
                  config.wheelContinuity},
          link_{motors_, tx1_}, mappingVerified_{config.rs06MappingVerified && motors_.hasRs06Configuration()},
          selfDefinedCan2_{selfDefinedCan2}, cyclesPerUs_{cyclesPerUs}, lastCycle_{DWT->CYCCNT}
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
        balancing::CanRouter router{*this, &tx1_, &tx2_};
        // 片側の受信集中でも両バスを交互に回収し、有限回で戻る。
        balancing::pollCanEvents(&port1, &port2, router);
        // RX時刻と同じ時計を回収後に読む。回収中のIRQで取得した時刻が
        // 評価時刻より未来にならないよう、回収前の時刻では代用しない。
        const auto now = sampleClock();
        snapshot1.sampled_at_us = snapshot2.sampled_at_us = now;
        snapshot1.sampled = snapshot2.sampled = true;
        // 同じ評価時刻で全軸を固定。監視を読んだだけでは故障を消さない。
        pendingFaults_ = pendingFaults_ | motors_.takeObservationFaults(now);
        observation = balancing::MotorObservationSnapshot{
            motors_.feedback(now), ++observationGeneration_, snapshot1.bus_off, snapshot2.bus_off,
            pendingFaults_, mappingVerified_};
    }

    void bindApplication() noexcept
    {
        if (selfDefinedCan2_) balancing::bindApplicationRuntime(motors_,link_,tx1_,tx2_);
    }
    bool selfDefinedCan2() const noexcept { return selfDefinedCan2_; }

    void serviceCan2Tx() noexcept
    {
        if (!selfDefinedCan2_) return;
        if (auto* link=balancing::applicationCan2Link()) link->submitPending(tx2_,sampleClock());
    }

    bool acknowledge(std::uint64_t generation) noexcept
    {
        if (!observation || observation->generation != generation) return false;
        pendingFaults_ = balancing::FaultFlags::None;
        // コピーは次回回収まで変更しない。
        return true;
    }

    std::optional<balancing::MotorObservationSnapshot> observation{};
    balancing::CanPort port1;
    balancing::CanPort port2;
    CAN_HandleTypeDef& handle1;
    CAN_HandleTypeDef& handle2;
    BF446Can1Snapshot snapshot1{};
    BF446Can2Snapshot snapshot2{};

private:
    void onCanReceive(balancing::CanBus bus, balancing::CanRxTarget target, const balancing::CanFrame& frame,
                      std::uint64_t atUs) noexcept override
    {
        link_.onCanReceive(bus, target, frame, atUs);
        // ポート採取時刻はそのまま、評価時刻は配送時に同じDWTから採る。
        if (selfDefinedCan2_ && bus==balancing::CanBus::Can2)
            if (auto* link=balancing::applicationCan2Link())
                link->receive(bus,target,frame,atUs,sampleClock());
        // 監視値はポートが回収した受信の集計。機器の鮮度や正常応答の判定には使わない。
        if (bus == balancing::CanBus::Can1) {
            ++snapshot1.rx_frames;
            snapshot1.last_rx_us = atUs;
        } else {
            ++snapshot2.rx_frames;
            snapshot2.last_rx_us = atUs;
            if (target == balancing::CanRxTarget::C620LeftFeedback) {
                ++snapshot2.left_frames;
            } else if (target == balancing::CanRxTarget::C620RightFeedback) {
                ++snapshot2.right_frames;
            }
        }
    }

    void onCanTransmit(balancing::CanBus bus, balancing::TxToken token, balancing::CanTxOutcome outcome,
                       std::uint64_t atUs) noexcept override
    {
        // 受信専用。終端の配送だけで、送信候補は生成しない。
        link_.onCanTransmit(bus, token, outcome, atUs);
        balancing::drive_runtime::onTxTerminal(bus,token,outcome,atUs);
        if (selfDefinedCan2_ && bus==balancing::CanBus::Can2)
            if (auto* link=balancing::applicationCan2Link())
                link->onTxTerminal(bus,token,outcome,atUs);
    }

    void onCanPortStatus(balancing::CanBus bus, balancing::CanEventKind kind, std::uint64_t atUs) noexcept override
    {
        link_.onCanPortStatus(bus, kind, atUs);
        if (selfDefinedCan2_ && bus==balancing::CanBus::Can2)
            if (auto* link=balancing::applicationCan2Link())
                // 欠落通知は最初の欠落時刻を保持する。後続欠落もあり得るため、
                // ここでは回収時刻までの旧フレームを保守的に無効化する。
                link->onPortStatus(bus,kind,sampleClock());
        // 集計はバスごと。片側の欠落や復旧をもう片側へ伝播させない。
        if (bus == balancing::CanBus::Can1) {
            updateStatus(snapshot1, kind);
        } else {
            updateStatus(snapshot2, kind);
        }
    }

    template<class Snapshot>
    static void updateStatus(Snapshot& snapshot, balancing::CanEventKind kind) noexcept
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

    balancing::CanTxScheduler tx1_{port1};
    balancing::CanTxScheduler tx2_{port2};
    balancing::MotorService motors_;
    balancing::MotorCanLink link_;
    bool mappingVerified_;
    bool selfDefinedCan2_;
    std::uint64_t observationGeneration_{0};
    balancing::FaultFlags pendingFaults_{balancing::FaultFlags::None};
    std::uint32_t cyclesPerUs_;
    std::uint32_t lastCycle_;
    std::uint64_t accumulated_{0};
};

std::optional<CanRuntime> runtime;

// 通知有効化中のIRQからは構築済みポートを見せるが、両バス起動が完了するまで
// 主ループへの観測公開はしない。失敗後の不明なHAL状態からの再起動は拒否する。
bool started = false;
bool rollbackFailed = false;

} // namespace

extern "C" bool balancing_f446_can_start(CAN_HandleTypeDef* can1, CAN_HandleTypeDef* can2)
{
    return balancing::startCanObservation(can1, can2, {});
}

bool balancing::startCanObservation(CAN_HandleTypeDef* can1, CAN_HandleTypeDef* can2,
                                    const MotorObservationConfig& config,bool selfDefinedCan2) noexcept
{
    if (started) {
        return &runtime->handle1 == can1 && &runtime->handle2 == can2 &&
               runtime->selfDefinedCan2()==selfDefinedCan2;
    }
    if ((selfDefinedCan2 && !applicationCan2Link()) || rollbackFailed || runtime || can1 == nullptr || can2 == nullptr ||
        can1->Instance != CAN1 || can2->Instance != CAN2 ||
        HAL_CAN_GetState(can1) != HAL_CAN_STATE_READY ||
        HAL_CAN_GetState(can2) != HAL_CAN_STATE_READY ||
        SystemCoreClock == 0u || SystemCoreClock % microsecondsPerSecond != 0u ||
        (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u) {
        return false;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u ||
        !installCan1Filters(*can1) || !installCan2Filters(*can2,selfDefinedCan2)) {
        return false;
    }
    // HAL_CAN_Start失敗時もHALがLISTENINGへ移行済みの場合がある。
    // READYに戻っていなければ停止を試み、復帰不能なら再起動を禁止する。
    auto stopIfNeeded = [](CAN_HandleTypeDef* handle) {
        if (HAL_CAN_GetState(handle) != HAL_CAN_STATE_READY &&
            HAL_CAN_Stop(handle) != HAL_OK) {
            rollbackFailed = true;
        }
    };
    if (HAL_CAN_Start(can1) != HAL_OK) {
        stopIfNeeded(can1);
        return false;
    }
    if (HAL_CAN_Start(can2) != HAL_OK) {
        stopIfNeeded(can2);
        stopIfNeeded(can1);
        return false;
    }
    // NVICはCubeMXが先に有効化するため、構築途中のポートをIRQに見せない。
    const auto constructionMask = __get_PRIMASK();
    __disable_irq();
    runtime.emplace(*can1, *can2, SystemCoreClock / microsecondsPerSecond, config,selfDefinedCan2);
    __set_PRIMASK(constructionMask);
    if (HAL_CAN_ActivateNotification(can1, notifications) != HAL_OK ||
        HAL_CAN_ActivateNotification(can2, notifications) != HAL_OK) {
        // IRQからoptionalを参照できないよう、通知停止と破棄を同じ排他区間で行う。
        const auto mask = __get_PRIMASK();
        __disable_irq();
        __HAL_CAN_DISABLE_IT(can1, notifications);
        __HAL_CAN_DISABLE_IT(can2, notifications);
        runtime.reset();
        __set_PRIMASK(mask);
        stopIfNeeded(can2);
        stopIfNeeded(can1);
        return false;
    }
    started = true;
    runtime->bindApplication();
    return true;
}

extern "C" bool balancing_f446_can1_capture_irq(void)
{
    if (!runtime) {
        return false;
    }
    runtime->port1.captureInterrupt(runtime->sampleClock());
    return true;
}

extern "C" bool balancing_f446_can2_capture_irq(void)
{
    if (!runtime) {
        return false;
    }
    runtime->port2.captureInterrupt(runtime->sampleClock());
    return true;
}

extern "C" void balancing_f446_can_service(void)
{
    if (started) {
        runtime->service();
    }
}

extern "C" bool balancing_f446_can1_snapshot(BF446Can1Snapshot* snapshot)
{
    if (!started || snapshot == nullptr) {
        return false;
    }
    *snapshot = runtime->snapshot1;
    return true;
}

extern "C" bool balancing_f446_can2_snapshot(BF446Can2Snapshot* snapshot)
{
    if (!started || snapshot == nullptr) {
        return false;
    }
    *snapshot = runtime->snapshot2;
    return true;
}

std::optional<balancing::MotorObservationSnapshot> balancing::motorObservation() noexcept
{
    return started ? runtime->observation : std::nullopt;
}

bool balancing::acknowledgeMotorObservation(std::uint64_t generation) noexcept
{
    return started && runtime->acknowledge(generation);
}

std::optional<std::uint64_t> balancing::observationClockUs() noexcept
{
    if (!started) return std::nullopt;
    return runtime->sampleClock();
}

void balancing::serviceCan2ApplicationTx() noexcept
{
    if (started) runtime->serviceCan2Tx();
}
