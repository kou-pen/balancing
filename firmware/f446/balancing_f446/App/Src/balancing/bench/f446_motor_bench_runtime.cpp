#include "balancing_f446_bench_bridge.h"
#include "balancing/bench/MotorBench.hpp"

#ifdef BALANCING_BENCH_BUILD
namespace balancing {
namespace {
constexpr auto notifications=CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING |
    CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR;
// 実機IDや上限を推測で設定しない。検証済み設定への変更は別承認。
// このビルドにrequestPulseの入口は存在しない。
constexpr MotorBenchConfig boardConfig{};

bool filters(CAN_HandleTypeDef& handle,const MotorBenchConfig& cfg)
{
    const bool rs=cfg.axis==BenchAxis::Rs06;
    const unsigned first=rs?0u:14u;
    // 共有フィルタRAMの所属を維持し、選択バスの旧バンクも無効化する。
    for (unsigned i=0;i<14;++i) {
        CAN_FilterTypeDef f{};
        f.FilterBank=first+i;
        f.SlaveStartFilterBank=14;
        f.FilterMode=CAN_FILTERMODE_IDMASK;
        f.FilterScale=CAN_FILTERSCALE_32BIT;
        f.FilterFIFOAssignment=CAN_FILTER_FIFO0;
        const bool enabled=i<(rs?2u:1u);
        f.FilterActivation=enabled?ENABLE:DISABLE;
        if (enabled) {
            // RS06の状態/故障ビットだけはマスク。軸/主機IDとIDE/RTRは厳密。
            const std::uint32_t word=rs?
                (((i==0?2u:24u)<<24u | (std::uint32_t{cfg.motorId}<<8u) | cfg.hostId)<<3u)|CAN_RI0R_IDE:
                (0x200u+cfg.motorId)<<21u;
            const std::uint32_t mask=rs?
                ((0x1f00ffffu<<3u)|CAN_RI0R_IDE|CAN_RI0R_RTR):
                ((0x7ffu<<21u)|CAN_RI0R_IDE|CAN_RI0R_RTR);
            f.FilterIdHigh=word>>16u;f.FilterIdLow=word&0xffffu;
            f.FilterMaskIdHigh=mask>>16u;f.FilterMaskIdLow=mask&0xffffu;
        }
        if (HAL_CAN_ConfigFilter(&handle,&f)!=HAL_OK) return false;
    }
    return true;
}
struct Runtime {
    Runtime(CAN_HandleTypeDef& handle,const MotorBenchConfig& cfg) noexcept:
        bus{cfg.axis==BenchAxis::Rs06?1u:2u},clock{SystemCoreClock/1000000u},
        port{handle},bench{port,cfg} {}
    unsigned bus;
    CanClock clock;
    CanPort port;
    MotorBench bench;
    std::optional<std::uint32_t> sequence{};
    std::optional<std::uint64_t> lastUs{};
    CycleHealthMonitor health{};
    WatchdogPort watchdog{};
};
std::optional<Runtime> runtime;
bool started=false,rollbackFailed=false;
}
bool startCanBench(CAN_HandleTypeDef* can1,CAN_HandleTypeDef* can2,
                   const MotorBenchConfig& cfg) noexcept
{
    // 一度の起動に固定。通常起動との混用はビルド構成で排除する。
    if (runtime || started || rollbackFailed || !MotorBench::validConfig(cfg) ||
        !can1 || !can2 || can1->Instance!=CAN1 || can2->Instance!=CAN2 ||
        HAL_CAN_GetState(can1)!=HAL_CAN_STATE_READY ||
        HAL_CAN_GetState(can2)!=HAL_CAN_STATE_READY || !CanClock::enable()) return false;
    auto& selected=*(cfg.axis==BenchAxis::Rs06?can1:can2);
    if (!filters(selected,cfg)) return false;
    auto rollback=[&]{
        // HAL失敗時もLISTENING/ERRORに遷移済みの場合がある。
        if (HAL_CAN_GetState(&selected)!=HAL_CAN_STATE_READY &&
            HAL_CAN_Stop(&selected)!=HAL_OK) rollbackFailed=true;
    };
    if (HAL_CAN_Start(&selected)!=HAL_OK) { rollback();return false; }
    const auto mask=__get_PRIMASK();
    __disable_irq();
    runtime.emplace(selected,cfg);
    __set_PRIMASK(mask);
    // 通知中のIRQには構築済みポートのみ公開。主ループは成功後に開始。
    if (HAL_CAN_ActivateNotification(&selected,notifications)!=HAL_OK) {
        const auto previous=__get_PRIMASK();
        __disable_irq();
        __HAL_CAN_DISABLE_IT(&selected,notifications);
        runtime.reset();
        __set_PRIMASK(previous);
        rollback();
        return false;
    }
    started=true;
    return true;
}
bool bindBenchWatchdog(WatchdogConfig config) noexcept
{
    // 起動済みの選択バスにのみ結合する。周期開始後は上書きできない。
    if (!started || runtime->sequence || runtime->watchdog.summary().configured ||
        !config.verified || !config.refresh) return false;
    runtime->watchdog=WatchdogPort{config};
    return true;
}
std::optional<MotorBenchSummary> canBenchSummary() noexcept
{
    if (!started) return std::nullopt;
    return runtime->bench.summary(runtime->clock.sample());
}
}
extern "C" bool balancing_f446_bench_start(CAN_HandleTypeDef* c1,CAN_HandleTypeDef* c2)
{ return balancing::startCanBench(c1,c2,balancing::boardConfig); }

extern "C" bool balancing_f446_bench_capture_irq(std::uint8_t bus)
{
    using namespace balancing;
    if (!runtime || runtime->bus!=bus) return false;
    runtime->port.captureInterrupt(runtime->clock.sample());
    return true;
}
extern "C" void balancing_f446_bench_service(std::uint32_t sequence)
{
    using namespace balancing;
    if (!started) return;
    auto& r=*runtime;
    const auto now=r.clock.sample();
    // 周期番号とDWT進行を別に確認する。停止処理は故障後も継続する。
    if (r.sequence) {
        const auto delta=static_cast<std::uint32_t>(sequence-*r.sequence);
        // 既存6ms契約: 1回取りこぼしても6ms未満の処理は許容する。
        // 重複、逆順（unsignedで大差）、3周期以上、時計停止を故障とする。
        if (delta==0u || delta>=3u ||
            (r.lastUs && (now<=*r.lastUs || now-*r.lastUs>=6000u))) r.bench.fault();
    }
    const CycleTicket ticket{sequence,now};
    const bool begun=r.health.begin(ticket);
    r.sequence=sequence;r.lastUs=now;
    r.bench.poll(now);
    // CAN回収・停止処理を終えた周期のみ。FAULT中でも処理継続は健全。
    if (begun) {
        if (const auto proof=r.health.complete(ticket,r.clock.sample(),true))
            (void)r.watchdog.refresh(*proof);
    }
}
#endif
