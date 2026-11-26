#include "balancing/device/F446Watchdog.hpp"
#include "stm32f4xx_hal.h"
#ifdef BALANCING_BENCH_BUILD
#include "balancing/bench/MotorBench.hpp"
#else
#include "balancing/app/ApplicationObservation.hpp"
#endif

namespace balancing {
namespace {
constexpr F446WatchdogConfig boardConfig{}; // 実機承認までは無効
F446WatchdogSummary status{};
IWDG_HandleTypeDef handle{};
bool decided=false;

bool refresh(void*) noexcept
{
    // IWDGは開始後に停止できない。HAL異常を一度検出したら以後は更新せず、
    // リセットを妨げない。状態機械のFAULTとは別のハードウェア異常。
    if (status.state!=BF446_WATCHDOG_STARTED || status.refreshFailed) return false;
    if (HAL_IWDG_Refresh(&handle)!=HAL_OK) {
        status.refreshFailed=true;
        return false;
    }
    return true;
}
}

BF446WatchdogStartResult startF446Watchdog(const F446WatchdogConfig& cfg) noexcept
{
    // 初回判断がDisabledでも同じ起動で設定変更しない。
    // HAL_Initは開始キーを先に発行するため失敗後の再試行は特に禁止する。
    if (decided) return BF446_WATCHDOG_FAILED;
    decided=true;
    status.hardwareMode=(FLASH->OPTCR&FLASH_OPTCR_WDG_SW)==0u;
    status.debugFrozen=(DBGMCU->APB1FZ&DBGMCU_APB1_FZ_DBG_IWDG_STOP)!=0u;
    if (!status.captured || status.hardwareMode) {
        // ハードウェア自動起動設定は無効化できない。option byteは変更しない。
        return status.state=BF446_WATCHDOG_FAILED;
    }
    if (!cfg.enabled) return status.state=BF446_WATCHDOG_DISABLED;
    if (!cfg.verified || status.debugFrozen || cfg.prescaler!=32u || cfg.reload!=499u)
        return status.state=BF446_WATCHDOG_FAILED;

    // IWDG起動前に更新先を一意に確定。起動失敗時も結合を戻さない。
    const WatchdogConfig refreshConfig{nullptr,refresh,true};
#ifdef BALANCING_BENCH_BUILD
    if (!bindBenchWatchdog(refreshConfig)) return status.state=BF446_WATCHDOG_FAILED;
#else
    if (!bindApplicationWatchdog(refreshConfig)) return status.state=BF446_WATCHDOG_FAILED;
#endif
    // 保存済みの原因だけ消去し、次回のリセット原因を区別できるようにする。
    // 無効時・設定拒否時にはRCC CSRを変更しない。
    __HAL_RCC_CLEAR_RESET_FLAGS();
    handle.Instance=IWDG;
    handle.Init.Prescaler=IWDG_PRESCALER_32;
    handle.Init.Reload=cfg.reload;
    status.attempted=true;
    const auto result=HAL_IWDG_Init(&handle);
    status.initResult=static_cast<std::uint32_t>(result);
    // Initは内部で初回reloadする。以後のreloadは完了証跡からだけ。
    // Init失敗時もIWDGがすでに動作中の可能性があるためmainは停止する。
    return status.state=result==HAL_OK?BF446_WATCHDOG_STARTED:BF446_WATCHDOG_FAILED;
}

F446WatchdogSummary f446WatchdogSummary() noexcept { return status; }
}

extern "C" void balancing_f446_watchdog_capture_reset_flags(void)
{
    using namespace balancing;
    if (status.captured) return;
    // CSR全体を保存。IWDGRSTFだけで唯一の原因と断定しない。
    status.rawResetFlags=RCC->CSR;
    status.captured=true;
}
extern "C" BF446WatchdogStartResult balancing_f446_watchdog_start(void)
{
    return balancing::startF446Watchdog(balancing::boardConfig);
}
