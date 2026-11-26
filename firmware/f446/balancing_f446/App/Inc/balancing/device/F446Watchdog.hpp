#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/** @brief 起動時のIWDG初期化結果。Disabledは正常な未有効化を表す。
 * @spec{BOARD-F446-001}
 */
typedef enum {
    BF446_WATCHDOG_DISABLED=0,
    BF446_WATCHDOG_STARTED=1,
    BF446_WATCHDOG_FAILED=2
} BF446WatchdogStartResult;
/** @brief 周辺初期化より前にRCCのリセット原因をコピーする。再取得しない。
 * @spec{BOARD-F446-001}
 */
void balancing_f446_watchdog_capture_reset_flags(void);
/** @brief 選択ビルドの起動完了後に固定設定で一回だけ開始を判断する。
 * @spec{BOARD-F446-001}
 * @warning 既定では無効。失敗後の再開始は不可。
 */
BF446WatchdogStartResult balancing_f446_watchdog_start(void);
#ifdef __cplusplus
}
#include "balancing/app/CycleHealthMonitor.hpp"
namespace balancing {
/** @brief 実機起動時の明示設定。500msは公称LSIでの候補値に過ぎない。
 * @spec{BOARD-F446-001}
 */
struct F446WatchdogConfig {
    bool enabled{},verified{};
    std::uint16_t prescaler{32},reload{499};
};
/** @brief 起動・リセット原因の診断。stickyな原因は複数同時成立し得る。
 * @spec{BOARD-F446-001}
 */
struct F446WatchdogSummary {
    BF446WatchdogStartResult state{BF446_WATCHDOG_DISABLED};
    std::uint32_t rawResetFlags{};
    bool captured{},hardwareMode{},debugFrozen{},attempted{},refreshFailed{};
    // HAL初期化を実行しなかった場合は空。
    std::optional<std::uint32_t> initResult{};
};
/** @brief mainの起動手順専用。対象ランタイムへの更新境界の結合後にHALを開始する。
 * @pre 対象ランタイムとTIM7起動済み。実行中の再設定禁止。
 * @spec{BOARD-F446-001}
 */
BF446WatchdogStartResult startF446Watchdog(const F446WatchdogConfig&) noexcept;
/** @brief 主ループ専用の起動診断コピー。再起動をまたぐFAULT保持ではない。
 * @spec{BOARD-F446-001}
 */
F446WatchdogSummary f446WatchdogSummary() noexcept;
}
#endif
