#pragma once

#include "balancing/power/BoardIO.hpp"
#include "balancing/power/PowerCan.hpp"
#include "balancing/power/PowerMonitor.hpp"

namespace balancing::power {

/** @brief 起動時固定の設定。既定値は実機投入を禁止する。
 * @spec{BOARD-F303-001}
 */
struct ApplicationConfig {
    PowerConfig controller{};
    PowerMonitorConfig monitor{};
    ResetCause resetCause{ResetCause::AbnormalOrUnknown};
    bool powerPinsVerified{false}; ///< 実機検証までfalse
};

/** @brief タイマ周期ごとに採取した入力。IRQから直接Applicationを呼ばない。
 * @spec{BOARD-F303-001}
 */
struct CycleInput {
    std::uint32_t pendingTicks{0}; ///< 未消費の1 kHzタイマ通知数（飽和可）
    std::uint64_t nowUs{0}; ///< 単調時計 [us]
    PowerAdcSample adc{};
    PowerCanRx rx{};
    bool estopOk{false}; ///< 物理停止イベントを含む観測。短い停止もfalseで渡す
    bool hardwareFault{true};
    bool mainConfirmed{false}; ///< 実際の導通確認手段は未確定
    bool shutdownPending{true}; ///< IRQで遮断した未処理イベント
    PowerCommand shutdownCommand{PowerCommand::None}; ///< CAN OFF/ESTOPのみ。ONは禁止
};

/** @brief 一周期の結果。GPIO適用を確認できた候補だけ外部に返す。
 * @spec{BOARD-F303-001}
 */
struct CycleResult {
    bool ran{false};
    bool completed{false};
    bool watchdogEligible{false}; ///< 実際のIWDG更新は外部
    PowerOutput power{};
    BoardOutput board{};
    PowerCanOutput tx{}; ///< 完了時のみ送信候補。物理送信成功を意味しない
};

/** @brief 出力を適用する接続点。HAL未接続のためテストでは模擬基板を使用する。
 * @param context 呼出側所有の接続先。
 * @param output 論理出力。
 * @return 適用完了時true。false時は両電源を遮断済みであること。
 * @warning 実機実装ではIRQラッチ・現在入力・周期超過の再確認とGPIO書込みを
 * 不可分に行う。適用後のISRとCAN送信/IWDG更新の競合も呼出側で防ぐ。
 * @spec{BOARD-F303-001}
 */
using ApplyBoardOutput = bool (*)(void* context, const BoardOutput& output) noexcept;

/** @brief 4モジュールを1 kHzのメインループ処理へ接続する。
 * @spec{BOARD-F303-001}
 */
class Application {
public:
    /** @brief 起動固定設定を所有する。HAL・RTOS・動的確保に依存しない。
     * @spec{BOARD-F303-001}
     */
    explicit Application(ApplicationConfig config = {}) noexcept;

    /** @brief タイマ通知を一回消費し、観測→制御→出力適用→送信候補の順に処理する。
     * @param input IRQと排他して取得した周期入力。
     * @param apply 出力接続先。nullptrの場合は周期完了しない。
     * @param context 出力接続先のコンテキスト。
     * @return 完了とIWDG更新の論理許可。実時間・実GPIOは未検証。
     * @spec{BOARD-F303-001}
     */
    CycleResult runCycle(const CycleInput& input, ApplyBoardOutput apply,
                         void* context) noexcept;

private:
    PowerController controller_;
    PowerMonitor monitor_;
    PowerCan can_;
    BoardIO board_;
    const bool powerPinsVerified_;
    bool seenCycle_{false};
    std::uint64_t lastCycleUs_{0};
};

} // namespace balancing::power
