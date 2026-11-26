#pragma once
#include <cstdint>
#include <optional>

namespace balancing {
/** @brief TIM7イベント番号と主ループ開始時刻。番号は32bit周回を許容。
 * @spec{BOARD-F446-001}
 */
struct CycleTicket {
    std::uint32_t eventSequence{};
    std::uint64_t startedAtUs{};
};
/** @brief 完了した一周期の証跡。実行終了時刻は開始時刻以上。
 * @spec{BOARD-F446-001}
 */
struct CycleCompletion {
    CycleTicket ticket{};
    std::uint64_t completedAtUs{};
};

/** @brief 周期処理の診断コピー。通信故障や駆動許可そのものではない。
 * @spec{BOARD-F446-001}
 */
struct CycleHealthSnapshot {
    std::optional<CycleTicket> lastStarted{};
    std::optional<CycleCompletion> lastCompleted{};
    std::uint32_t duplicateEvents{},oldEvents{},missedEvents{},clockRegressions{};
    std::uint32_t incompleteCycles{},completedCycles{};
    bool inProgress{},refreshEligible{};
};
/** @brief 主ループの開始と完了を照合する。ISRから呼ばない。
 * @spec{BOARD-F446-001}
 */
class CycleHealthMonitor final {
public:
    /** @brief 新しい番号を受け付ける。欠落/未完了直後の周期は更新証跡を出さない。
     * @spec{BOARD-F446-001}
     */
    bool begin(CycleTicket ticket) noexcept;
    /** @brief 同一ticketの完了のみ受理し、健全時に一回だけ証跡を返す。
     * @spec{BOARD-F446-001}
     */
    std::optional<CycleCompletion> complete(CycleTicket ticket,std::uint64_t completedAtUs,
                                            bool processingCompleted) noexcept;
    /** @brief 時計取得不能などにより処理完了を証明できない。
     * @spec{BOARD-F446-001}
     */
    void abandon() noexcept;
    /** @brief 主ループで診断コピーを取得。
     * @spec{BOARD-F446-001}
     */
    CycleHealthSnapshot summary() const noexcept { return status_; }
private:
    CycleHealthSnapshot status_{};
    bool eligible_{false};
    std::optional<std::uint64_t> clockHighWater_{};
};

/** @brief 明示的に構成済みの更新専用ポート。IWDGの開始は行わない。
 * @spec{BOARD-F446-001}
 */
struct WatchdogConfig {
    using Refresh = bool(*)(void*) noexcept;
    void* context{};
    Refresh refresh{};
    bool verified{};
};
/** @brief 更新要求とコールバック結果の診断。実機リセット期限の保証ではない。
 * @spec{BOARD-F446-001}
 */
struct WatchdogSummary {
    bool configured{};
    std::uint32_t attempts{},successes{},failures{},rejected{};
};
/** @brief 健全な新規完了証跡を一回だけ更新要求へ変換する。
 * @spec{BOARD-F446-001}
 */
class WatchdogPort final {
public:
    explicit WatchdogPort(WatchdogConfig config={}) noexcept : config_{config} {}
    /** @brief 旧/重複/逆行証跡を拒否。未設定なら無出力。
     * @spec{BOARD-F446-001}
     */
    bool refresh(const CycleCompletion& completion) noexcept;
    /** @brief 最後の更新結果を取得。
     * @spec{BOARD-F446-001}
     */
    WatchdogSummary summary() const noexcept;
private:
    WatchdogConfig config_{};
    WatchdogSummary status_{};
    std::optional<CycleCompletion> last_{};
};
}
