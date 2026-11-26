#pragma once
#include "balancing/core/CycleTicket.hpp"
#include <optional>

namespace balancing {
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
}
