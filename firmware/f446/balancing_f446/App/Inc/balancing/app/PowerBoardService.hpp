#pragma once
#include "balancing/app/RobotSequence.hpp"

namespace balancing {
enum class PowerState : std::uint8_t { Off, Precharge, On, Fault };
struct PowerStatusReport { PowerState state; std::uint64_t receivedAtUs; };
struct PowerBoardSnapshot {
    std::optional<PowerStatusReport> latest{};
    bool fresh{false};
    bool ready{false};
    bool faultObserved{false};
};
/** @brief F303の復号済み状態と500ms受信期限を管理する。電源操作は行わない。
 * @spec{PROTO-PWR-001}
 * @spec{BOARD-F446-001}
 */
class PowerBoardService final {
public:
    /** @brief 受信元と新規性を通信層が確認した報告のみ受理。
     * @spec{PROTO-PWR-001}
     */
    bool acceptDecoded(PowerStatusReport report, std::uint64_t nowUs) noexcept;
    /** @brief 同一周期時刻における電源状態。ONかつ鮮度成立のみready。
     * @spec{BOARD-F446-001}
     */
    PowerBoardSnapshot snapshot(std::uint64_t nowUs) const noexcept;
    /** @brief 明示FAULTの保留通知を消費する。上位FAULT解除ではない。
     * @spec{BEHAV-001}
     */
    FaultFlags takeObservedFaults(std::uint64_t nowUs) noexcept;
private:
    std::optional<PowerStatusReport> latest_{};
    bool faultPending_{false};
};
}
