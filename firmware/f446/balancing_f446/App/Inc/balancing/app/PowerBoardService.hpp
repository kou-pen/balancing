#pragma once
#include "balancing/app/RobotSequence.hpp"
#include "balancing/communication/can/CanRouter.hpp"

namespace balancing {
enum class PowerState : std::uint8_t { Off, Precharge, On, Fault };
/** @brief ワイヤの未解釈データ。状態・故障ビットの意味は未確定。
 * @spec{PROTO-PWR-001}
 */
struct PowerWireMetadata {
    std::uint8_t flags;
    std::uint16_t faultBits;
    std::uint16_t sequence;
};
/** @brief 電源状態とポート採取時刻 [us]。従来の模擬入力ではwireは空。
 * @spec{PROTO-PWR-001}
 */
struct PowerStatusReport {
    PowerState state;
    std::uint64_t receivedAtUs;
    std::optional<PowerWireMetadata> wire{};
};
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
    /** @brief CAN2のPOWER_STATUSを復号し、既存の状態・鮮度管理へ渡す。
     * @param bus イベントの発生元。
     * @param target CanRouterの受信分類。
     * @param frame 採取済みのCANデータフレーム。
     * @param receivedAtUs ポートが採取した時刻 [us]。
     * @param nowUs 主ループの評価時刻 [us]。
     * @return 有効かつ新しい受信ならtrue。拒否時は保持状態を変更しない。
     * @warning 連番の再起動・周回契約は未確定。実機ARM経路には接続しない。
     * @spec{PROTO-PWR-001}
     */
    bool acceptCanStatus(CanBus bus, CanRxTarget target, const CanFrame& frame,
                         std::uint64_t receivedAtUs, std::uint64_t nowUs) noexcept;
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
