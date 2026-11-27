#pragma once
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/app/MotorCanLink.hpp"
#include "balancing/communication/can/MotorFramePlan.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

namespace balancing {

/** @brief 駆動送達監視の明示期限。既定0は非駆動。
 * @spec{PROTO-RS06-001}
 * @spec{PROTO-C620-001}
 */
struct DriveExecutionConfig {
    std::uint64_t rs06TxTimeoutUs{}, c620TxTimeoutUs{};
};

/** @brief 共通周期の結果。フレームは数学fixtureのプレビュー。
 * @spec{BOARD-F446-001}
 */
struct DriveExecutionResult {
    ControlTickResult control{};
    std::optional<MotorFramePreview> frames{};
    bool outputInhibited{true};
};

/** @brief 状態機械とモータ送信を接続する共通周期実行。
 * @note 通常構築ではフレーム生成なし、StopAllとゼロのみ。実機非ゼロ出力は未接続。
 * @spec{BEHAV-001}
 * @spec{BOARD-F446-001}
 */
class DriveExecution final {
public:
    /** @brief 起動時固定の数学フレーム生成境界。通常運転からは渡さない。
     * @spec{BOARD-F446-001}
     */
    using Compose=MotorFramePreview(*)(const RobotSequenceResult&,const MotorCommandPreview&) noexcept;
    /** @brief 既存所有者を借用する。生成関数なしなら停止専用。
     * @spec{BOARD-F446-001}
     */
    DriveExecution(RobotSequence&,MotorService&,MotorCanLink&,
                   CanTxScheduler&,CanTxScheduler&,DriveExecutionConfig={},Compose=nullptr) noexcept;
    /** @brief 周期を一度だけ評価し、現在の要求と送信を接続。
     * @spec{BEHAV-001}
     */
    DriveExecutionResult step(ControlCycle&,const ApplicationObservation&,ControlTickInput) noexcept;
    /** @brief 新周期指令を増やさず保留送信を進める。
     * @spec{BOARD-F446-001}
     */
    void poll(std::uint64_t nowUs) noexcept;
    /** @brief 照合済み終端のみ配送。
     * @spec{BOARD-F446-001}
     */
    void onTxTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t atUs) noexcept;
    /** @brief 既存停止専用の診断コピー。
     * @spec{BOARD-F446-001}
     */
    StopOutputSummary stopSummary() const noexcept;
private:
    void withdraw() noexcept;
    bool queue(CanBus,const MotorFramePreview&,std::uint64_t nowUs) noexcept;
    void service(CanBus,std::uint64_t nowUs) noexcept;
    FaultFlags txFaults(std::uint64_t nowUs) const noexcept;
    void fault(FaultFlags) noexcept;
    bool acceptStopCycle(const ApplicationObservation&,std::uint64_t nowUs) noexcept;
    RobotSequence& sequence_;
    MotorService& motors_;
    MotorCanLink& link_;
    MotorPeriodicTx tx_;
    DriveExecutionConfig config_;
    Compose compose_;
    std::optional<std::uint64_t> activeSince_{},lastRsDue_{},lastTick_{};
    std::optional<CycleTicket> lastTicket_{};
    std::optional<std::uint64_t> lastGeneration_{},lastClock_{};
    FaultFlags pending_{FaultFlags::None},outputFaults_{FaultFlags::None};
    bool active_{false},configured_{false},cycleSeen_{false};
};
}
