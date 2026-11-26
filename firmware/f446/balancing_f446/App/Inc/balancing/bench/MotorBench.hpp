#pragma once
#include "balancing/communication/can/CanPort.hpp"
#include "balancing/app/CycleHealthMonitor.hpp"
#include "balancing/communication/can/CanRouter.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"
#include "balancing/device/Rs06Motor.hpp"
#include "balancing/device/C620Motor.hpp"

namespace balancing {
/** @brief 単独の試運転対象。通常の4脚2輪の状態機械とは独立。
 * @spec{BEHAV-001}
 */
enum class BenchAxis : std::uint8_t { Rs06, C620 };
enum class BenchState : std::uint8_t { Disabled, Stopping, Idle, Enabling, Pulsing, Fault };

/** @brief 試運転対象1台の明示設定。旧ベンチの0.5 Nm/A・200ms上限を維持。
 * @note limitの単位はRS06=Nm、C620=A。CAN2の他3チャネルは常にゼロ。
 * @spec{BOARD-F446-001}
 * @spec{PROTO-RS06-001}
 * @spec{PROTO-C620-001}
 */
struct MotorBenchConfig {
    BenchAxis axis{BenchAxis::Rs06};
    std::uint8_t motorId{},hostId{};
    float limit{};
    std::uint16_t pulseDurationMs{};
    std::uint64_t txTimeoutUs{};
    bool verified{};
};
struct MotorBenchSummary {
    BenchState state{BenchState::Disabled};
    bool stopSent{},feedbackFresh{},busOff{},inFlight{};
    std::uint32_t txFailures{};
};

/** @brief 独立した単軸試運転。既存CAN Port/Router/Schedulerと機器状態を使用。
 * @warning 専用ポート・専用IRQ経路が必要。通常Applicationと同じCANの所有権を共有しない。
 * 初期化・構築は送信しない。明示試験開始を許可された隔離構成専用。
 * @spec{BOARD-F446-001}
 * @spec{BEHAV-001}
 */
class MotorBench final : private CanEventSink {
public:
    /** @brief 明示設定の形式検証。実機安全の証明ではない。
     * @spec{BOARD-F446-001}
     */
    static bool validConfig(const MotorBenchConfig&) noexcept;
    MotorBench(CanPort& exclusivePort,MotorBenchConfig config={}) noexcept;
    MotorBench(const MotorBench&)=delete;
    MotorBench& operator=(const MotorBench&)=delete;
    /** @brief 初期停止確認と新規フィードバックを満たす場合のみ一回のパルスを要求。
     * @spec{BEHAV-001}
     */
    bool requestPulse(float output,std::uint64_t nowUs) noexcept;
    /** @brief CAN回収・期限確認・最大1件の送信を主ループで進める。
     * @spec{BOARD-F446-001}
     */
    void poll(std::uint64_t nowUs) noexcept;
    /** @brief 旧非ゼロの終端を待ち、Type4または電流ゼロを優先する。FAULTは解除しない。
     * @spec{BEHAV-001}
     */
    void cancel() noexcept;
    /** @brief 時計・基板側の異常を停止方向へラッチする。
     * @spec{BEHAV-001}
     */
    void fault() noexcept;
    /** @brief 試験・通信状態の診断コピー。停止送達は物理停止を保証しない。
     * @spec{BOARD-F446-001}
     */
    MotorBenchSummary summary(std::uint64_t nowUs) const noexcept;
private:
    enum class TxKind : std::uint8_t { Stop,Enable,Pulse };
    struct Pending {
        TxToken token{};
        TxKind kind{};
        std::optional<Rs06TxToken> rsToken{};
        std::uint64_t atUs{};
        bool accepted{},abortRequested{};
    };
    CanBus bus() const noexcept;
    bool active() const noexcept;
    bool fresh(std::uint64_t nowUs) const noexcept;
    void stop(bool fault) noexcept;
    void submitStop(std::uint64_t nowUs) noexcept;
    void submitDrive(const CanFrame&,TxKind,std::optional<Rs06TxToken>,
                     std::uint64_t nowUs) noexcept;
    void onCanReceive(CanBus,CanRxTarget,const CanFrame&,std::uint64_t) noexcept override;
    void onCanTransmit(CanBus,TxToken,CanTxOutcome,std::uint64_t) noexcept override;
    void onCanPortStatus(CanBus,CanEventKind,std::uint64_t) noexcept override;
    CanPort& port_;
    MotorBenchConfig config_;
    CanTxScheduler scheduler_;
    CanRouter router_;
    Rs06Motor rs06_;
    C620Motor c620_{};
    BenchState state_{BenchState::Disabled};
    std::optional<Pending> pending_{};
    std::optional<std::uint64_t> lastPoll_{},lastDue_{},lastPulseSent_{};
    std::uint64_t requestedAt_{},pulseAt_{},serial_{};
    std::uint32_t txFailures_{};
    float output_{};
    bool stopSent_{},busOff_{};
};
/** @brief 試運転ビルド専用の停止確認ランタイム。起動時のみ。非ゼロ要求を公開しない。
 * @pre 通常Applicationは起動しない。両CANは初期化済み、未開始。
 * @spec{BOARD-F446-001}
 */
bool startCanBench(CAN_HandleTypeDef* can1,CAN_HandleTypeDef* can2,
                   const MotorBenchConfig& config) noexcept;
/** @brief 試運転の周期開始前に更新ポートを一度だけ結合。
 * @spec{BOARD-F446-001}
 */
bool bindBenchWatchdog(WatchdogConfig) noexcept;
/** @brief 試運転の状態コピー。起動失敗時は空。主ループのみ。
 * @spec{BOARD-F446-001}
 */
std::optional<MotorBenchSummary> canBenchSummary() noexcept;
}
