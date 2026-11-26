#pragma once
#include "balancing/app/PowerBoardService.hpp"
#include "balancing/app/RemoteControlService.hpp"
#include "balancing/app/DiagnosticsService.hpp"
#include "balancing/communication/can/DiagnosticsCodec.hpp"
#include "balancing/communication/can/PowerBoardCodec.hpp"

namespace balancing {

/** @brief 非駆動のCAN2試験設定。未設定では通信を開始しない。
 * @spec{PROTO-REMOTE-001}
 */
struct Can2ServiceConfig {
    std::uint64_t requestLifetimeUs{}, basePeriodUs{}, telemetryLifetimeUs{};
    std::uint16_t minimumDetailPeriodMs{}, maximumLeaseMs{};
    std::size_t responseCapacity{}; // 1〜8。実機の容量は未決定
    std::uint32_t tokenOrigin{};
    bool allowMockPowerRequests{false};
};
struct Can2CycleInput {
    OperatorEvents operations{};
    FaultFlags faults{FaultFlags::None};
};
struct Can2ServiceSummary {
    std::uint32_t droppedResponses{};
    std::optional<protocol::PowerResult> powerResult{};
};
using Can2Telemetry = std::array<std::optional<CanFrame>,10>;

/** @brief CAN2の自作通信を既存サービスと一周期の状態機械に接続する。
 * @warning 非駆動検証用。物理安全判断や実機電源操作の権限を持たない。
 * @spec{BOARD-F446-001}
 * @spec{PROTO-REMOTE-001}
 * @spec{PROTO-PWR-001}
 */
class Can2ServiceLink final {
public:
    /** @brief 長寿命の既存サービスを参照。設定不成立時は無効。
     * @spec{BOARD-F446-001}
     */
    Can2ServiceLink(PowerBoardService&,RemoteControlService&,DiagnosticsService&,
                    Can2ServiceConfig={}) noexcept;
    /** @brief 明示設定が成立している場合だけtrue。
     * @spec{BOARD-F446-001}
     */
    bool enabled() const noexcept { return enabled_; }
    /** @brief CAN2内部送信トークンの発行元。ワイヤIDではない。
     * @spec{BOARD-F446-001}
     */
    std::uint32_t tokenOrigin() const noexcept { return config_.tokenOrigin; }
    /** @brief Routerの主ループ配送。受信時刻と評価時刻は共通時計[us]。
     * @spec{PROTO-REMOTE-001}
     */
    void receive(CanBus,CanRxTarget,const CanFrame&,std::uint64_t receivedAtUs,
                 std::uint64_t nowUs) noexcept;
    /** @brief CAN2欠落時に旧入力・未処理の非停止要求を破棄。
     * @spec{BOARD-F446-001}
     */
    void onPortStatus(CanBus,CanEventKind,std::uint64_t atUs) noexcept;
    /** @brief 一周期の入力を取得。解除許可は上位が決める。
     * @pre completeCycleと対で一度ずつ呼ぶ。
     * @spec{BEHAV-001}
     */
    Can2CycleInput takeCycleInput(std::uint64_t nowUs,RobotState state,
                                  bool clearFaultAuthorized=false) noexcept;
    /** @brief 同周期の状態機械結果から操作応答を確定。
     * @spec{BEHAV-001}
     */
    void completeCycle(const RobotSequenceResult&,std::uint64_t nowUs) noexcept;
    /** @brief 有効観測の符号化済み最新10フレームで全面置換。空欄は送らない。
     * @spec{PROTO-REMOTE-001}
     */
    void publishTelemetry(const Can2Telemetry&,std::uint64_t nowUs) noexcept;
    /** @brief 背景枠へ最大1件投入。未受付時に古い通常データを積まない。
     * @spec{BOARD-F446-001}
     */
    void submitPending(CanTxScheduler&,std::uint64_t nowUs) noexcept;
    /** @brief CAN2の照合済み終端だけを消費。
     * @spec{BOARD-F446-001}
     */
    void onTxTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t atUs) noexcept;
    /** @brief 模擬試験用の電源要求。既定で拒否し、未応答要求を自動再送しない。
     * @spec{PROTO-PWR-001}
     */
    bool requestMockPower(protocol::PowerRequestKind,std::uint16_t,std::uint64_t nowUs) noexcept;
    /** @brief 応答欠落数と照合済み電源応答のコピー。ON通知の代用ではない。
     * @spec{PROTO-PWR-001}
     */
    Can2ServiceSummary summary() const noexcept { return {droppedResponses_,powerResult_}; }
private:
    struct Request { protocol::RemoteRequest value; std::uint64_t atUs; };
    struct Response { CanFrame frame; std::uint64_t atUs; };
    struct Lease { std::uint16_t periodMs{}; std::uint64_t untilUs{}; };
    struct PowerExchange {
        protocol::PowerRequestKind kind;
        std::uint16_t id;
        std::uint64_t atUs;
        std::optional<TxToken> token{};
        std::optional<std::uint64_t> sentAtUs{};
    };
    bool recent(std::uint64_t atUs,std::uint64_t nowUs) const noexcept;
    void queueResponse(std::optional<CanFrame>,std::uint64_t nowUs) noexcept;
    void remoteResult(const Request&,protocol::ResultCode,RobotState,std::uint64_t nowUs) noexcept;
    void handleDiagnostic(const CanFrame&,std::uint64_t nowUs) noexcept;
    void removeResponse() noexcept;
    void countDroppedResponse() noexcept;
    PowerBoardService& power_;
    RemoteControlService& remote_;
    DiagnosticsService& diagnostics_;
    Can2ServiceConfig config_;
    bool enabled_{};
    bool busOff_{};
    bool emergencyPending_{};
    std::optional<std::uint64_t> rxBarrier_{}, motionBarrier_{};
    std::optional<Request> pending_{}, disarm_{}, executing_{}, executingDisarm_{};
    RobotState previousState_{RobotState::Disarmed};
    std::array<Response,8> responses_{};
    std::size_t responseCount_{};
    std::uint32_t droppedResponses_{};
    std::optional<TxToken> responseInFlight_{};
    Can2Telemetry telemetry_{};
    std::optional<std::uint64_t> telemetryAtUs_{};
    std::array<Lease,10> leases_{};
    std::array<std::optional<std::uint64_t>,10> lastTelemetryTx_{};
    std::size_t nextTelemetry_{};
    std::uint64_t serial_{};
    std::optional<PowerExchange> powerExchange_{};
    std::optional<protocol::PowerResult> powerResult_{};
};
}
