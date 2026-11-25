#pragma once
#include "balancing/app/RobotSequence.hpp"
#include "balancing/communication/can/CanTypes.hpp"
#include <cstddef>

namespace balancing {
enum class DiagnosticKind : std::uint8_t { Fault, StateTransition };
struct FaultEvent {
    std::uint64_t eventSequence{0};
    std::uint32_t occurredAtMs{0};
    FaultRecord record{};
};
struct DiagnosticEvent {
    std::uint64_t sequence{0};
    DiagnosticKind kind{DiagnosticKind::StateTransition};
    std::uint32_t occurredAtMs{0};
    FaultEvent fault{};
    RobotState from{RobotState::Disarmed}, to{RobotState::Disarmed};
};
struct DiagnosticPart { DiagnosticEvent event{}; std::uint8_t frameIndex{0}; };
struct DiagnosticSlot {
    DiagnosticEvent event{};
    std::uint8_t nextIndex{0};
    std::optional<TxToken> accepted{};
};
struct DiagnosticStatus {
    std::uint64_t sampledAtUs{0};
    RobotState state{RobotState::Disarmed};
    std::optional<FaultRecord> fault{};
};
struct DiagnosticSummary {
    std::size_t importantPending{0}, normalPending{0};
    std::uint32_t importantLost{0}, normalLost{0};
    std::optional<DiagnosticStatus> latest{};
};
/** @brief 論理診断イベントの固定領域FIFO。CAN IDと符号化は別契約。
 * @spec{BOARD-F446-001}
 * @spec{BEHAV-001}
 */
class DiagnosticsService final {
public:
    /** @brief 呼出側の固定領域を使用。寿命中に再配置しない。ゼロ容量は保存なし。
     * @spec{BOARD-F446-001}
     */
    DiagnosticsService(DiagnosticSlot* important=nullptr,std::size_t importantSize=0,
                       DiagnosticSlot* normal=nullptr,std::size_t normalSize=0,
                       std::uint32_t tokenOrigin=0) noexcept;
    DiagnosticsService(const DiagnosticsService&)=delete;
    DiagnosticsService& operator=(const DiagnosticsService&)=delete;
    /** @brief 異常記録を3部分の論理イベントとして保留する。
     * @spec{BEHAV-001}
     */
    bool pushFault(const FaultRecord& record,std::uint64_t eventAtUs) noexcept;
    /** @brief 状態遷移を通常FIFOへ追加する。
     * @spec{BEHAV-001}
     */
    bool pushStateTransition(RobotState from,RobotState to,std::uint64_t nowUs) noexcept;
    /** @brief 周期結果を観測。同一FAULTと同一状態を連投しない。
     * @spec{BOARD-F446-001}
     */
    void updateStatus(const DiagnosticStatus& status) noexcept;
    /** @brief 未受付部分を重要優先で取得。受付済み重要項目は完了まで保持。
     * @spec{BOARD-F446-001}
     */
    std::optional<DiagnosticPart> nextPending() const noexcept;
    /** @brief メールボックス受付後だけ照合キーを登録する。
     * @spec{BOARD-F446-001}
     */
    bool onAccepted(const DiagnosticPart& part,TxToken token) noexcept;
    /** @brief Sentでのみ進捗を進める。失敗は新しいトークンで再試行。
     * @spec{BOARD-F446-001}
     */
    bool onTerminal(TxToken token,CanTxOutcome outcome) noexcept;
    /** @brief 最新状態とキュー・欠落数の読取コピー。
     * @spec{BOARD-F446-001}
     */
    DiagnosticSummary summary() const noexcept;
private:
    struct Queue {
        DiagnosticSlot* slots{};
        std::size_t capacity{}, head{}, count{};
        std::uint32_t lost{};
        DiagnosticSlot* front() const noexcept { return count ? &slots[head] : nullptr; }
    };
    bool push(Queue& queue,DiagnosticEvent event) noexcept;
    Queue important_,normal_;
    std::uint64_t eventSequence_{0}, lastSerial_{0};
    std::uint32_t origin_;
    std::optional<DiagnosticStatus> latest_{};
};
}
