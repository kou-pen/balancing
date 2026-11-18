#pragma once

#include "balancing/core/StateMachine.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace balancing {

enum class RobotState : std::uint8_t { Disarmed, Arming, Balancing, Fault };
enum class DesiredMotorState : std::uint8_t { StopAll, EnableRs06, DriveAllowed };
enum class Rs06ArmStatus : std::uint8_t { Pending, Confirmed, Failed };
enum class MechanismSafety : std::uint8_t { Unverified, Safe, Unsafe };

// セッション 0 はARM受理前。各軸の結果は同じセッションのスナップショットを渡す。
using ArmSessionId = std::uint64_t;

enum class FaultFlags : std::uint16_t {
    None = 0,
    EmergencyStop = 1U << 0,
    BusOff = 1U << 1,
    MotorFault = 1U << 2,
    ExternalFault = 1U << 3,
    EnableFailed = 1U << 4,
    MechanismUnsafe = 1U << 5,
    InterlockLost = 1U << 6,
    ClockRegression = 1U << 7,
    ControlTiming = 1U << 8,
    ArmTimeout = 1U << 9,
};

/** @brief 同じ周期に成立した複数の異常を結合する。 @spec{BEHAV-001} */
constexpr FaultFlags operator|(FaultFlags left, FaultFlags right) noexcept
{
    return static_cast<FaultFlags>(static_cast<std::uint16_t>(left) |
                                   static_cast<std::uint16_t>(right));
}

struct ArmTimeoutConfig {
    std::uint64_t durationUs{0};
    bool verified{false};
};

struct OperatorEvents {
    // ARM・DISARM・CLEAR_FAULT は呼び出し側で取り出した一周期限りのイベント。
    bool arm{false};
    bool disarm{false};
    bool clearFault{false};
    bool clearFaultAuthorized{false};
};

struct SequenceInterlocks {
    // 受信鮮度などは各機器の責務で評価し、今周期の判定結果を渡す。
    bool imuReady{false};
    bool motorsReady{false};
    bool powerReady{false};
    bool calibrationVerified{false};
    bool modelVerified{false};
    bool driveLimitsVerified{false};
    // 現在の機構安全判断。初期姿勢の指定範囲は独立した診断情報。
    MechanismSafety mechanism{MechanismSafety::Unverified};
};

struct Rs06ArmFeedback {
    ArmSessionId session{0};
    // Confirmed は該当軸で同じセッションのType 3 Tx完了後の正常応答があった場合。
    std::array<Rs06ArmStatus, 4> axes{};
    bool stopPending{false};
};

struct RobotSequenceInput {
    OperatorEvents operations{};
    SequenceInterlocks interlocks{};
    Rs06ArmFeedback rs06{};
    // 非常停止、バスオフ、制御周期異常などの同周期の判定結果。
    FaultFlags observedFaults{FaultFlags::None};
};

struct FaultRecord {
    // primaryCause は triggerFlags から優先順で選んだ一つのビット。
    FaultFlags primaryCause{FaultFlags::None};
    FaultFlags triggerFlags{FaultFlags::None};
    FaultFlags subsequentFlags{FaultFlags::None};
    std::uint64_t occurredAtUs{0};
};

struct RobotSequenceResult {
    RobotState state{RobotState::Disarmed};
    DesiredMotorState desiredMotorState{DesiredMotorState::StopAll};
    ArmSessionId armSession{0};
};

class RobotSequence final : public StateMachine<RobotState> {
public:
    /** @brief DISARMEDで初期化し、ARM期限設定を保持する。
     *  @param[in] timeout 検証済みの正の期限 [us]。無効な設定ではARMを受け付けない。
     *  @spec{BEHAV-001}
     */
    explicit RobotSequence(ArmTimeoutConfig timeout) noexcept;

    /** @brief 今周期の状態とモータへの要求を確定する。
     *  @param[in] input 操作エッジ、各機器の安全判断、セッション付きの4軸の結果。
     *  @param[in] nowUs 同一の単調時計で計った周期時刻 [us]。
     *  @return 現在状態、継続的な駆動要求、最後に受理したARMのセッション番号。
     *  @note StopAll は全4軸停止・車輪ゼロ電流の要求。送信状態は受け取り側が保持する。
     *  @spec{BEHAV-001}
     *  @spec{BOARD-F446-001}
     */
    RobotSequenceResult update(const RobotSequenceInput& input, std::uint64_t nowUs) noexcept;

    /** @brief ラッチ中のFAULT記録のコピーを返す。 @spec{BEHAV-001} */
    std::optional<FaultRecord> currentFault() const noexcept;

    /** @brief 明示解除された直近1件の記録のコピーを返す。 @spec{BEHAV-001} */
    std::optional<FaultRecord> lastResolvedFault() const noexcept;

private:
    RobotSequenceResult result() const noexcept;
    void latchFault(FaultFlags observed, std::uint64_t nowUs) noexcept;

    ArmTimeoutConfig timeout_;
    std::optional<std::uint64_t> previousUpdateUs_{};
    std::uint64_t armDeadlineUs_{0};
    ArmSessionId armSession_{0};
    std::optional<FaultRecord> currentFault_{};
    std::optional<FaultRecord> lastResolvedFault_{};
};

} // namespace balancing
