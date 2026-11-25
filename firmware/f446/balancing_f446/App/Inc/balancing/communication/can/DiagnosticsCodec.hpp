#pragma once
#include "balancing/communication/can/RemoteControlCodec.hpp"
#include <variant>

namespace balancing::protocol {

/** @brief 状態・操作元・未確定状態フラグ・現在故障・未確定観測有効性。
 * @spec{PROTO-REMOTE-001}
 */
struct RobotStatusPayload {
    WireRobotState state;
    WireInputSource source;
    std::uint16_t flags;
    std::uint16_t faultBits;
    std::uint16_t validity;
};

/** @brief Pitch/Roll角 [rad]。
 * @spec{PROTO-REMOTE-001}
 */
struct AttitudeAngle {
    float pitchRad;
    float rollRad;
};

/** @brief Pitch/Roll角速度 [rad/s]。
 * @spec{PROTO-REMOTE-001}
 */
struct AttitudeRate {
    float pitchRadS;
    float rollRadS;
};

/** @brief Yaw角速度 [rad/s]。
 * @spec{PROTO-REMOTE-001}
 */
struct YawRate {
    float yawRadS;
};

/** @brief 前後位置 [m] と速度 [m/s]。
 * @spec{PROTO-REMOTE-001}
 */
struct ChassisState {
    float positionM;
    float velocityMps;
};

/** @brief 左右車輪角速度 [rad/s]。
 * @spec{PROTO-REMOTE-001}
 */
struct WheelState {
    float leftRadS;
    float rightRadS;
};

/** @brief 左脚長 [m] と角度 [rad]。
 * @spec{PROTO-REMOTE-001}
 */
struct LeftLegState {
    float lengthM;
    float angleRad;
};

/** @brief 右脚長 [m] と角度 [rad]。
 * @spec{PROTO-REMOTE-001}
 */
struct RightLegState {
    float lengthM;
    float angleRad;
};

/** @brief 前後速度目標 [m/s] とYaw角速度目標 [rad/s]。
 * @spec{PROTO-REMOTE-001}
 */
struct ControlTarget {
    float velocityMps;
    float yawRateRadS;
};

/** @brief 実測周期 [us] と超過累計。
 * @spec{PROTO-REMOTE-001}
 */
struct ControlTiming {
    std::uint32_t periodUs;
    std::uint32_t overrunCount;
};

/** @brief イベント番号・イベント時刻 [ms]・主原因単一ビット。
 * @spec{PROTO-REMOTE-001}
 */
struct FaultPart1 {
    std::uint16_t eventId;
    std::uint32_t eventAtMs;
    std::uint16_t primaryCause;
};

/** @brief イベント番号・発生時と追加異常のビット。
 * @spec{PROTO-REMOTE-001}
 */
struct FaultPart2 {
    std::uint16_t eventId;
    std::uint16_t triggerFlags;
    std::uint16_t subsequentFlags;
};

/** @brief イベント番号・元FAULT発生時刻 [ms]。
 * @spec{PROTO-REMOTE-001}
 */
struct FaultPart3 {
    std::uint16_t eventId;
    std::uint32_t originalAtMs;
};

/** @brief 詳細購読要求。周期・期限 [ms] は上位で検査。
 * @spec{PROTO-REMOTE-001}
 */
struct StreamSubscribe {
    std::uint16_t registerId;
    std::uint16_t periodMs;
    std::uint16_t leaseMs;
    std::uint16_t requestId;
};

/** @brief 購読解除要求。
 * @spec{PROTO-REMOTE-001}
 */
struct StreamUnsubscribe {
    std::uint16_t registerId;
    std::uint16_t requestId;
};

/** @brief 購読操作の応答。
 * @spec{PROTO-REMOTE-001}
 */
struct StreamResult {
    std::uint16_t requestId;
    std::uint16_t requestRegisterId;
    ResultCode result;
};

/** @brief 設定番号の読み出し要求。
 * @spec{PROTO-REMOTE-001}
 */
struct ParamRead {
    std::uint16_t paramId;
    std::uint16_t requestId;
};

/** @brief 設定の生32bit値。型は番号別定義まで未確定。
 * @spec{PROTO-REMOTE-001}
 */
struct ParamValue {
    std::uint16_t paramId;
    std::uint16_t requestId;
    std::uint32_t rawBits;
};

/** @brief 設定読み出しエラー。
 * @spec{PROTO-REMOTE-001}
 */
struct ParamError {
    std::uint16_t paramId;
    std::uint16_t requestId;
    ResultCode result;
};

/** @brief 診断・設定読み出しの19種類。受信IDで型を区別する。
 * @spec{PROTO-REMOTE-001}
 */
using DiagnosticPayload = std::variant<RobotStatusPayload,
    AttitudeAngle,
    AttitudeRate,
    YawRate,
    ChassisState,
    WheelState,
    LeftLegState,
    RightLegState,
    ControlTarget,
    ControlTiming,
    FaultPart1,
    FaultPart2,
    FaultPart3,
    StreamSubscribe,
    StreamUnsubscribe,
    StreamResult,
    ParamRead,
    ParamValue,
    ParamError>;

/** @brief RobotStatusPayloadの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(RobotStatusPayload value) noexcept;
/** @brief AttitudeAngleの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(AttitudeAngle value) noexcept;
/** @brief AttitudeRateの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(AttitudeRate value) noexcept;
/** @brief YawRateの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(YawRate value) noexcept;
/** @brief ChassisStateの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(ChassisState value) noexcept;
/** @brief WheelStateの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(WheelState value) noexcept;
/** @brief LeftLegStateの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(LeftLegState value) noexcept;
/** @brief RightLegStateの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(RightLegState value) noexcept;
/** @brief ControlTargetの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(ControlTarget value) noexcept;
/** @brief ControlTimingの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(ControlTiming value) noexcept;
/** @brief FaultPart1の符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(FaultPart1 value) noexcept;
/** @brief FaultPart2の符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(FaultPart2 value) noexcept;
/** @brief FaultPart3の符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(FaultPart3 value) noexcept;
/** @brief StreamSubscribeの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(StreamSubscribe value) noexcept;
/** @brief StreamUnsubscribeの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(StreamUnsubscribe value) noexcept;
/** @brief StreamResultの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(StreamResult value) noexcept;
/** @brief ParamReadの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(ParamRead value) noexcept;
/** @brief ParamValueの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(ParamValue value) noexcept;
/** @brief ParamErrorの符号化。 @spec{PROTO-REMOTE-001} */
std::optional<CanFrame> encodeDiagnostic(ParamError value) noexcept;

/** @brief 固定ID・IDE・DLC・値・予約領域を検査し対応型へ復号する。
 * @warning 観測鮮度、購読許可、設定番号の有効性、FAULT再構成は上位の責務。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<DiagnosticPayload> decodeDiagnostic(const CanFrame& frame) noexcept;

} // namespace balancing::protocol
