#pragma once

#include "balancing/communication/can/DiagnosticsCodec.hpp"
#include "balancing/communication/can/PowerBoardCodec.hpp"

namespace balancing::pi {

/** @brief Piが初期実装で受け付けるCAN v1の型。
 * 未確定のフラグは既存Codecの生値のまま保持する。
 * @spec{PROTO-REMOTE-001}
 * @spec{PROTO-PWR-001}
 */
using Incoming = std::variant<protocol::PowerStatusPayload, protocol::PowerResult,
    protocol::RemoteResult, protocol::NodeId, protocol::RobotStatusPayload,
    protocol::AttitudeAngle, protocol::AttitudeRate, protocol::YawRate,
    protocol::ChassisState, protocol::WheelState, protocol::LeftLegState,
    protocol::RightLegState, protocol::ControlTarget, protocol::FaultPart1,
    protocol::FaultPart2, protocol::FaultPart3>;

/** @brief CAN受信をPi宛て・全体通知だけに限定して復号する。
 * @param frame SocketCAN由来のID、IDE、DLC、データ。
 * @param remoteRequest RTRフラグ。
 * @param errorFrame エラーフレームフラグ。
 * @return 既存Codecで有効と判断した受信型。不正・対象外は空。
 * @warning 時刻・受信途絶・操作応答の照合は上位の責務。CAN IDは認証ではない。
 * @spec{PROTO-REMOTE-001}
 * @spec{PROTO-PWR-001}
 */
std::optional<Incoming> decodeIncoming(const CanFrame& frame, bool remoteRequest,
                                       bool errorFrame) noexcept;

/** @brief Piを送信元とする電源操作候補を生成。実機送信・操作許可は行わない。
 * @spec{PROTO-PWR-001}
 */
std::optional<CanFrame> powerRequest(protocol::PowerRequestKind kind,
                                     std::uint16_t requestId) noexcept;

/** @brief Pi起点の非常停止候補。送信完了や物理遮断を保証しない。
 * @spec{PROTO-PWR-001}
 */
std::optional<CanFrame> emergencyStop() noexcept;

} // namespace balancing::pi
