#pragma once
#include "balancing/communication/can/ProtocolId.hpp"
#include "balancing/communication/can/ProtocolPayload.hpp"

namespace balancing::protocol {

/** @brief Piの運動目標。有限値のみ。物理限界は上位で検査する。
 * @spec{PROTO-REMOTE-001}
 */
struct MotionCommand {
    float forwardVelocityMps;
    float yawRateRadS;
};

/** @brief イベント操作のRegister ID。
 * @spec{PROTO-REMOTE-001}
 */
enum class RemoteRequestKind : std::uint16_t { Arm=2, Disarm=3, ClearFault=4, SelectSource=5 };

/** @brief ワイヤ上の操作元。実際の選択状態とは独立。
 * @spec{PROTO-REMOTE-001}
 */
enum class WireInputSource : std::uint8_t { UsbDualSense=0, CanRemote=1 };

/** @brief ワイヤ上のロボット状態。
 * @spec{PROTO-REMOTE-001}
 */
enum class WireRobotState : std::uint8_t { Disarmed=0, Arming=1, Balancing=2, Fault=3 };

/** @brief 操作要求。sourceはSelectSourceの場合のみ必須。
 * @spec{PROTO-REMOTE-001}
 */
struct RemoteRequest {
    RemoteRequestKind kind;
    std::uint16_t requestId;
    std::optional<WireInputSource> source{};
};

/** @brief 要求への応答。Acceptedは駆動成立ではない。
 * @spec{PROTO-REMOTE-001}
 */
struct RemoteResult {
    RemoteRequestKind kind;
    std::uint16_t requestId;
    ResultCode result;
    WireRobotState state;
};

/** @brief 運動目標を有限float32二個のフレームに変換。不正値なら空。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<CanFrame> encodeMotionCommand(MotionCommand value) noexcept;
/** @brief 固定ID・IDE・DLC・有限値を検査し復号。鮮度・操作許可は判定しない。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<MotionCommand> decodeMotionCommand(const CanFrame& frame) noexcept;
/** @brief 操作要求を符号化。種別とsourceの組合せが不正なら空。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<CanFrame> encodeRemoteRequest(RemoteRequest value) noexcept;
/** @brief Piからの操作要求を復号。操作実行・重複判定は行わない。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<RemoteRequest> decodeRemoteRequest(const CanFrame& frame) noexcept;
/** @brief 操作結果を符号化。未知の列挙値を拒否。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<CanFrame> encodeRemoteResult(RemoteResult value) noexcept;
/** @brief F446の応答を復号。列挙値・予約領域も検査。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<RemoteResult> decodeRemoteResult(const CanFrame& frame) noexcept;

} // namespace balancing::protocol
