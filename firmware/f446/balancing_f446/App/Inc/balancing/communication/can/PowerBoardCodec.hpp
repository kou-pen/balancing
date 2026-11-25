#pragma once

#include "balancing/communication/can/ProtocolId.hpp"
#include "balancing/communication/can/ProtocolPayload.hpp"

namespace balancing::protocol {

/** @brief ワイヤ上の電源状態。実際の電源供給・ARM許可とは独立。
 * @spec{PROTO-PWR-001}
 */
enum class WirePowerState : std::uint8_t { Off=0, Precharge=1, On=2, Fault=3 };

/** @brief 電源状態報告。flags/faultBitsは未確定の生ビット、sequenceは報告連番。
 * @spec{PROTO-PWR-001}
 */
struct PowerStatusPayload {
    WirePowerState state;
    std::uint8_t flags;
    std::uint16_t faultBits;
    std::uint16_t sequence;
};

/** @brief 電源操作のRegister ID。非常停止とは別の要求。
 * @spec{PROTO-PWR-001}
 */
enum class PowerRequestKind : std::uint16_t { On=1, Off=2 };

/** @brief F446/PiからF303への要求。requestIdは応答照合用。
 * @spec{PROTO-PWR-001}
 */
struct PowerRequest {
    NodeId source;
    PowerRequestKind kind;
    std::uint16_t requestId;
};

/** @brief F303から要求元への応答。Acceptedは実行完了を意味しない。
 * @spec{PROTO-PWR-001}
 */
struct PowerResult {
    NodeId destination;
    PowerRequestKind kind;
    std::uint16_t requestId;
    ResultCode result;
    WirePowerState state;
};

/** @brief F303の状態報告を符号化。未知の状態は空を返す。
 * @spec{PROTO-PWR-001}
 */
std::optional<CanFrame> encodePowerStatus(PowerStatusPayload value) noexcept;
/** @brief 状態報告のID・DLC・状態・予約領域を検査して復号。
 * @warning フラグの意味と鮮度、実際の電力供給は判断しない。
 * @spec{PROTO-PWR-001}
 */
std::optional<PowerStatusPayload> decodePowerStatus(const CanFrame& frame) noexcept;
/** @brief 許可された送信元と操作種別の要求を符号化。不正なら空。
 * @spec{PROTO-PWR-001}
 */
std::optional<CanFrame> encodePowerRequest(PowerRequest value) noexcept;
/** @brief 電源要求を復号。実行の許否は判断しない。
 * @spec{PROTO-PWR-001}
 */
std::optional<PowerRequest> decodePowerRequest(const CanFrame& frame) noexcept;
/** @brief 要求元・要求種別・結果・状態を検査して応答を符号化。
 * @spec{PROTO-PWR-001}
 */
std::optional<CanFrame> encodePowerResult(PowerResult value) noexcept;
/** @brief F303の応答を復号。不正な列挙値・予約領域は拒否。
 * @spec{PROTO-PWR-001}
 */
std::optional<PowerResult> decodePowerResult(const CanFrame& frame) noexcept;
/** @brief 共通の非常停止フレームを符号化。許可された機器のみ。
 * @spec{PROTO-PWR-001}
 */
std::optional<CanFrame> encodeEmergencyStop(NodeId source) noexcept;
/** @brief DLC0の非常停止フレームの送信元を復号。
 * @warning CAN IDの送信元は認証ではない。
 * @spec{PROTO-PWR-001}
 */
std::optional<NodeId> decodeEmergencyStop(const CanFrame& frame) noexcept;

} // namespace balancing::protocol
