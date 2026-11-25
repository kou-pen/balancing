#pragma once

#include "balancing/communication/can/CanTypes.hpp"
#include <optional>

namespace balancing::protocol {

/** @brief CAN2自作通信の機器番号。Broadcastは宛先専用。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
enum class NodeId : std::uint8_t { F446=1, F303=2, Pi=3, Broadcast=15 };

/** @brief Data typeの割当。メーカーCAN通信とは別体系。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
enum class DataType : std::uint8_t {
    Common=0x01, Power=0x10, Remote=0x20, Diagnostics=0x30, Parameters=0x40
};

/** @brief 29bit IDの論理フィールド。priorityは0〜3、regは10bit。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
struct ProtocolId {
    std::uint8_t priority;
    DataType type;
    NodeId source, destination;
    std::uint16_t reg;
};

/** @brief v1で割当済みのIDと正確なDLC。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
struct MessageSpec {
    ProtocolId fields;
    std::uint8_t dlc;
};

/** @brief DJI標準IDの予約領域。0x700も予防的に禁止する。
 * @spec{PROTO-PWR-001}
 */
constexpr bool isDjiReserved(std::uint32_t id) noexcept
{
    return (id >= 0x1ffu && id <= 0x208u) || id == 0x700u;
}

/** @brief 拡張IDそのものと上位11bitの両方を予約領域と照合する。
 * @spec{PROTO-PWR-001}
 */
bool isReservedExtendedId(std::uint32_t id) noexcept;

/** @brief 論理フィールドを検証して符号化。未割当IDの生成も可能。
 * @return 不正フィールドまたは予約衝突なら空。
 * @warning 生成成功は送受信許可ではない。findMessageで割当を確認する。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<std::uint32_t> encodeId(ProtocolId fields) noexcept;

/** @brief 29bit範囲、フィールド、予約領域を検査して分解。
 * @return 不正IDなら空。登録有無は別途findMessageで確認する。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<ProtocolId> decodeId(std::uint32_t id) noexcept;

/** @brief v1の固定35 IDから完全一致する定義を取得。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
std::optional<MessageSpec> findMessage(std::uint32_t id) noexcept;

/** @brief 拡張データフレームのIDとDLCの完全一致を確認。
 * @warning 内容の意味、操作元の実状態、実機送信許可は検査しない。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
bool matchesProtocolFrame(const CanFrame& frame) noexcept;

} // namespace balancing::protocol
