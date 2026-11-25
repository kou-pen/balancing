#pragma once

#include "balancing/communication/can/CanTypes.hpp"
#include <cstddef>
#include <optional>

namespace balancing::protocol {

/** @brief 共通の要求処理結果。Acceptedは受理のみで実行完了ではない。
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
enum class ResultCode : std::uint8_t {
    Accepted=0, AlreadyInState=1, Busy=2, SafetyRejected=3,
    InvalidRequest=4, Unauthorized=5
};

/** @brief DLC内の指定位置にuint16をBig endianで格納。
 * @return 領域不足・不正DLCならfalse。失敗時はフレーム不変。
 * @spec{PROTO-REMOTE-001}
 */
bool writeU16BE(CanFrame& frame, std::size_t offset, std::uint16_t value) noexcept;
/** @brief DLC内の指定位置にuint32をBig endianで格納。
 * @return 失敗時false、フレーム不変。
 * @spec{PROTO-REMOTE-001}
 */
bool writeU32BE(CanFrame& frame, std::size_t offset, std::uint32_t value) noexcept;
/** @brief 有限のIEEE754 binary32をBig endianで格納。
 * @return 不正領域・NaN・Infならfalse、フレーム不変。
 * @spec{PROTO-REMOTE-001}
 */
bool writeFloat32BE(CanFrame& frame, std::size_t offset, float value) noexcept;
/** @brief DLC内のuint16を復号。範囲外なら空。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<std::uint16_t> readU16BE(const CanFrame& frame, std::size_t offset) noexcept;
/** @brief DLC内のuint32を復号。範囲外なら空。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<std::uint32_t> readU32BE(const CanFrame& frame, std::size_t offset) noexcept;
/** @brief 有限のbinary32を復号。不正領域・NaN・Infなら空。
 * @spec{PROTO-REMOTE-001}
 */
std::optional<float> readFloat32BE(const CanFrame& frame, std::size_t offset) noexcept;

} // namespace balancing::protocol
