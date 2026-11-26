#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

namespace balancing {
/** @brief 照合済みのUSBデバイス/非Boot HID設定。未検証の既定値では接続しない。
 * @spec{PROTO-PS5-001}
 */
struct UsbHostConfig {
    std::uint16_t vendorId{},productId{};
    std::uint8_t interfaceNumber{},endpointAddress{},packetBytes{},intervalMs{},reportLength{};
    std::uint64_t transferTimeoutUs{};
    bool verified{};
};
/** @brief フルスピード非Boot HIDのInterrupt IN descriptor。
 * @spec{PROTO-PS5-001}
 */
struct UsbHostInterface {
    std::uint16_t vendorId{},productId{};
    std::uint8_t number{},alternate{},interfaceClass{},subclass{},protocol{};
    std::uint8_t endpointAddress{},attributes{},intervalMs{};
    std::uint16_t packetBytes{};
    bool fullSpeed{};
};
/** @brief 設定自体の検査。物理VBUS・配線の検証とは別。
 * @spec{PROTO-PS5-001}
 */
bool validUsbHostConfig(const UsbHostConfig&) noexcept;
/** @brief 列挙済みデバイスと設定を照合する。
 * @spec{PROTO-PS5-001}
 */
bool matchesDualSenseInterface(const UsbHostConfig&,const UsbHostInterface&) noexcept;
/** @brief 生の構成descriptorを境界検査し、明示指定の非Boot HID INだけを抽出する。
 * @spec{PROTO-PS5-001}
 */
std::optional<UsbHostInterface> findDualSenseInterface(const UsbHostConfig&,
    std::uint16_t vendorId,std::uint16_t productId,bool fullSpeed,
    const std::uint8_t* data,std::size_t length) noexcept;
}
