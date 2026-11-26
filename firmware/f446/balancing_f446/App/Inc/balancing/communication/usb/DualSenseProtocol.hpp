#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

namespace balancing {
/** @brief 1バイト軸の校正。実機の配置・方向・最大物理量は外部設定。
 * @spec{PROTO-PS5-001}
 */
struct HidAxis { std::uint8_t offset{}, min{}, center{}, max{}; bool inverted{}; float limit{}; };
struct HidButton { std::uint8_t offset{}, mask{}; };
struct DualSenseHidConfig {
    std::uint8_t reportId{}, length{};
    HidAxis forward{}, yaw{};
    HidButton options{}, circle{};
    bool verified{};
};
struct DualSenseInput { float speedMps{}, yawRadS{}; bool options{}, circle{}; };
/** @brief USBレポートの固定長・ID・軸範囲を検査する。配置は決め打ちしない。
 * @spec{PROTO-PS5-001}
 */
class DualSenseHidCodec final {
public:
    explicit DualSenseHidCodec(DualSenseHidConfig config={}) noexcept;
    bool enabled() const noexcept { return valid_; }
    /** @brief 全項目が正しい場合のみSI操作量とボタンを返す。
     * @param data 完了したレポート。呼出中だけ借用。
     * @param length 実際の転送完了長。
     * @spec{PROTO-PS5-001}
     */
    std::optional<DualSenseInput> decode(const std::uint8_t* data,std::size_t length) const noexcept;
private:
    DualSenseHidConfig config_;
    bool valid_{};
};

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
