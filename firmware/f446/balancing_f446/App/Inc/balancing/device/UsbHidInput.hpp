#pragma once
#include "balancing/communication/usb/DualSenseHidCodec.hpp"
namespace balancing {
/** @brief 主ループ専用USB完了通知の新規性境界。HAL/IRQは所有しない。
 * @spec{PROTO-PS5-001}
 */
class UsbHidInput final {
public:
    explicit UsbHidInput(DualSenseHidConfig c={},std::uint64_t timeoutUs=0) noexcept
        : codec_{c},timeout_{timeoutUs} {}
    /** @brief 新接続の単調増加世代を登録する。旧接続の遅延通知は拒否。
     * @spec{PROTO-PS5-001}
     */
    bool connect(std::uint64_t generation,std::uint64_t atUs) noexcept;
    bool disconnect(std::uint64_t generation,std::uint64_t atUs) noexcept;
    /** @brief 完了済みの新規転送のみ。欠落・不正レポート時は連続性を取り消す。
     * @spec{PROTO-PS5-001}
     */
    std::optional<DualSenseInput> receive(std::uint64_t generation,std::uint64_t serial,
        const std::uint8_t* data,std::size_t length,std::uint64_t atUs,std::uint64_t nowUs) noexcept;
    bool fresh(std::uint64_t nowUs) const noexcept;
    bool enabled() const noexcept { return codec_.enabled() && timeout_; }
    std::uint64_t continuity() const noexcept { return continuity_; }
private:
    void invalidate() noexcept;
    DualSenseHidCodec codec_;
    std::uint64_t timeout_{},generation_{},serial_{},boundary_{},continuity_{};
    std::optional<std::uint64_t> last_{};
    bool connected_{};
};
}
