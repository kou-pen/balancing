#pragma once
#include "balancing/communication/usb/DualSenseProtocol.hpp"
#include "balancing/app/RemoteControlService.hpp"
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

struct DualSenseConfig {
    DualSenseHidConfig hid{};
    std::uint64_t reportTimeoutUs{}, armHoldUs{}, disarmHoldUs{};
};
/** @brief USB報告から操作量と一回限りのボタンイベントを生成する。
 * @spec{PROTO-PS5-001}
 * @spec{BEHAV-001}
 */
class DualSenseService final {
public:
    DualSenseService(RemoteControlService& remote,DualSenseConfig c={}) noexcept;
    /** @brief USB接続/切断は主ループで配送。世代はUSB層が所有。
     * @spec{PROTO-PS5-001}
     */
    bool connect(std::uint64_t generation,std::uint64_t atUs) noexcept;
    bool disconnect(std::uint64_t generation,std::uint64_t atUs) noexcept;
    /** @brief 完了通知を直接消費。コピーの再読出しやIRQからの呼出は禁止。
     * @spec{PROTO-PS5-001}
     */
    bool receive(std::uint64_t generation,std::uint64_t serial,const std::uint8_t* data,
                 std::size_t length,std::uint64_t atUs,std::uint64_t nowUs) noexcept;
    /** @brief 操作元選択後に周期イベントを回収。DISARMは操作元非依存。
     * @spec{BEHAV-001}
     */
    OperatorEvents takeEvents(std::uint64_t nowUs) noexcept;
    /** @brief USB側の明示選択要求。ボタン割当は未確定。
     * @spec{PROTO-PS5-001}
     */
    bool selectSource(OperatorSource target,RobotState state,std::uint64_t nowUs) noexcept;
    bool enabled() const noexcept { return input_.enabled() && config_.armHoldUs && config_.disarmHoldUs; }
private:
    struct Hold {
        bool released{}, fired{};
        std::optional<std::uint64_t> since{};
        bool update(bool pressed,std::uint64_t at,std::uint64_t duration) noexcept;
    };
    void resetInput() noexcept;
    void syncSource() noexcept;
    RemoteControlService& remote_;
    DualSenseConfig config_;
    UsbHidInput input_;
    Hold arm_{},disarm_{};
    std::optional<OperatorSource> source_{};
    bool pendingArm_{},pendingDisarm_{};
};
}
