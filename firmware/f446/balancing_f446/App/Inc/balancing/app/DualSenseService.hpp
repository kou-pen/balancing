#pragma once
#include "balancing/device/UsbHidInput.hpp"
#include "balancing/app/RemoteControlService.hpp"
namespace balancing {
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
    bool selectSource(BOperatorSource target,RobotState state,std::uint64_t nowUs) noexcept;
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
    std::optional<BOperatorSource> source_{};
    bool pendingArm_{},pendingDisarm_{};
};
}
