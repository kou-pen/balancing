#pragma once
#include "balancing/communication/usb/DualSenseHostClass.hpp"
#include <array>
#include <optional>

namespace balancing {
/** @brief IN転送の識別子。接続世代と転送番号を独立に照合。
 * @spec{PROTO-PS5-001}
 */
struct UsbTransferToken { std::uint64_t generation{},serial{}; };
/** @brief 固定長のUSB完了通知。採取時刻は転送完了時刻でありpoll時刻ではない。
 * @spec{PROTO-PS5-001}
 */
struct UsbHostReport {
    UsbTransferToken token{};
    std::uint64_t receivedAtUs{};
    std::size_t length{};
    std::array<std::uint8_t,64> data{};
};
/** @brief 接続/切断の一回限りの通知。
 * @spec{PROTO-PS5-001}
 */
struct UsbHostConnection { std::uint64_t generation{},atUs{}; bool connected{}; };
enum class UsbHostEventKind : std::uint8_t { None,Ready,Detached,Done,Nak,Stall,Error };
/** @brief 下位ドライバの主ループ回収イベント。attachmentは物理接続ごとに増加。
 * @spec{PROTO-PS5-001}
 */
struct UsbHostEvent {
    UsbHostEventKind kind{};
    std::uint64_t attachment{},atUs{};
    UsbHostInterface interface{};
    UsbTransferToken token{};
    std::size_t length{};
    std::array<std::uint8_t,64> data{};
};
/** @brief 下位USB Hostとの境界。全関数は有限・非ブロッキング、主ループ専用。
 * @note submitのバッファ所有と旧URBの隔離は下位側の責務。pollは一件だけ返す。
 * @spec{PROTO-PS5-001}
 */
struct UsbHostBackend {
    void* context{};
    bool (*start)(void*,const UsbHostConfig&) noexcept{};
    UsbHostEvent (*poll)(void*,std::uint64_t) noexcept{};
    bool (*submit)(void*,UsbTransferToken,std::uint8_t) noexcept{};
};
struct UsbHostSummary {
    bool started{},connected{},inFlight{};
    std::uint32_t reports{},invalidReports{},naks{},faults{},rejectedEvents{};
};
/** @brief Hostの接続世代・一件のInterrupt IN・期限・完了配送を管理。
 * @spec{PROTO-PS5-001}
 */
class UsbHostPort final {
public:
    /** @brief 明示設定で一回だけ下位を起動。未検証なら副作用なし。
     * @spec{PROTO-PS5-001}
     */
    bool start(UsbHostConfig,UsbHostBackend) noexcept;
    /** @brief 一回の呼出しで下位poll一回と高々一件の転送受付。
     * @spec{PROTO-PS5-001}
     */
    void poll(std::uint64_t nowUs) noexcept;
    /** @brief 通知を一度だけ消費する。
     * @spec{PROTO-PS5-001}
     */
    std::optional<UsbHostConnection> takeConnectionEvent() noexcept;
    std::optional<UsbHostReport> takeReport() noexcept;
    UsbHostSummary summary() const noexcept { return summary_; }
private:
    void loseConnection(std::uint64_t atUs) noexcept;
    void handle(const UsbHostEvent&,std::uint64_t nowUs) noexcept;
    UsbHostConfig config_{};
    UsbHostBackend backend_{};
    UsbHostSummary summary_{};
    std::uint64_t attachment_{},generation_{},serial_{},submittedAt_{},lastBoundary_{};
    std::optional<std::uint64_t> lastPoll_{},lastSubmit_{};
    std::optional<UsbHostConnection> connection_{};
    std::optional<UsbHostReport> report_{};
};
}
