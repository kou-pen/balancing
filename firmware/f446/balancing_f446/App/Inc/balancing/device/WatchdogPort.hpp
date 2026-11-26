#pragma once
#include "balancing/core/CycleTicket.hpp"
#include <optional>

namespace balancing {
/** @brief 明示的に構成済みの更新専用ポート。IWDGの開始は行わない。
 * @spec{BOARD-F446-001}
 */
struct WatchdogConfig {
    using Refresh = bool(*)(void*) noexcept;
    void* context{};
    Refresh refresh{};
    bool verified{};
};
/** @brief 更新要求とコールバック結果の診断。実機リセット期限の保証ではない。
 * @spec{BOARD-F446-001}
 */
struct WatchdogSummary {
    bool configured{};
    std::uint32_t attempts{},successes{},failures{},rejected{};
};
/** @brief 健全な新規完了証跡を一回だけ更新要求へ変換する。
 * @spec{BOARD-F446-001}
 */
class WatchdogPort final {
public:
    explicit WatchdogPort(WatchdogConfig config={}) noexcept : config_{config} {}
    /** @brief 旧/重複/逆行証跡を拒否。未設定なら無出力。
     * @spec{BOARD-F446-001}
     */
    bool refresh(const CycleCompletion& completion) noexcept;
    /** @brief 最後の更新結果を取得。
     * @spec{BOARD-F446-001}
     */
    WatchdogSummary summary() const noexcept;
private:
    WatchdogConfig config_{};
    WatchdogSummary status_{};
    std::optional<CycleCompletion> last_{};
};
}
