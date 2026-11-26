#pragma once
#include "balancing/app/StatusIndicator.hpp"
#include "balancing/device/Led.hpp"

namespace balancing {
/** @brief RGB・ブザーの順で固定。極性・方式が確認できた場合のみverified。
 * @spec{BOARD-F446-001}
 */
struct IndicatorOutputConfig {
    std::array<IndicatorPin,4> pins{};
    bool verified{};
};
struct IndicatorOutputSummary {
    bool configured{}, failed{};
    std::uint32_t writeFailures{};
    std::optional<IndicatorState> applied{};
};
/** @brief 論理表示の出力境界。ブザー期限はStatusIndicatorだけが管理する。
 * @note GPIOのレベル出力のみ。パッシブブザーPWMは未決定。
 * @spec{BOARD-F446-001}
 */
class StatusIndicatorPort final {
public:
    explicit StatusIndicatorPort(const IndicatorOutputConfig& c={}) noexcept;
    /** @brief 4チャネル以内で出力。失敗時は全OFFを試み、失敗を保持。
     * @spec{BOARD-F446-001}
     */
    bool apply(const IndicatorState& state) noexcept;
    /** @brief 取得不能・再初期化時のOFF要求。未設定では書込まない。
     * @spec{BOARD-F446-001}
     */
    void disable() noexcept;
    IndicatorOutputSummary summary() const noexcept { return summary_; }
private:
    bool setAll(const std::array<bool,4>& levels) noexcept;
    std::array<Led,4> outputs_;
    IndicatorOutputSummary summary_{};
};
#if defined(__arm__) || defined(__thumb__)
/** @brief 既存main.hのRGB/BUZZERピンを参照して出力設定を作る。
 * @warning 極性・ブザー方式確認済みの明示設定専用。構築時はGPIOへ書き込まない。
 * @spec{BOARD-F446-001}
 */
IndicatorOutputConfig makeF446IndicatorOutput(const std::array<bool,4>& activeHigh,
                                               bool verified) noexcept;
#endif
}
