#pragma once
#include "balancing/app/RobotSequence.hpp"
#include <array>

namespace balancing {
struct IndicatorPattern {
    bool red{false},green{false},blue{false};
    std::uint32_t beepMs{0};
    std::uint64_t halfPeriodUs{0}; // 0:点灯固定
};
struct IndicatorConfig {
    std::array<IndicatorPattern,4> patterns{};
    bool verified{false};
};
struct IndicatorState {
    RobotState state{RobotState::Disarmed};
    bool configured{false};
    bool red{false},green{false},blue{false},buzzer{false};
};
/** @brief 状態表示の論理パターン。物理出力や非常停止に依存しない。
 * @spec{BOARD-F446-001}
 * @spec{BEHAV-001}
 */
class StatusIndicator final {
public:
    explicit StatusIndicator(IndicatorConfig config={}) noexcept : config_{config} {}
    /** @brief FAULT記録を通常表示より優先。状態変化時だけ単発鳴動を開始する。
     * @spec{BOARD-F446-001}
     */
    IndicatorState update(RobotState state,const std::optional<FaultRecord>& fault,
                          std::uint64_t nowUs) noexcept;
private:
    IndicatorConfig config_;
    std::optional<RobotState> previous_{};
    std::optional<std::uint64_t> lastUs_{};
    std::uint64_t enteredAtUs_{0};
    bool clockInvalid_{false};
};
}
