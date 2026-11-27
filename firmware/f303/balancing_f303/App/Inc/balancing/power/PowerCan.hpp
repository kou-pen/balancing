#pragma once
#include "balancing/power/PowerController.hpp"
#include "balancing/communication/can/PowerBoardCodec.hpp"
#include <array>
#include <cstddef>

namespace balancing::power {

/** @brief 一周期のCAN受信スナップショット。IRQ側でデータフレームのみ採取する。
 * @warning 古いフレームを次周期に持ち越さない。容量超過/受信欠落はlostで通知。
 * @spec{PROTO-PWR-001}
 */
struct PowerCanRx {
    static constexpr std::size_t capacity = 8;
    std::array<CanFrame, capacity> frames{};
    std::size_t count{0};
    bool lost{false};
};

/** @brief 当該周期の論理出力と送信候補。送信完了を意味しない。
 * @warning GPIO適用と緊急遮断ラッチ照合後に送信する。古い候補を蓄積しない。
 * @spec{PROTO-PWR-001}
 */
struct PowerCanOutput {
    PowerOutput power{};
    std::optional<CanFrame> emergency{};
    std::optional<CanFrame> status{};
    std::array<CanFrame, PowerCanRx::capacity> replies{};
    std::size_t replyCount{0};
};

/** @brief 電源CAN v1の受信集約、操作応答、周期報告。HAL非依存。
 * @spec{PROTO-PWR-001}
 */
class PowerCan {
public:
    /** @brief 受信を集約しControllerを一度だけ実行する。
     * @param controller 唯一の電源状態所有者。
     * @param inputs 同周期の観測。ローカル遮断イベントも優先する。
     * @param rx 当該周期の受信バッチ。
     * @param nowUs 単調時計 [us]。
     * @return 論理出力と送信候補。実送信は外部の責務。
     * @spec{BOARD-F303-001}
     * @spec{PROTO-PWR-001}
     */
    PowerCanOutput step(PowerController& controller, PowerInputs inputs,
                        const PowerCanRx& rx, std::uint64_t nowUs) noexcept;
private:
    bool statusIssued_{false};
    std::uint64_t lastStatusUs_{0};
    PowerState lastStatusState_{PowerState::Off};
    std::uint16_t sequence_{0};
    bool estopWasOk_{true};
};

} // namespace balancing::power
