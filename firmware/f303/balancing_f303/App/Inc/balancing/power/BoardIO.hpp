#pragma once

#include "balancing/power/PowerController.hpp"

namespace balancing::power {

/** @brief GPIO適用直前の安全条件。実機のピン読出し・遮断ラッチは別途接続する。
 * @spec{BOARD-F303-001}
 */
struct BoardInterlock {
    bool powerPinsVerified{false}; ///< ピン・極性・重複導通を確認済みの場合だけtrue
    bool estopOk{false}; ///< 物理停止が解除されている
    bool hardwareFault{true}; ///< ハード保護異常。未確認時も遮断
    bool shutdownPending{true}; ///< IRQ遮断イベントの未処理状態
};

/** @brief GPIOへ適用する直前の論理要求。物理出力の実測値ではない。
 * @spec{BOARD-F303-001}
 */
struct BoardOutput {
    bool prechargeOn{false};
    bool mainOn{false};
    bool ledOn{false}; ///< LEDの論理点灯。GPIO極性への変換は外部
    bool accepted{false}; ///< Controller要求が論理ゲートを通過した
};

/** @brief 基板出力の最終論理判定とLED表示。電源状態は所有しない。
 * @spec{BOARD-F303-001}
 */
class BoardIO {
public:
    /** @brief 制御要求と適用直前の安全条件を評価する。
     * @param request Controllerの当該周期出力。
     * @param interlock 出力直前の安全条件。
     * @param nowUs 単調時計 [us]。
     * @return GPIOへ渡す論理要求。拒否時は両電源OFF。
     * @warning 本関数はHAL/GPIO未接続。実機ではラッチの確認とGPIO書込みを不可分にし、
     * 拒否時のControllerへの通知・CAN候補破棄をapplication側で実装する。
     * @spec{BOARD-F303-001}
     */
    BoardOutput evaluate(const PowerOutput& request, const BoardInterlock& interlock,
                         std::uint64_t nowUs) noexcept;
private:
    bool ledStarted_{false};
    PowerState ledState_{PowerState::Off}; ///< 表示位相用。電源状態そのものではない。
    std::uint64_t ledSinceUs_{0};
    std::uint64_t lastUs_{0};
};

} // namespace balancing::power
