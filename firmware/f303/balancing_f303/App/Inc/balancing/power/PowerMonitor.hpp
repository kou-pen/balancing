#pragma once

#include "balancing/power/PowerController.hpp"

namespace balancing::power {

/** @brief ADC 1チャネルの起動固定校正。物理値 = count * scale + offset。
 * @spec{BOARD-F303-001}
 */
struct AdcCalibration {
    std::uint16_t minCount{0}; ///< 有効な生値の下限（包含）
    std::uint16_t maxCount{0}; ///< 有効な生値の上限（包含）
    float scale{0.0F}; ///< V/count または A/count。極性を含む。
    float offset{0.0F}; ///< V または A
};

/** @brief 電源監視設定。既定値は未検証であり、観測を許可しない。
 * @warning 数値は回路・負荷・実測から決める。テスト値を流用しない。
 * @spec{BOARD-F303-001}
 */
struct PowerMonitorConfig {
    bool verified{false};
    AdcCalibration battery{};
    AdcCalibration driveBus{};
    AdcCalibration current{};
    float minBatteryV{0.0F};
    float maxBatteryV{0.0F};
    float minOnBusV{0.0F}; ///< ON時のみ適用
    float maxBusV{0.0F}; ///< 全状態で適用
    float maxAbsCurrentA{0.0F}; ///< 電流絶対値の許容上限
    std::uint64_t overcurrentHoldUs{0}; ///< 継続過電流の判定時間
    std::uint64_t observationMaxAgeUs{0}; ///< 観測期限、等号で失効
};

/** @brief 同じDMA取得単位の3チャネル生値。時刻は取得時に固定。
 * @spec{BOARD-F303-001}
 */
struct PowerAdcSample {
    bool valid{false};
    std::uint16_t batteryCount{0};
    std::uint16_t driveBusCount{0};
    std::uint16_t currentCount{0};
    std::uint64_t observedAtUs{0};
};

/** @brief 変換と保護判定の結果。無効観測は電源制御に採用しない。
 * @spec{BOARD-F303-001}
 */
struct PowerObservation {
    bool valid{false};
    bool protectionOk{false};
    float batteryVoltageV{0.0F};
    float driveBusVoltageV{0.0F};
    float currentA{0.0F};
    std::uint64_t observedAtUs{0};
};

/** @brief HAL非依存のADC換算・ソフトウェア保護判定。
 * @spec{BOARD-F303-001}
 */
class PowerMonitor {
public:
    /** @brief 起動固定設定をコピーする。未検証設定での正常判定は禁止。
     * @spec{BOARD-F303-001}
     */
    explicit PowerMonitor(PowerMonitorConfig config = {}) noexcept : config_(config) {}

    /** @brief 現在のController状態と同じ時計のADC観測を評価する。
     * @param sample DMA完了時に固定した生値。重複は同一内容のみ許可。
     * @param state Controllerが所有する現在状態。Monitorは状態遷移しない。
     * @param nowUs 単調時計 [us]。未来、逆行、期限切れは無効。
     * @return 物理量と有効性、保護判定。ハード異常と主電源成立は別入力。
     * @spec{BOARD-F303-001}
     */
    PowerObservation evaluate(const PowerAdcSample& sample, PowerState state,
                              std::uint64_t nowUs) noexcept;
private:
    bool validConfig() const noexcept;
    const PowerMonitorConfig config_;
    bool seenClock_{false};
    std::uint64_t lastNowUs_{0};
    bool seenSample_{false};
    PowerAdcSample lastSample_{};
    bool currentHolding_{false};
    std::uint64_t currentSinceUs_{0};
};

} // namespace balancing::power
