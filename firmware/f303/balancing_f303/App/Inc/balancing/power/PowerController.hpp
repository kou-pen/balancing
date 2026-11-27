#pragma once

#include <cstdint>

namespace balancing::power {

/** @brief CAN報告と対応する電源状態。重複導通中もPRECHARGEとする。
 * @spec{BOARD-F303-001}
 */
enum class PowerState : std::uint8_t { Off, Precharge, On, Fault };

/** @brief 当該周期の操作イベント。複数受信時の優先順はESTOP、OFF、ON。
 * @spec{PROTO-PWR-001}
 */
enum class PowerCommand { None, On, Off, EmergencyStop };

/** @brief 操作の論理的な受理結果。受理は物理的な通電を保証しない。
 * @spec{PROTO-PWR-001}
 */
enum class PowerCommandResult { None, Accepted, AlreadyInState, Busy, SafetyRejected };

/** @brief 起動原因。未判別のリセットは自動起動させない。
 * @spec{BOARD-F303-001}
 */
enum class ResetCause { AbnormalOrUnknown, BatteryConnection };

/** @brief 起動時固定の判定設定。既定値では投入を許さない。
 * @warning 実機値・検証済み設定は回路/負荷/実測確認後のみ設定する。
 * @spec{BOARD-F303-001}
 */
struct PowerConfig {
    bool verified{false};
    float maxVoltageDifferenceV{0.0F}; ///< 許容絶対電圧差 [V]
    std::uint64_t stableUs{0}; ///< 電圧差の連続成立時間 [us]
    std::uint64_t prechargeTimeoutUs{0}; ///< プリチャージ全体の期限 [us]
    std::uint64_t overlapTimeoutUs{0}; ///< 主電源成立確認の期限 [us]
    std::uint64_t observationMaxAgeUs{0}; ///< 観測有効期限 [us]、等号は失効
};

/** @brief 一周期の論理観測。ADC換算・状態別保護判定はPowerMonitorの責務。
 * @warning hardwareFault/mainConfirmedは生GPIOではなく、回路に照合した論理値。
 * @spec{BOARD-F303-001}
 */
struct PowerInputs {
    PowerCommand command{PowerCommand::None}; ///< 一度だけ消費するイベント
    bool estopOk{false}; ///< 物理非常停止が解除されている
    bool hardwareFault{false}; ///< 確認済みのハードウェア異常
    bool observationValid{false}; ///< ADC観測と時刻が有効
    bool protectionOk{false}; ///< 現在状態でのソフトウェア保護条件成立
    bool mainConfirmed{false}; ///< 主電源投入成立の独立した確認（手段未確定）
    float batteryVoltageV{0.0F};
    float driveBusVoltageV{0.0F};
    std::uint64_t observedAtUs{0}; ///< 単調時計上の観測時刻 [us]
};

/** @brief 論理出力要求。GPIO極性・実回路操作には直接対応させない。
 * @spec{BOARD-F303-001}
 */
struct PowerOutput {
    PowerState state{PowerState::Off};
    bool prechargeRequested{false};
    bool mainRequested{false};
    PowerCommandResult commandResult{PowerCommandResult::None};
};

/** @brief 電源状態を唯一所有するHAL非依存の周期処理。
 * @spec{BOARD-F303-001}
 */
class PowerController {
public:
    /** @brief 出力OFFで構築する。異常/未判別リセットではFAULTから開始する。
     * @param config 起動固定設定。未検証の設定での投入は禁止。
     * @param resetCause 確認済み起動原因。
     * @spec{BOARD-F303-001}
     */
    explicit PowerController(PowerConfig config = {},
                             ResetCause resetCause = ResetCause::AbnormalOrUnknown) noexcept;

    /** @brief 一周期の観測・操作を評価し、次の論理出力を返す。
     * @param inputs 最新の論理観測と一回限りの操作イベント。
     * @param nowUs 観測と同一の単調時計 [us]。逆行はFAULT。
     * @return 現在状態・出力要求・当該操作の結果。
     * @warning 出力は実回路への接続前に別途検証する。緊急遮断ISRは外部。
     * @spec{BOARD-F303-001}
     */
    PowerOutput step(const PowerInputs& inputs, std::uint64_t nowUs) noexcept;

    /** @brief 操作結果を含まない現在の論理出力を取得する。
     * @spec{BOARD-F303-001}
     */
    PowerOutput output() const noexcept;

private:
    enum class Stage { Off, Charging, Overlap, On, Fault };
    bool validConfig() const noexcept;
    bool validObservation(const PowerInputs& input, std::uint64_t nowUs) const noexcept;
    void enterFault() noexcept;
    void enterOff() noexcept;
    void startPrecharge(std::uint64_t nowUs) noexcept;
    PowerOutput result(PowerCommandResult commandResult) const noexcept;

    const PowerConfig config_;
    Stage stage_;
    bool runRequested_; ///< 通常起動/物理停止解除の復帰意図。CAN OFF/FAULTで消去。
    bool stepped_{false};
    std::uint64_t lastStepUs_{0};
    bool seenObservation_{false};
    std::uint64_t lastObservationUs_{0};
    std::uint64_t stageStartedUs_{0};
    bool voltageHolding_{false};
    std::uint64_t voltageHoldStartedUs_{0};
};

} // namespace balancing::power
