#pragma once
#include "balancing/app/RobotSequence.hpp"
#include "balancing/communication/can/CanTypes.hpp"
#include <optional>
#include <cstdint>

namespace balancing {
/** @brief 2つの操作経路。CANの数値IDやUSB HIDボタン番号は含めない。 */
enum class OperatorSource : std::uint8_t {
    UsbDualSense = 0,
    CanRemote = 1
};

/**
 * @brief 操作入力処理の外部検証済み設定。
 * @spec{BEHAV-001}
 * @spec{PROTO-REMOTE-001}
 * @warning デッドゾーン・通信周期・速度上限は実機未確定である。
 *          ホスト試験では人工的な数値を明示的に注入する。
 */
struct OperatorInputConfig {
    std::uint64_t input_timeout_us{}; /**< 最後の新規入力から有効とする期限 [us]。 */
    std::uint64_t neutral_hold_us{}; /**< 復旧時ニュートラル継続確認に必要な時間 [us]。 */
    float max_speed_m_s{}; /**< デコード済み前後速度要求の絶対値上限 [m/s]。 */
    float max_yaw_rate_rad_s{}; /**< デコード済みYaw角速度要求上限 [rad/s]。 */
    float neutral_speed_m_s{}; /**< 速度をニュートラルと扱う幅 [m/s]。 */
    float neutral_yaw_rad_s{}; /**< Yawをニュートラルと扱う幅 [rad/s]。 */
    float speed_slew_m_s2{}; /**< 速度目標の変化率上限 [m/s^2]。 */
    float yaw_slew_rad_s2{}; /**< Yaw目標の変化率上限 [rad/s^2]。 */
    bool verified{}; /**< 実機設定の検証、または明示した人工fixtureの確認。 */
};

/** @brief 最後に新規と確認されたデコード済みの入力。 */
struct OperatorSample {
    bool valid{};
    std::uint64_t stamp_us{}; /**< 上位入力の実測定・受信時刻 [us]。再読出し時刻ではない。 */
    float speed_m_s{};
    float yaw_rate_rad_s{};
};

/** @brief 500 Hzティックが返す操作目標。ARMを要求するフィールドはない。 */
struct OperatorTarget {
    OperatorSource source{};
    bool source_fresh{};
    bool operator_ready{};
    bool waiting_neutral{};
    float speed_m_s{};
    float yaw_rate_rad_s{};
};


/** @brief 復号済み操作入力を単一所有し、既存の変化率・復帰処理を利用する。
 * @spec{BEHAV-001}
 * @spec{PROTO-REMOTE-001}
 * @spec{PROTO-PS5-001}
 */
class RemoteControlService final {
public:
    /** @brief 検証済みの物理量設定で初期化する。既定は無効。
     * @spec{BEHAV-001}
     */
    explicit RemoteControlService(const OperatorInputConfig& config = {}) noexcept;
    /** @brief DISARMED中の明示切替のみを受理する。
     * @spec{PROTO-REMOTE-001}
     */
    bool selectSource(OperatorSource source, RobotState state) noexcept;
    /** @brief 通信層で新規性・形式・送信元を確認済みの入力のみ受理する。
     * @spec{BEHAV-001}
     */
    bool acceptDecoded(OperatorSource source, const OperatorSample& sample,
                       bool newSampleProven, std::uint64_t nowUs) noexcept;
    /** @brief CAN運動指令を復号して選択元・受信期限を検査する。
     * @spec{PROTO-REMOTE-001}
     */
    bool acceptCanMotion(const CanFrame& frame,std::uint64_t receivedAtUs,std::uint64_t nowUs) noexcept;
    /** @brief 初期化済みの操作元。未設定なら空。
     * @spec{PROTO-REMOTE-001}
     */
    std::optional<OperatorSource> selectedSource() const noexcept;
    /** @brief CAN欠落時に旧入力と中立確認を破棄する。選択元と減速状態は維持。
     * @spec{BEHAV-001}
     */
    void invalidateCanInput() noexcept;
    /** @brief USB切断・転送欠落時の旧入力無効化。減速は既存周期処理に委ねる。
     * @spec{PROTO-PS5-001}
     */
    void invalidateUsbInput() noexcept;
    /** @brief 操作量の周期コピー。ARM要求は生成しない。
     * @spec{BEHAV-001}
     */
    std::optional<OperatorTarget> update(RobotState state, std::uint64_t nowUs) noexcept;
private:
    struct InputState {
        OperatorInputConfig config;
        OperatorSource selected;
        OperatorSample last_sample;
        bool waiting_neutral;
        bool neutral_start_valid;
        std::uint64_t neutral_start_us;
        unsigned neutral_sample_count;
        bool tick_valid;
        std::uint64_t last_tick_us;
        float target_speed_m_s;
        float target_yaw_rate_rad_s;
        bool initialized;
    };
    InputState input_{};
    void requireNeutral() noexcept;
    bool sampleRecent(std::uint64_t nowUs) const noexcept;
    void invalidate(OperatorSource source) noexcept;
};
}
