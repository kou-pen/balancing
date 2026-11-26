#pragma once

#include "balancing/control/controller/BalanceControl.hpp"

#include <array>
#include <cstdint>

namespace balancing {

/** @brief RS06の機構軸からモータ軸へのトルク換算・上限設定。
 *  @note motorRevsPerJointRev は「モータ回転数 / 機構入力軸回転数」。
 *        実機の外部伝達と原点符号を別途照合し、起動時に固定する。
 *  @spec{CTRL-LQR-001}
 *  @spec{PROTO-RS06-001}
 */
struct Rs06TorqueConversion {
    float motorRevsPerJointRev{0.0f};
    float motorDirection{0.0f};      // +1/-1、モータ軸と機構軸の符号
    float maxAbsMotorTorqueNm{0.0f}; // 物理安全上限（CAN表現幅とは独立）
    bool verified{false};
};

/** @brief C620の車輪機構軸トルク→電流換算・上限設定。
 *  @note wheelTorqueNmPerAmp はモータ定数・全減速比・効率を含む実測係数。
 *        複数の減速比を重複して掛けない。
 *  @spec{CTRL-LQR-001}
 *  @spec{PROTO-C620-001}
 */
struct C620CurrentConversion {
    float wheelTorqueNmPerAmp{0.0f}; // 車輪側で得られるトルク [N·m/A]
    float currentDirection{0.0f};    // 正の車輪トルクに対応する電流符号±1
    float maxAbsCurrentA{0.0f};      // 車輪ごとの実機検証済み電流制限[A]
    bool verified{false};
};

struct MotorCommandConfig {
    std::array<Rs06TorqueConversion, 4> rs06{}; // 左脚A/B、右脚A/B
    std::array<C620CurrentConversion, 2> c620{}; // 左車輪・右車輪
};

enum class CommandStatus : std::uint8_t {
    Ready,
    Saturated,
    UnverifiedConfiguration,
    InvalidInput,
    ConversionFailed
};

/** @brief 片軸の機構側トルクからRS06モータ側トルクへの数学換算結果。 */
struct Rs06TorquePreview {
    float requestedMotorTorqueNm{0.0f};
    float limitedMotorTorqueNm{0.0f};
    bool saturated{false};
};

/** @brief 片車輪の機構側トルクからC620電流への数学換算結果。 */
struct C620CurrentPreview {
    float requestedCurrentA{0.0f};
    float limitedCurrentA{0.0f};
    bool saturated{false};
};

/** @brief 6軸の数学変換と軸別飽和診断。CAN送信可能なMotorTargetsではない。
 *  @warning fixtureOnly=trueを固定し、直接送信するAPIを提供しない。
 *  @spec{CTRL-LQR-001}
 */
struct MotorCommandPreview {
    CommandStatus status{CommandStatus::UnverifiedConfiguration};
    std::array<Rs06TorquePreview, 4> rs06{};
    std::array<C620CurrentPreview, 2> c620{};
    bool upstreamWheelSaturated{false};
    bool fixtureOnly{true};
};

/** @brief MotorCommandの最終換算数学を1箇所へ集約するSIL専用段階。
 *  @note 実機モデルの検証状況を勝手に昇格しない。CAN/HAL/MotorServiceに依存しない。
 *  @spec{CTRL-LQR-001}
 *  @spec{PROTO-RS06-001}
 *  @spec{PROTO-C620-001}
 */
class MotorCommand final {
public:
    /** @brief 起動時に設定をコピーし、有効性を一度だけ検査する。
     *  @param[in] config 4関節＋2車輪の独立した検証済み設定。
     *  @note 未確認の実機数値をデフォルトで採用しない。
     *  @spec{CTRL-LQR-001}
     */
    explicit MotorCommand(const MotorCommandConfig& config) noexcept;

    /** @brief 4関節のNmと2車輪のNmから、モータNm・C620電流Aへ換算する。
     *  @param[in] input 数学fixtureのBalanceControl出力。
     *  @return 飽和なら制限後の値と元要求値を両方返す。
     *          無効時は全軸の出力をゼロとし、理由だけ返す。
     *  @warning 常にfixtureOnly=true。実機へ渡す前段にしてはならない。
     *  @spec{CTRL-LQR-001}
     */
    MotorCommandPreview evaluate(const BalanceControlOutput& input) const noexcept;

    /** @brief 検証済みと明示された換算値が6軸とも構築時検査に通ったか。
     *  @warning 人工fixtureのverifiedは実機の安全証明ではない。
     *  @spec{CTRL-LQR-001}
     */
    bool hasVerifiedConfiguration() const noexcept { return configValid_; }

    /** @brief 軸別の変換設定を既存の数値・通信範囲条件で検査。
     * @spec{CTRL-LQR-001}
     * @spec{PROTO-RS06-001}
     */
    static bool valid(const Rs06TorqueConversion& axis) noexcept;
    /** @brief 車輪別の換算設定を検査。実機の許容電流は認定しない。
     * @spec{PROTO-C620-001}
     */
    static bool valid(const C620CurrentConversion& axis) noexcept;

private:

    MotorCommandConfig config_{};
    bool configValid_{false};
};

} // namespace balancing
