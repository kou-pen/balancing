#pragma once

#include "balancing/control/estimator/StateEstimator.hpp"
#include "balancing/control/controller/balancing_core.h"
#include "balancing/control/controller/balancing_leg_pd.h"
#include "balancing/control/controller/balancing_yaw_pi.h"

#include <array>
#include <optional>

namespace balancing {

/** @brief 今周期の6状態LQR・Yaw PIが追従する数学目標。
 *  @note 位置基準の生成・加減速制限は未実装の操作側の責務。
 *        ここに未合意の位置追従規則を暗黙に追加しない。
 *  @spec{CTRL-LQR-001}
 */
struct BalanceReference {
    float positionM{0.0f};
    float forwardVelocityMps{0.0f};
    float yawRateRadS{0.0f};
};

struct BalanceControlConfig {
    BLqrConfig lqr{};
    BLegPdGains legPd{};
    BYawPiGains yawPi{};
    std::array<float, 2> wheelTorqueLimitNm{};
};

/** @brief 機構側トルクの純粋な数学計算結果でありCAN指令ではない。
 *  @warning 脚のCAD、摩擦、ガスシリンダ、変速比、電流換算、実機限界は未結合。
 *           fixtureOnlyの結果をモータへ配送するAPIは存在しない。
 *  @spec{CTRL-LQR-001}
 */
struct BalanceControlOutput {
    std::array<float, 4> jointTorqueNm{}; // 左A/B、右A/B（機構軸）
    std::array<float, 2> wheelTorqueNm{}; // 左、右（車輪機構軸）
    float wheelBalanceTotalNm{0.0f};
    float yawAppliedNm{0.0f};
    bool wheelSaturated{false};
    bool yawIntegralFrozen{false};
    bool fixtureOnly{true};
};

/** @brief C11で検証済みのLQR/脚PD/Roll PD/VMC/Yaw PIを順に組み合わせる。
 *  @note 制御目標の生成や実トルク上限の決定は行わない。
 *  @warning SIL数値比較のみ。MotorCommand、MotorService、CAN/HALへ接続しない。
 *  @spec{CTRL-LQR-001}
 */
class BalanceControl final {
public:
    /** @brief 実機値と別に、数学fixtureのゲインを持つ計算器を組み立てる。
     *  @param[in] config LQRとPD/PI係数、車輪の数学上限。
     *  @spec{CTRL-LQR-001}
     */
    explicit BalanceControl(const BalanceControlConfig& config) noexcept;

    /** @brief ARM前の左右脚長を固定目標として保持し、Yaw積分を初期化する。
     *  @param[in] snapshot すべての観測が成立した、DISARMED中の周期コピー。
     *  @return 有効な脚長ならtrue。前の初期化結果は再利用しない。
     *  @spec{CTRL-LQR-001}
     */
    bool initialize(const ControlSnapshot& snapshot) noexcept;

    /** @brief 既存C11数学で今周期の機構軸トルクを計算する。
     *  @param[in] snapshot 状態推定済みの周期コピー。
     *  @param[in] reference 速度、Yaw、および上流で明示した位置基準。
     *  @return 数学計算がすべて成立したときだけ値を返す。
     *  @note 失敗後の古い計算結果は再利用しない。CANへ配送しない。
     *  @spec{CTRL-LQR-001}
     */
    std::optional<BalanceControlOutput> compute(
        const ControlSnapshot& snapshot, const BalanceReference& reference) noexcept;

    /** @brief 非BALANCING・FAULTまたはARMセッション変更時に履歴を消す。
     *  @spec{CTRL-LQR-001}
     */
    void reset() noexcept;

private:
    BalanceControlConfig config_;
    BLegPdReference legReference_{};
    BYawPiState yawState_{};
};

} // namespace balancing
