#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace balancing {

/** @brief RS06モータ軸から平面2入力の機構入力軸への校正。
 *  @note 歯数、方向、ゼロ点は左右各脚で独立。実機未測定値を自動補完しない。
 *  @spec{CTRL-EST-001}
 */
struct LegMotorTransform {
    std::array<float, 2> motorZeroRad{};
    std::array<float, 2> jointOffsetRad{};
    std::array<float, 2> direction{};
    float drivingTeeth{0.0f};
    float drivenTeeth{0.0f};
    bool calibrated{false};
};

/** @brief 既存C11閉リンクの数学fixture。実機六節リンクCADの幾何ではない。
 *  @spec{CTRL-EST-001}
 */
struct LegFixtureGeometry {
    float baseSpanM{0.0f};
    float firstCrankM{0.0f};
    float secondCrankM{0.0f};
    float firstRodM{0.0f};
    float secondRodM{0.0f};
    float minCrossSine{0.0f};
    float minLegLengthM{0.0f};
    int branch{0}; // -1 or +1。組立分岐は計算中に自動変更しない。
};

struct LegKinematicsConfig {
    LegMotorTransform motor{};
    LegFixtureGeometry fixture{};
};

struct LegEstimate {
    float lengthM{0.0f};
    float thetaBodyRad{0.0f};
    float lengthRateMps{0.0f};
    float thetaRateRadS{0.0f};
    std::array<float, 2> lengthJacobianM{};
    std::array<float, 2> thetaJacobian{};
};

enum class LegSolveError : std::uint8_t {
    None,
    InvalidCalibration,
    InvalidGeometry,
    Unreachable,
    Singular,
};

struct LegSolveResult {
    std::optional<LegEstimate> state{};
    LegSolveError error{LegSolveError::None};
};

/** @brief 二つのRS06軸角から閉リンクの脚長・角度・ヤコビアンを解く数学境界。
 *  @warning 現段階では旧C11の人工幾何を再利用。実機のリンク構成・原点・符号
 *           がCADと照合されるまで、制御出力を許可するモデルではない。
 *  @spec{CTRL-EST-001}
 */
class LegKinematics final {
public:
    /** @brief モータ角・速度を機構入力角へ変換し、指定された分岐で解く。
     *  @param[in] config 機構比・原点・分岐を固定した数学fixture。
     *  @param[in] angleRad 片脚のモータA・Bの角度 [rad]。
     *  @param[in] velocityRadS 片脚のモータA・Bの角速度 [rad/s]。
     *  @return 成立した脚の状態、または校正・到達性・特異性のエラー。
     *  @spec{CTRL-EST-001}
     */
    static LegSolveResult solve(const LegKinematicsConfig& config,
                                const std::array<float, 2>& angleRad,
                                const std::array<float, 2>& velocityRadS) noexcept;
};

} // namespace balancing
