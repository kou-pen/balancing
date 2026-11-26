#include "balancing/control/estimator/LegKinematics.hpp"
#include <cmath>

namespace balancing {
/*
 * これは実機六節リンクの確定モデルではなく、閉リンク演算の数学fixture。
 * CADと関節の接続・出力点・原点が一致するまで実機トルク指令へ使わない。
 */
namespace {
constexpr float kPi = 3.14159265358979323846f;

/* 入力2軸のうち片側だけでもNaN/Infなら拘束式・ヤコビアンに伝播する。 */
static bool finitePair(const std::array<float,2>& pair) noexcept
{
    return std::isfinite(pair[0]) && std::isfinite(pair[1]);
}

/*
 * 符号は単なる倍率ではなく回転軸方向を定義するので±1だけ認める。
 * verified相当の校正フラグは外部から渡される契約であり、
 * 実機エンコーダのゼロ点・多回転の正しさをここで認定しない。
 */
static bool validMotorMap(const LegMotorTransform& map) noexcept
{
    if (!map.calibrated ||
        !std::isfinite(map.drivingTeeth) || !std::isfinite(map.drivenTeeth) ||
        map.drivingTeeth <= 0.0f || map.drivenTeeth <= 0.0f) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (!std::isfinite(map.motorZeroRad[i]) ||
            !std::isfinite(map.jointOffsetRad[i]) ||
            (map.direction[i] != 1.0f && map.direction[i] != -1.0f)) {
            return false;
        }
    }
    return true;
}

} // namespace

bool LegKinematics::motorToJoint(const LegMotorTransform& map, const std::array<float,2>& motor_angle_rad,
                          const std::array<float,2>& motor_rate_rad_s, LegJointState& joint) noexcept
{
    joint = {};
    if (!validMotorMap(map) || !finitePair(motor_angle_rad) ||
        !finitePair(motor_rate_rad_s)) {
        return false;
    }
    /* 外部伝達比は入力軸の角変位倍率。原点差と符号は軸ごとに適用する。
     * 差動速度には原点オフセットが寄与しない点に注意する。 */
    const float ratio = map.drivingTeeth / map.drivenTeeth;
    if (!std::isfinite(ratio) || ratio <= 0.0f) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        const float scale = ratio * map.direction[i];
        const float q = scale * (motor_angle_rad[i] - map.motorZeroRad[i]) +
                        map.jointOffsetRad[i];
        const float qdot = scale * motor_rate_rad_s[i];
        if (!std::isfinite(q) || !std::isfinite(qdot)) {
            /* 途中の片軸だけ成功した状態を下流へ流さない。 */
            joint = {};
            return false;
        }
        joint.angleRad[i] = q;
        joint.velocityRadS[i] = qdot;
    }
    return true;
}

/* 負長・ゼロ長・無効な特異閾値を早期拒否し、未確定CAD値を補完しない。 */
static bool validGeometry(const LegFixtureGeometry& g) noexcept
{
    return std::isfinite(g.baseSpanM) &&
           std::isfinite(g.firstCrankM) && std::isfinite(g.secondCrankM) &&
           std::isfinite(g.firstRodM) && std::isfinite(g.secondRodM) &&
           std::isfinite(g.minCrossSine) &&
           std::isfinite(g.minLegLengthM) &&
           g.baseSpanM > 0.0f && g.firstCrankM > 0.0f &&
           g.secondCrankM > 0.0f && g.firstRodM > 0.0f &&
           g.secondRodM > 0.0f && g.minCrossSine > 0.0f &&
           g.minCrossSine < 1.0f && g.minLegLengthM > 0.0f;
}

LegSolveError LegKinematics::forward(const LegFixtureGeometry& g,
                                 const LegJointState& joint, LegEstimate& out) noexcept
{
    out = {};
    if (!validGeometry(g) || (g.branch != -1 && g.branch != 1) ||
        !finitePair(joint.angleRad) || !finitePair(joint.velocityRadS)) {
        return LegSolveError::InvalidGeometry;
    }

    /*
     * 数学fixtureの平面内座標：左固定軸A=(0,0)、右固定軸B=(base_span,0)。
     * 入力角q0,q1からクランク先端E,Fを求める。E,Fを中心とした2円の
     * 交点Pを「仮想足先」と仮定するが、実機六節リンクの接続ではない。
     * 入力角を2π周期へ正規化し、大角入力で三角関数の誤差を増やさない。
     */
    const float q0 = std::remainder(joint.angleRad[0], 2.0f * kPi);
    const float q1 = std::remainder(joint.angleRad[1], 2.0f * kPi);
    const float ex = g.firstCrankM * std::cos(q0);
    const float ey = g.firstCrankM * std::sin(q0);
    const float fx = g.baseSpanM + g.secondCrankM * std::cos(q1);
    const float fy = g.secondCrankM * std::sin(q1);
    const float dx = fx - ex;
    const float dy = fy - ey;
    const float d = std::hypot(dx, dy);
    if (!std::isfinite(d) || d <= 0.0f) {
        return LegSolveError::Singular;
    }
    const float left = g.firstRodM;
    const float right = g.secondRodM;
    /* 2円の交差可能条件：|left-right| <= |F-E| <= left+right。
     * 交点が存在しない場合は近傍へ勝手に丸めず到達不能を返す。 */
    const float reach_max = left + right;
    const float reach_min = std::fabs(left - right);
    if (d > reach_max || d < reach_min) {
        return LegSolveError::Unreachable;
    }

    /*
     * EからEF方向への射影長aと垂直距離hを円の幾何から算出。
     * P = E + a*unit(EF) ± h*perp(unit(EF))。branchの±は組立分岐で、
     * 同じ角度で別の脚姿勢を取り得るため自動切替をしない。
     */
    const float projection = (left * left - right * right + d * d) / (2.0f * d);
    float height_sq = left * left - projection * projection;
    if (!std::isfinite(height_sq)) {
        return LegSolveError::InvalidGeometry;
    }
    /*
     * h^2がわずかに負になる浮動小数点丸めだけをepsで吸収する。
     * 2円が接する幾何は特異なので、後段で外積により必ず拒否する。
     * epsは数学fixtureの数値許容差であり、実機の安全しきい値ではない。
     */
    const float eps = 16.0f * 1.1920928955078125e-7f * (left * left + right * right);
    if (height_sq < -eps) {
        return LegSolveError::Unreachable;
    }
    height_sq = std::fmax(0.0f, height_sq);
    const float h = std::sqrt(height_sq);
    const float ux = dx / d;
    const float uy = dy / d;
    const float side = static_cast<float>(g.branch);
    const float px = ex + projection * ux + side * h * (-uy);
    const float py = ey + projection * uy + side * h * ux;
    const float c0x = px - ex;
    const float c0y = py - ey;
    const float c1x = px - fx;
    const float c1y = py - fy;
    /*
     * 拘束勾配(PE,PF)を行とする2×2行列の行列式。
     * 2本の先端ロッドがほぼ平行なときdet→0となり逆行列が発散する。
     * |det|/(left*right)=|sin(ロッド間角)|で条件を判定し、特異を拒否する。
     */
    const float determinant = c0x * c1y - c0y * c1x;
    const float cross_sine = std::fabs(determinant) / (left * right);
    if (!std::isfinite(cross_sine) || cross_sine < g.minCrossSine) {
        return LegSolveError::Singular;
    }

    /* 脚原点は固定軸A,Bの中点。実際の機体座標系との対応は未確定。
     * 仮想脚長0では角度の微分が定義できないため別途拒否する。 */
    const float leg_x = px - g.baseSpanM * 0.5f;
    const float leg_y = py;
    const float length = std::hypot(leg_x, leg_y);
    if (!std::isfinite(length) || length < g.minLegLengthM) {
        return LegSolveError::Singular;
    }
    /*
     * 円拘束 |P-E|^2=left^2 と |P-F|^2=right^2 を入力角で陰関数微分する。
     * (P-E)・dP=(P-E)・dE、(P-F)・dP=(P-F)・dF の線形2元系を解き、
     * 各モータ入力角に対するPの偏微分(dPx/dq, dPy/dq)を得る。
     * 先に行列式の特異判定を終えているので、ここでのみdetで除算する。
     */
    const float rhs0 = c0x * (-g.firstCrankM * std::sin(q0)) +
                       c0y * (g.firstCrankM * std::cos(q0));
    const float rhs1 = c1x * (-g.secondCrankM * std::sin(q1)) +
                       c1y * (g.secondCrankM * std::cos(q1));
    const float jpx[2] = {rhs0 * c1y / determinant, -c0y * rhs1 / determinant};
    const float jpy[2] = {-rhs0 * c1x / determinant, c0x * rhs1 / determinant};

    LegEstimate value{};
    value.footXM = leg_x;
    value.footYM = leg_y;
    value.lengthM = length;
    /* θ=atan2(脚x,-脚y)と定めることで数学fixtureでは下向きが0。
     * 実機のpitch軸・符号・ゼロ姿勢と一致するかはCAD校正が必要。 */
    value.thetaBodyRad = std::atan2(leg_x, -leg_y);
    const float inv_length = 1.0f / length;
    const float inv_length_sq = inv_length * inv_length;
    /*
     * dL/dq=(x*dPx/dq+y*dPy/dq)/L。
     * dθ/dq=(x*dPy/dq-y*dPx/dq)/L^2（atan2(x,-y)の微分）。
     * 2軸の速度は同じ解析Jとの積で求め、有限差分はテスト側でのみ使用。
     */
    for (unsigned i = 0; i < 2; ++i) {
        value.lengthJacobianM[i] =
            (leg_x * jpx[i] + leg_y * jpy[i]) * inv_length;
        value.thetaJacobian[i] =
            (leg_x * jpy[i] - leg_y * jpx[i]) * inv_length_sq;
        value.lengthRateMps += value.lengthJacobianM[i] * joint.velocityRadS[i];
        value.thetaRateRadS += value.thetaJacobian[i] * joint.velocityRadS[i];
    }
    if (!std::isfinite(value.footXM) || !std::isfinite(value.footYM) ||
        !std::isfinite(value.lengthM) || !std::isfinite(value.thetaBodyRad) ||
        !std::isfinite(value.lengthRateMps) || !std::isfinite(value.thetaRateRadS) ||
        !finitePair(value.lengthJacobianM) ||
        !finitePair(value.thetaJacobian)) {
        return LegSolveError::InvalidGeometry;
    }
    out = value;
    return LegSolveError::None;
}

bool LegKinematics::virtualToJointTorque(const LegEstimate& state, float axialForceN,
                                   float pitchTorqueNm, std::array<float,2>& torque) noexcept
{
    torque = {};
    if (!std::isfinite(axialForceN) || !std::isfinite(pitchTorqueNm) ||
        !finitePair(state.lengthJacobianM) ||
        !finitePair(state.thetaJacobian)) {
        return false;
    }
    /* 仮想仕事の保存：τ_joint=J^T*[軸力, 仮想脚トルク]。
     * 実アクチュエータの上限、ガスシリンダ補償、摩擦は含まない。 */
    for (unsigned i = 0; i < 2; ++i) {
        const float value = axialForceN * state.lengthJacobianM[i] +
                            pitchTorqueNm * state.thetaJacobian[i];
        if (!std::isfinite(value)) {
            torque = {};
            return false;
        }
        torque[i] = value;
    }
    return true;
}

bool LegKinematics::jointToIdealMotorTorque(const LegMotorTransform& map,
                                       const std::array<float,2>& joint_torque,
                                       std::array<float,2>& motor_torque) noexcept
{
    motor_torque = {};
    if (!validMotorMap(map) || !finitePair(joint_torque)) {
        return false;
    }
    /* 理想歯車の入力仕事率保存：τ_motor = τ_joint * dq_joint/dq_motor。
     * 効率・バックラッシュは仮定していないため、安全なトルク指令ではない。 */
    const float ratio = map.drivingTeeth / map.drivenTeeth;
    for (unsigned i = 0; i < 2; ++i) {
        const float value = joint_torque[i] * map.direction[i] * ratio;
        if (!std::isfinite(value)) {
            motor_torque = {};
            return false;
        }
        motor_torque[i] = value;
    }
    return true;
}

LegSolveResult LegKinematics::solve(const LegKinematicsConfig& config,
    const std::array<float,2>& angleRad,const std::array<float,2>& velocityRadS) noexcept
{
    LegJointState joint{};
    // 校正不成立時は生のモータ角を代用しない。
    if (!motorToJoint(config.motor,angleRad,velocityRadS,joint))
        return {std::nullopt,LegSolveError::InvalidCalibration};
    LegEstimate result{};
    const auto error=forward(config.fixture,joint,result);
    if (error!=LegSolveError::None) return {std::nullopt,error};
    return {result,LegSolveError::None};
}

} // namespace balancing
