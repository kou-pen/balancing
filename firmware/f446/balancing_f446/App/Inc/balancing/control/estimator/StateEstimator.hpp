#pragma once

#include "balancing/app/MotorService.hpp"
#include "balancing/control/estimator/LegKinematics.hpp"
#include "balancing/control/estimator/WheelOdometry.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <variant>

namespace balancing {

/** @brief 外部の姿勢フィルタが確定した機体姿勢と観測鮮度。
 *  @note valid/freshは上流の静止校正・新規測定判定に従う。
 *  @spec{CTRL-EST-001}
 */
struct ImuAttitude {
    std::uint64_t receivedAtUs{0};
    float pitchRad{0.0f};
    float rollRad{0.0f};
    float pitchRateRadS{0.0f};
    float rollRateRadS{0.0f};
    float yawRateRadS{0.0f};
    bool valid{false};
    bool fresh{false};
};

/** @brief 制御周期先頭で固定したIMUとCAN6軸の入力。
 *  @spec{CTRL-EST-001}
 */
struct StateEstimatorInput {
    ImuAttitude imu{};
    MotorFeedbackSnapshot motors{};
};

struct BodyState {
    float pitchRad;
    float rollRad;
    float pitchRateRadS;
    float rollRateRadS;
    float yawRateRadS;
};

struct ChassisState {
    float positionM;
    float velocityMps;
};

struct LegState {
    LegEstimate mechanism;
    float thetaVerticalRad; // fixture座標で機体Pitchを加算した数学値
};

struct WheelState {
    WheelMotion motion;
};

/** @brief 1つの周期の入力から得た、読み取り専用の数学的な制御状態。
 *  @warning fixtureOnly=trueの結果を実機の駆動許可に使わない。
 *  @spec{CTRL-EST-001}
 */
struct ControlSnapshot {
    std::uint64_t sampledAtUs;
    std::chrono::microseconds elapsed;
    std::uint64_t imuReceivedAtUs;
    std::array<std::uint64_t, 4> rs06ReceivedAtUs;
    std::array<std::uint64_t, 2> wheelReceivedAtUs;
    BodyState body;
    ChassisState chassis;
    std::array<LegState, 2> legs;
    std::array<WheelState, 2> wheels;
    bool fixtureOnly{true};
};

enum class StateEstimateError : std::uint8_t {
    InvalidPeriod,
    ImuUnavailable,
    InvalidImu,
    LegFeedbackUnavailable,
    LeftLegUnsolved,
    RightLegUnsolved,
    WheelFeedbackUnavailable,
};

struct StateEstimatorConfig {
    std::array<LegKinematicsConfig, 2> legs{};
    std::array<WheelOdometryConfig, 2> wheels{};
};

/** @brief 500 Hz周期に固定したIMU姿勢、RS06脚4軸、C620車輪2軸を結合する。
 *  @note 受信・センサ新規性は上流で判定。EstimatorはHAL/CAN送信をしない。
 *  @warning 数学fixtureの幾何を使う現実装は、実機BALANCINGを許可しない。
 *  @spec{CTRL-EST-001}
 */
class StateEstimator final {
public:
    /** @brief 検証状態が明示された脚・車輪の設定を構築時に固定する。
     *  @param[in] config 左右脚と車輪の独立した変換・幾何設定。
     *  @spec{CTRL-EST-001}
     */
    explicit StateEstimator(const StateEstimatorConfig& config) noexcept;

    /** @brief 一周期の同じ観測時刻から制御スナップショットを作る。
     *  @param[in] input MotorServiceと姿勢フィルタの最新コピー。
     *  @param[in] elapsed Applicationが単調時計から求めた周期間隔。
     *  @return 成功時ControlSnapshot、欠損・不成立時は部位別の失敗理由。
     *  @note 入力不成立の周期は部分的なControlSnapshotを返さない。
     *  @spec{CTRL-EST-001}
     */
    std::variant<ControlSnapshot, StateEstimateError>
    update(const StateEstimatorInput& input, std::chrono::microseconds elapsed) noexcept;

    /** @brief 観測原点の再設定時に車輪の積算基準を消す。
     *  @note 通常のARM/DISARMでは呼ばない。IMU姿勢フィルタ自体のresetは上流担当。
     *  @spec{CTRL-EST-001}
     */
    void reset() noexcept;

private:
    StateEstimatorConfig config_;
    WheelOdometry odometry_;
};

} // namespace balancing
