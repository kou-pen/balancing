#pragma once
#include "balancing/device/Bno055.hpp"
#include <array>
#include "balancing/control/estimator/StateEstimator.hpp"
namespace balancing {
/** @brief センサxyzを機体xyzへ写す直交軸変換。 */
struct ImuMount {
    std::array<std::uint8_t,3> sensor_index{}; /**< 機体X/Y/Zに対応するセンサ成分0..2。重複不可。 */
    std::array<std::int8_t,3> direction{}; /**< 軸ごとの符号。+1/-1のみ。 */
    bool verified{}; /**< 取付方向を実測・CADで確認済み、または人工fixture。 */
};

/**
 * @brief BNO055 ACCGYROモードを想定したHAL非依存の数学設定。
 * @spec{CTRL-EST-001}
 * @warning 実機の取付方向、係数、静止判定条件は未確定。ホストでは人工値のみ。
 */
struct AttitudeFilterConfig {
    ImuMount mount{};
    std::uint32_t bias_samples_required{}; /**< 事前に静止と証明した独立サンプル数。2以上。 */
    float gravity_min_m_s2{}; /**< 静止重力と見なす加速度ノルムの下限 [m/s^2]。 */
    float gravity_max_m_s2{}; /**< 静止重力と見なす加速度ノルムの上限 [m/s^2]。 */
    float gyro_alpha{}; /**< 相補係数。予測側の重み。0..1未満。 */
    float max_sample_dt_s{}; /**< 測定更新の最大許容間隔 [s]。 */
    bool verified{}; /**< 明示した実機校正、または人工数学fixture。 */
};

/** @brief 最後に新規取得した観測時刻で固定された推定値。 */
struct AttitudeEstimate {
    bool valid{};
    bool accel_fused{}; /**< 重力範囲内で加速度角度を補正へ使用したか。 */
    std::uint64_t stamp_us{}; /**< 元の新規センササンプル時刻 [us]。 */
    float pitch_rad{}; /**< 機体X前・Y左・Z上のfixture角 [rad]。 */
    float roll_rad{}; /**< 同じ座標系での角 [rad]。 */
    float pitch_rate_rad_s{}; /**< ジャイロ機体Y軸 [rad/s]。厳密なEuler Pitch微分ではない。 */
    float roll_rate_rad_s{}; /**< ジャイロ機体X軸 [rad/s]。厳密なEuler Roll微分ではない。 */
    float yaw_rate_rad_s{}; /**< ジャイロ機体Z軸 [rad/s]。絶対Yaw角や厳密なYaw微分ではない。 */
};

/** @brief 単一所有者が保持するバイアス累積と相補フィルタ状態。 */
struct AttitudeFilterState {
    AttitudeFilterConfig config{};
    std::array<double,3> gyro_sum_rad_s{};
    std::array<float,3> gyro_bias_rad_s{};
    std::uint32_t bias_samples_collected{};
    bool bias_ready{};
    bool initialized{};
    bool has_last_calibration_stamp{};
    std::uint64_t last_calibration_stamp_us{};
    AttitudeEstimate latest{};
};

/** @brief HALに依存しない相補フィルタの数値演算。状態の製品所有者はImuAttitudeSource。 */
namespace attitude_math {
/**
 * @brief 既存のバイアスと姿勢を破棄し、静止測定から再学習する状態に戻す。
 * @spec{CTRL-EST-001}
 * @param[out] filter フィルタ状態。失敗時はゼロ初期化される。
 * @param[in] config 外部が検証済みと判断した数学・取付設定。
 * @return 有効な設定の場合true。
 * @warning 実機で検証していない取付マップを自動確定しない。
 */
bool initialize(AttitudeFilterState& filter, const AttitudeFilterConfig& config) noexcept;

/**
 * @brief 明示的に静止が証明された新規サンプルを用いてジャイロ平均バイアスを算出。
 * @spec{CTRL-EST-001}
 * @param[in,out] filter 初期化済みのフィルタ。
 * @param[in] sample 校正済みのSI加速度・ジャイロ値。
 * @param[in] stationary_proven 静止の独立した証拠。加速度ノルムだけでは代用不可。
 * @param[in] new_measurement_proven I2C再読出しではなく真の新規測定の証拠。
 * @param[in] stamp_us センサ測定時刻 [us]。
 * @param[in] now_us 同一単調時計での受付時刻 [us]。
 * @return 正当な静止サンプルを1件受理した場合true。
 * @warning 必要数が揃うまで推定値を利用してはいけない。
 */
bool addStaticSample(AttitudeFilterState& filter, const ImuSample& sample,
                                  bool stationary_proven, bool new_measurement_proven,
                                  std::uint64_t stamp_us, std::uint64_t now_us) noexcept;

/**
 * @brief 静止校正後のジャイロ予測と加速度角度の相補フィルタ。
 * @spec{CTRL-EST-001}
 * @param[in,out] filter 静止バイアスが有効なフィルタ。
 * @param[in] sample 新規測定の加速度・ジャイロ値。
 * @param[in] new_measurement_proven 新しいBNO055測定であることの外部証拠。
 * @param[in] stamp_us センサ元時刻 [us]。同時刻や逆行を拒否。
 * @param[in] now_us 同じ時計での受理時刻 [us]。
 * @param[out] out 成功時の推定。失敗時はゼロ化。
 * @return サンプルを正しく受理できたらtrue。
 * @warning 新規測定間隔が大きいときは推測で再初期化せずfalseを返す。
 *          ジャイロ機体X/Y/ZをEuler角速度に近似する小角モデルである。
 *          大きな傾き・Yaw結合でのEuler角微分として使用しない。
 *          Yaw角の絶対方位は計算しない。実機姿勢制御には未接続。
 */
bool update(AttitudeFilterState& filter, const ImuSample& sample,
                       bool new_measurement_proven, std::uint64_t stamp_us,
                       std::uint64_t now_us, AttitudeEstimate& out) noexcept;

} // namespace attitude_math

/** @brief 既存相補フィルタと静止バイアスの単一所有者。
 * @spec{CTRL-EST-001}
 */
class ImuAttitudeSource final {
public:
    /** @brief 起動時の検証済み数学設定。未検証なら姿勢無効。
     * @spec{CTRL-EST-001}
     */
    explicit ImuAttitudeSource(const AttitudeFilterConfig& config) noexcept;
    /** @brief 新規性が証明された観測だけを受理する。静止証明は独立入力。
     * @spec{CTRL-EST-001}
     */
    bool accept(const TimedImuSample& sample,bool stationaryProven,std::uint64_t nowUs) noexcept;
    /** @brief 元時刻のまま50ms鮮度を評価する。
     * @spec{CTRL-EST-001}
     */
    ImuAttitude attitude(std::uint64_t nowUs) const noexcept;
    /** @brief 通信・センサ故障後は再初期化まで旧姿勢を使わない。
     * @spec{CTRL-EST-001}
     */
    void invalidate() noexcept { failed_=true; }
private:
    AttitudeFilterState filter_{};
    bool failed_{false};
};
}
