#ifndef BALANCING_ATTITUDE_FILTER_H
#define BALANCING_ATTITUDE_FILTER_H

#include "balancing/control/estimator/balancing_observation.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief センサxyzを機体xyzへ写す直交軸変換。 */
typedef struct {
    uint8_t sensor_index[3]; /**< 機体X/Y/Zに対応するセンサ成分0..2。重複不可。 */
    int8_t direction[3]; /**< 軸ごとの符号。+1/-1のみ。 */
    bool verified; /**< 取付方向を実測・CADで確認済み、または人工fixture。 */
} BImuMount;

/**
 * @brief BNO055 ACCGYROモードを想定したHAL非依存の数学設定。
 * @spec{CTRL-EST-001}
 * @warning 実機の取付方向、係数、静止判定条件は未確定。ホストでは人工値のみ。
 */
typedef struct {
    BImuMount mount;
    uint32_t bias_samples_required; /**< 事前に静止と証明した独立サンプル数。2以上。 */
    float gravity_min_m_s2; /**< 静止重力と見なす加速度ノルムの下限 [m/s^2]。 */
    float gravity_max_m_s2; /**< 静止重力と見なす加速度ノルムの上限 [m/s^2]。 */
    float gyro_alpha; /**< 相補係数。予測側の重み。0..1未満。 */
    float max_sample_dt_s; /**< 測定更新の最大許容間隔 [s]。 */
    bool verified; /**< 明示した実機校正、または人工数学fixture。 */
} BAttitudeFilterConfig;

/** @brief 最後に新規取得した観測時刻で固定された推定値。 */
typedef struct {
    bool valid;
    bool accel_fused; /**< 重力範囲内で加速度角度を補正へ使用したか。 */
    uint64_t stamp_us; /**< 元の新規センササンプル時刻 [us]。 */
    float pitch_rad; /**< 機体X前・Y左・Z上のfixture角 [rad]。 */
    float roll_rad; /**< 同じ座標系での角 [rad]。 */
    float pitch_rate_rad_s; /**< ジャイロバイアス除去後 [rad/s]。 */
    float roll_rate_rad_s; /**< ジャイロバイアス除去後 [rad/s]。 */
    float yaw_rate_rad_s; /**< ジャイロバイアス除去後 [rad/s]。Yaw積分角ではない。 */
} BAttitudeEstimate;

/** @brief 単一所有者が保持するバイアス累積と相補フィルタ状態。 */
typedef struct {
    BAttitudeFilterConfig config;
    double gyro_sum_rad_s[3];
    float gyro_bias_rad_s[3];
    uint32_t bias_samples_collected;
    bool bias_ready;
    bool initialized;
    bool has_last_calibration_stamp;
    uint64_t last_calibration_stamp_us;
    BAttitudeEstimate latest;
} BAttitudeFilter;

/**
 * @brief 既存のバイアスと姿勢を破棄し、静止測定から再学習する状態に戻す。
 * @spec{CTRL-EST-001}
 * @param[out] filter フィルタ状態。失敗時はゼロ初期化される。
 * @param[in] config 外部が検証済みと判断した数学・取付設定。
 * @return 有効な設定の場合true。
 * @warning 実機で検証していない取付マップを自動確定しない。
 */
bool b_attitude_init(BAttitudeFilter *filter, const BAttitudeFilterConfig *config);

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
bool b_attitude_add_static_sample(BAttitudeFilter *filter, const BImuSample *sample,
                                  bool stationary_proven, bool new_measurement_proven,
                                  uint64_t stamp_us, uint64_t now_us);

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
 *          Yaw角の絶対方位は計算しない。実機姿勢制御には未接続。
 */
bool b_attitude_update(BAttitudeFilter *filter, const BImuSample *sample,
                       bool new_measurement_proven, uint64_t stamp_us,
                       uint64_t now_us, BAttitudeEstimate *out);

#ifdef __cplusplus
}
#endif
#endif /* BALANCING_ATTITUDE_FILTER_H */
