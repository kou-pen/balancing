#ifndef BALANCING_OBSERVATION_H
#define BALANCING_OBSERVATION_H

#include "balancing/core/balancing_observation_time.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 校正後のIMU物理値。軸向きとバイアスの適合は呼出元で検証する。 */
typedef struct {
    float accel_m_s2[3]; /**< X/Y/Z加速度 [m/s^2]。 */
    float gyro_rad_s[3]; /**< X/Y/Z角速度 [rad/s]。 */
} BImuSample;

/** @brief 車輪ロータ角の変位を車輪移動量に換算する設定。実機値の検証は別。 */
typedef struct {
    float wheel_radius_m;            /**< 車輪半径 [m]。正値。 */
    float motor_revs_per_wheel_rev;  /**< 車輪1回転あたりのロータ回転数。正値。 */
    float direction;                 /**< 左右の正方向。+1または-1。 */
    bool verified;                   /**< 外部の校正・設定検証状態。 */
} BWheelObservationConfig;

/** @brief 車輪ごとに保持する最後の観測値。位置は受信開始時からの変位。 */
typedef struct {
    uint16_t encoder_raw; /**< 最終ロータ角 [0,8191]。 */
    int16_t rpm;          /**< ロータ速度 [rpm]。 */
    float position_m;     /**< 初回サンプル位置0からの積算値 [m]。 */
    float velocity_m_s;   /**< 車輪回転から換算した周速度 [m/s]。 */
} BWheelObservation;

/** @brief 非同期受信をメイン処理で逐次投入する1所有者の観測ストア。 */
typedef struct {
    BObservedStamps stamps;
    BImuSample imu;
    BWheelObservation wheel[2];
    BWheelObservationConfig wheel_config[2]; /**< 積算中の換算設定。変更時は再初期化が必要。 */
} BObservationStore;

/** @brief 500 Hz制御ティックの先頭でコピーする、その周期専用の値。 */
typedef struct {
    uint64_t frozen_at_us;
    BObservedStamps stamps;
    BImuSample imu;
    BWheelObservation wheel[2];
} BObservationSnapshot;

/**
 * @brief 観測ストアをゼロ初期化。valid=falseで開始する。
 * @spec{CTRL-EST-001}
 * @param[out] store 初期化するストア。
 */
void b_observation_init(BObservationStore *store);

/**
 * @brief 新規測定を**証明した**IMUサンプルのみ受け入れる。
 * @spec{CTRL-EST-001}
 * @param[in,out] store 唯一の書込み所有者が扱う観測ストア。
 * @param[in] value 単位・軸・バイアスが校正された観測値。
 * @param[in] new_measurement_proven 読み出し成功ではなくセンサの新規更新の証拠。
 * @param[in] stamp_us 実際の新規測定時刻 [us]。
 * @param[in] now_us 同じ単調増加時刻系における受付時刻 [us]。
 * @return 受理した場合だけtrue。拒否時はストアを変更しない。
 * @warning BNO055のI2C読出し成功だけでtrueにしてはいけない。
 */
bool b_observation_imu(BObservationStore *store, const BImuSample *value,
                       bool new_measurement_proven, uint64_t stamp_us,
                       uint64_t now_us);

/**
 * @brief C620生エンコーダ・RPMから1回転折返しを扱い車輪状態を更新。
 * @spec{CTRL-EST-001}
 * @spec{PROTO-C620-001}
 * @param[in,out] store 観測ストア。
 * @param[in] wheel_index 左:0、右:1。
 * @param[in] config 実車輪半径・ロータ/車輪比・左右符号。
 * @param[in] encoder_raw C620の0..8191のロータ1回転内角度。
 * @param[in] rpm C620から受信したロータRPM。
 * @param[in] stamp_us 受信した新規フィードバックの時刻 [us]。
 * @param[in] now_us 同じ時計における受付時刻 [us]。
 * @return 時刻と角度差が一意な場合だけtrue。失敗は更新せず従前の鮮度を保持。
 * @warning サンプル間に半回転以上の移動があり得る場合は巻数が復元不能。
 *          4096カウント差は方向が一意でないため拒否する。
 * @warning 位置積算中に半径・比・方向を変更する場合はストアを再初期化する。
 */
bool b_observation_wheel(BObservationStore *store, unsigned wheel_index,
                         const BWheelObservationConfig *config,
                         uint16_t encoder_raw, int16_t rpm,
                         uint64_t stamp_us, uint64_t now_us);

/**
 * @brief 同じ制御周期で使用する観測値と**元の測定時刻**をコピーする。
 * @spec{CTRL-EST-001}
 * @param[in] store 受信側で更新済みのストア。
 * @param[in] now_us コピー時刻。測定の鮮度とは無関係。
 * @param[out] snapshot コピー先。失敗時ゼロ。
 * @return 引数が有効ならtrue。
 * @warning 割込みと並列に書込み・読取りしない。受信イベントを主ループへ
 *          直列化した上で呼ぶ（IRQ用ロックフリー設計は別作業）。
 */
bool b_observation_freeze(const BObservationStore *store, uint64_t now_us,
                          BObservationSnapshot *snapshot);

#ifdef __cplusplus
}
#endif
#endif /* BALANCING_OBSERVATION_H */
