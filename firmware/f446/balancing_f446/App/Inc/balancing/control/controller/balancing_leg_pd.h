#ifndef BALANCING_LEG_PD_H
#define BALANCING_LEG_PD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 左右脚の測定長・伸縮速度。位置[m]、速度[m/s]。 */
typedef struct {
    float left_length_m;
    float right_length_m;
    float left_velocity_m_s;
    float right_velocity_m_s;
    float roll_rad;
    float roll_rate_rad_s;
} BLegPdFeedback;

/** @brief ARM直前に測定した固定脚長目標。装置固有の可動限界は含まない。 */
typedef struct {
    float common_length_m; /**< (左+右)/2 [m]。 */
    float half_difference_m; /**< (左-右)/2 [m]。 */
    bool valid; /**< 新鮮な観測からDISARMED中に取得できた場合のみtrue。 */
} BLegPdReference;

/** @brief 物理検証が終わるまで無効とする脚長・Rollゲイン。 */
typedef struct {
    float length_kp_n_m; /**< 支持力 / 脚長偏差 [N/m]。 */
    float length_kd_n_s_m; /**< 支持力 / 伸縮速度 [N s/m]。 */
    float roll_kp_n_rad; /**< 左右差動支持力 / Roll偏差 [N/rad]。 */
    float roll_kd_n_s_rad; /**< 左右差動支持力 / Roll角速度 [N s/rad]。 */
    float gravity_feedforward_n; /**< 片脚の重力補償支持力 [N]。 */
    bool verified; /**< 外部で物理モデル・符号を検証した場合にだけtrue。 */
} BLegPdGains;

/** @brief VMCへ渡す脚別の抽象支持力。モータ指令ではない。 */
typedef struct {
    float left_force_n;
    float right_force_n;
    float roll_differential_n;
} BLegPdOutput;

/**
 * @brief DISARMED中の新鮮な脚長から左右を保つ固定脚長目標を取得する。
 * @spec{CTRL-LQR-001}
 * @param[out] reference 保存先。失敗時は初期化して無効化する。
 * @param[in] feedback 同一周期で固定した左右の測定値。
 * @param[in] disarmed 真なら操縦系がDISARMEDにあることを示す。
 * @param[in] lengths_fresh 真なら左右の測定がともに新鮮なことを示す。
 * @return 有限・正の脚長を持ち、安全ゲートを満たした場合だけtrue。
 * @warning 目標を取得しただけでARM許可を出さない。実機可動域も検証しない。
 */
bool b_leg_pd_capture(BLegPdReference *reference, const BLegPdFeedback *feedback,
                      bool disarmed, bool lengths_fresh);

/**
 * @brief 脚長PDとRoll PDを合成し、左右の仮想支持力を計算する。
 * @spec{CTRL-LQR-001}
 * @param[in] reference ARM前に確定した目標（左=L0+差、右=L0-差）。
 * @param[in] gains 外部検証済みの非負のPDゲインと片脚重力補償値。
 * @param[in] feedback 同一周期の脚長・速度・Roll角・角速度。
 * @param[out] output 支持力 [N]。失敗時は常に全成分ゼロ。
 * @return 目標・ゲイン・測定と演算結果が有効な場合のみtrue。
 * @warning 実機の左右符号・Roll軸・可動域・力上限を保証しない。
 *          各RS06のトルクリミットはVMC変換後の別層で適用する。
 */
bool b_leg_pd_compute(const BLegPdReference *reference, const BLegPdGains *gains,
                      const BLegPdFeedback *feedback, BLegPdOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_LEG_PD_H */
