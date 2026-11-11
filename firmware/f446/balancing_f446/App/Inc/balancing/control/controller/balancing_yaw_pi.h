#ifndef BALANCING_YAW_PI_H
#define BALANCING_YAW_PI_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Yaw角速度PIゲイン。実機で確認するまではverified=false。 */
typedef struct {
    float kp_nm_s_rad; /**< P係数 [N m / (rad/s)]。 */
    float ki_nm_rad; /**< 積分係数 [N m / rad]。 */
    float integral_limit_nm; /**< 積分成分の絶対値上限 [N m]。 */
    bool verified; /**< 外部で実機妥当性を承認した場合のみtrue。 */
} BYawPiGains;

/** @brief 単一所有者が維持する積分器。ISRで同時更新しない。 */
typedef struct {
    float integral_nm; /**< トルク単位の積分成分。 */
    bool active; /**< 前回演算がBALANCING中だったか。 */
} BYawPiState;

/** @brief 一周期の測定と、倒立優先配分後に残ったYaw容量。 */
typedef struct {
    float target_rate_rad_s; /**< 操作層が加減速制限したYaw角速度目標。 */
    float measured_rate_rad_s; /**< ジャイロから推定したYaw角速度。 */
    float dt_s; /**< 実際の制御演算間隔 [s]。 */
    float available_delta_nm; /**< 車輪倒立成分確保後の左右差動余力 [N m]。 */
    bool balancing; /**< 実制御状態がBALANCINGの場合のみtrue。 */
    bool gyro_fresh; /**< ジャイロの新規測定時刻から鮮度判定した結果。 */
} BYawPiInput;

/** @brief 車輪配分器へ渡す左右逆符号の差動トルク要求。 */
typedef struct {
    float delta_nm; /**< 右へ+、左へ-として与える差動 [N m]。 */
    float integral_nm; /**< 採用された積分成分 [N m]。 */
    bool saturated; /**< 出力容量による差動制限が発生した場合。 */
    bool integral_frozen; /**< 飽和悪化を防ぐため積分を停止した場合。 */
} BYawPiOutput;

/**
 * @brief 積分器をゼロにし、次のARMセッション開始を準備する。
 * @spec{CTRL-LQR-001}
 * @param[in,out] state 積分状態。NULLなら何もしない。
 * @warning ARM開始・モード変更・FAULTで呼ぶこと。通常の非BALANCING
 *          ステップでも自動リセットするが、呼出し省略時に備えて明示する。
 */
void b_yaw_pi_reset(BYawPiState *state);

/**
 * @brief 角速度追従Yaw PIを条件付き積分・出力制限付きで1回更新する。
 * @spec{CTRL-LQR-001}
 * @param[in,out] state 前回までの積分状態。失敗時にはリセットする。
 * @param[in] gains 外部検証済みの非負ゲインと積分上限。
 * @param[in] input 一周期の目標・観測・時間間隔・差動余力・動作状態。
 * @param[out] output 差動要求と飽和・積分停止フラグ。失敗時ゼロ。
 * @return 入力が有効な場合true。BALANCING外ではtrueでゼロを返す。
 * @warning available_delta_nmはb_wheel_allocateと同じ倒立優先の
 *          左右容量から呼出し元が求める。ここではCAN/ARM操作を行わない。
 */
bool b_yaw_pi_step(BYawPiState *state, const BYawPiGains *gains,
                   const BYawPiInput *input, BYawPiOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_YAW_PI_H */
