#ifndef BALANCING_CORE_H
#define BALANCING_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 車輪左右のトルク配分結果。単位は全て[N·m]。 */
typedef struct {
    float left_nm;
    float right_nm;
    float base_each_nm;
    float yaw_applied_nm;
    bool saturated;
} BWheelAllocation;

/**
 * @brief 倒立用の左右共通トルクを優先し残余でYaw差動を割り当てる。
 * @spec{CTRL-LQR-001}
 * @param[in] total_balance_nm 左右合計の倒立要求トルク [N·m]。
 * @param[in] yaw_delta_nm 右車輪に加える差動トルク [N·m]（左は減算）。
 * @param[in] left_limit_nm 左車輪の正の絶対トルク上限 [N·m]。
 * @param[in] right_limit_nm 右車輪の正の絶対トルク上限 [N·m]。
 * @param[out] result 配分結果。失敗時は0出力。
 * @return 全入力が有限で上限が正ならtrue。飽和は失敗を意味しない。
 * @warning トルク飽和のみでFAULTとしない。
 */
bool b_wheel_allocate(float total_balance_nm, float yaw_delta_nm,
                      float left_limit_nm, float right_limit_nm,
                      BWheelAllocation *result);

/** @brief LQRの状態ベクトル順序。行列添字には必ずこの定数を使う。 */
typedef enum {
    B_STATE_THETA = 0,       /**< 仮想脚傾斜 [rad]。 */
    B_STATE_THETA_RATE,      /**< 仮想脚傾斜速度 [rad/s]。 */
    B_STATE_POSITION,        /**< 前後位置 [m]。 */
    B_STATE_VELOCITY,        /**< 前後速度 [m/s]。 */
    B_STATE_PITCH,           /**< 機体Pitch [rad]。 */
    B_STATE_PITCH_RATE,      /**< 機体Pitch角速度 [rad/s]。 */
    B_STATE_COUNT
} BStateIndex;

/** @brief LQR出力順序。行列の行と平衡トルクの要素に使用。 */
typedef enum {
    B_OUTPUT_WHEEL_TOTAL = 0, /**< 左右車輪合計 [N·m]。 */
    B_OUTPUT_HIP_TOTAL,       /**< 左右仮想股関節合計 [N·m]。 */
    B_OUTPUT_COUNT
} BOutputIndex;

/** @brief 推定器から制御器へ渡す6状態。すべてSI単位。 */
typedef struct {
    float theta_rad;
    float theta_rate_rad_s;
    float position_m;
    float velocity_m_s;
    float pitch_rad;
    float pitch_rate_rad_s;
} BControlState;

/** @brief 制御器から配分器へ渡す抽象トルク。実機CAN指令ではない。 */
typedef struct {
    float wheel_total_nm;
    float hip_total_nm;
} BControlTorque;

/** @brief 外部で算出・検証した6状態2出力の離散LQR係数。 */
typedef struct {
    float gain[B_OUTPUT_COUNT][B_STATE_COUNT];
    float equilibrium[B_OUTPUT_COUNT];
    float max_position_error_m;
    bool verified; /**< 上位設定の実機検証状態。計算からは検証できない。 */
} BLqrConfig;

/**
 * @brief 6状態LQRの抽象トルクを計算する（実機への指令は行わない）。
 * @spec{CTRL-LQR-001}
 * @param[in] config 検証済みの300 mmモデルのゲインと平衡トルク。
 * @param[in] state 名前と単位を持つ推定済み6状態。
 * @param[in] target 名前と単位を持つ目標6状態。
 * @param[out] torque 2出力の抽象トルク。失敗時はゼロ。
 * @return 入力と検証フラグが有効な場合だけtrue。失敗時は0出力。
 * @warning 値の有限性チェックは物理ゲインの実機妥当性を保証しない。
 */
bool b_lqr_compute(const BLqrConfig *config, const BControlState *state,
                   const BControlState *target, BControlTorque *torque);

#ifdef __cplusplus
}
#endif

#endif /* BALANCING_CORE_H */
