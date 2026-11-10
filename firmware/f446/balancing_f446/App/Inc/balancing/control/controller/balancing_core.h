#ifndef BALANCING_CORE_H
#define BALANCING_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ロボットの外部公開状態。内部のARM待機は独立した状態としない。 */
typedef enum {
    B_MODE_DISARMED = 0,
    B_MODE_BALANCING = 1,
    B_MODE_FAULT = 2
} BMode;

/** @brief FAULTに遷移した主原因。数値はCAN通信に使用しない。 */
typedef enum {
    B_FAULT_NONE = 0,
    B_FAULT_ESTOP,
    B_FAULT_EXTERNAL,
    B_FAULT_GUARD,
    B_FAULT_SCHEDULER /**< 500 Hz制御更新が6 ms以上停止、または時計が逆行。 */
} BFaultCause;

/** @brief モノトニック時刻付きの観測値。時刻単位はマイクロ秒。 */
typedef struct {
    bool valid;
    uint64_t stamp_us;
} BSampleStamp;

/** @brief 通信層が収集した最後の「新規・有効」測定の時刻。 */
typedef struct {
    BSampleStamp imu;
    BSampleStamp rs[4];
    BSampleStamp wheel[2];
    BSampleStamp power;
} BObservedStamps;

/** @brief センサの有効性・更新鮮度。集約状態はb_freshness_allで求める。 */
typedef struct {
    bool imu;
    bool rs[4];
    bool wheel[2];
    bool power;
} BFreshness;

/**
 * @brief センサの受信鮮度を判定する。時刻が未来なら無効。
 * @spec{BOARD-F446-001}
 * @spec{CTRL-EST-001}
 * @param[in] stamps それぞれの最後の新規・有効測定の時刻。
 * @param[in] now_us 同じ単調増加クロックの現在時刻 [us]。
 * @return IMU 50 ms、RS06各軸30 ms、車輪各軸20 ms、電源500 msの判定。
 * @warning 受信を繰り返しただけの古い測定に新しい時刻を与えないこと。
 */
BFreshness b_check_freshness(const BObservedStamps *stamps, uint64_t now_us);

/**
 * @brief 個別の測定鮮度から全体の有効性を導出する。
 * @spec{BOARD-F446-001}
 * @param[in] freshness 受信系ごとの鮮度判定。
 * @return すべての測定が有効であればtrue。
 * @note 集約フラグを構造体に重複保持しない。
 */
bool b_freshness_all(BFreshness freshness);

/** @brief 状態機械が保持する内部状態。 */
typedef struct {
    BMode mode;
    BFaultCause fault_cause;
    bool arm_pending;
    uint64_t enable_request_us;
    bool has_last_step; /**< 前回の制御更新時刻が有効か。 */
    uint64_t last_step_us; /**< 同一単調増加時計の前回時刻 [us]。 */
} BModeMachine;

/** @brief 外部のアダプタが判定した安全条件と操作イベント。 */
typedef struct {
    BFreshness fresh;
    bool power_on;
    bool calibration_verified;
    bool model_verified;
    bool limits_verified;
    bool posture_ok;
    bool external_fault;
    bool estop_active;
    bool arm_edge;
    bool disarm_edge;
    bool clear_fault_edge;
    bool clear_authorized;
    bool rs_operational[4];
    BSampleStamp rs_enable_sent[4]; /**< 各Type3指令を実際に送信した時刻。 */
    BSampleStamp rs_reply[4]; /**< それぞれの有効化指令後の新規状態応答。 */
} BModeInput;

/** @brief 通信アダプタへ渡す一周期分の要求。CANの送信形式は含まない。 */
typedef struct {
    BMode mode;
    BFaultCause fault_cause;
    bool drive_allowed;
    bool request_rs_enable;
    bool request_rs_disable;
    bool request_power_off;
    bool arm_rejected;
} BModeOutput;

/**
 * @brief 状態機械をDISARMEDで初期化する。
 * @spec{BEHAV-001}
 * @param[out] machine 初期化対象。
 */
void b_mode_init(BModeMachine *machine);

/**
 * @brief 状態遷移とARM許可を1回評価する。実機入出力は行わない。
 * @spec{BEHAV-001}
 * @spec{BOARD-F446-001}
 * @pre 操作イベントは立上りで入力する。外部故障と非常停止は独立入力とする。
 * @param[in,out] machine 状態機械。ARM待機はDISARMED中にのみ保持する。
 * @param[in] input 電源・姿勢・検証済み値・最新の通信状態。
 * @param[in] now_us 単調増加時刻 [us]。
 * @return 駆動許可と通信アダプタへの要求。
 * @note RS06への各Type3送信完了時刻より後の正常応答を4台分確認するまでBALANCINGにしない。
 * @pre rs_enable_sentは送信を試みた時刻ではなく、送信元が送信を受理した実時刻を記録する。
 * @warning FAULT解除の許可は呼出し元で個別に検証する。自動復帰はしない。
 */
BModeOutput b_mode_step(BModeMachine *machine, const BModeInput *input,
                        uint64_t now_us);

/**
 * @brief 目標値に変化率制限を適用する。
 * @spec{CTRL-LQR-001}
 * @param[in] previous 前周期の目標値。
 * @param[in] request 操作元が要求した目標値。
 * @param[in] max_rate 正の変化率上限 [対象単位/s]。
 * @param[in] dt_s 演算間隔 [s]。
 * @param[out] result 制限後の値。失敗時は0を格納。
 * @return 計算が有効ならtrue。
 */
bool b_rate_limit(float previous, float request, float max_rate,
                  float dt_s, float *result);

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
