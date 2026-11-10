#ifndef BALANCING_LEG_KINEMATICS_H
#define BALANCING_LEG_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 2入力の平面閉リンクに使う数学テスト専用の幾何パラメータ。
 * @warning このリンク構成は実機の六節リンクCADと未照合。
 *          実機への採用前に節の接続、取付点、組立分岐を再定義する。
 */
typedef struct {
    float base_span_m;          /**< 2つの固定モータ軸の間隔 [m]。 */
    float left_crank_m;         /**< 左モータ軸から左肘節までの長さ [m]。 */
    float right_crank_m;        /**< 右モータ軸から右肘節までの長さ [m]。 */
    float left_rod_m;           /**< 左肘節から仮想足先までの長さ [m]。 */
    float right_rod_m;          /**< 右肘節から仮想足先までの長さ [m]。 */
    float min_link_cross_sine;  /**< 先端2本の角度のsin絶対値の下限 (0,1)。 */
    float min_leg_length_m;     /**< 原点近傍を特異として拒否する脚長下限 [m]。 */
} BLegFixtureGeometry;

/** @brief 2つの円の交点を選ぶ組立分岐。自動で切り替えない。 */
typedef enum {
    B_LEG_BRANCH_NEGATIVE = -1, /**< 左肘→右肘方向の右側。 */
    B_LEG_BRANCH_POSITIVE = 1   /**< 左肘→右肘方向の左側。 */
} BLegAssemblyBranch;

/** @brief 計算成功・不正入力・非到達・特異を区別する。 */
typedef enum {
    B_LEG_OK = 0,
    B_LEG_INVALID_INPUT,
    B_LEG_UNREACHABLE,
    B_LEG_SINGULAR
} BLegStatus;

/**
 * @brief モータ軸→機構入力軸の外部伝達比、原点、方向を分離して保持する。
 * @warning calibration_valid=trueの数学fixtureを実機校正完了と混同しない。
 */
typedef struct {
    float motor_zero_rad[2];   /**< 機構角0に対応するモータ軸の測定角 [rad]。 */
    float joint_offset_rad[2]; /**< 機構原点からの明示的なオフセット [rad]。 */
    float sign[2];             /**< それぞれ+1または-1のみ。 */
    float driving_teeth;       /**< 外部伝達のモータ側歯数。 */
    float driven_teeth;        /**< 外部伝達の機構側歯数。 */
    bool calibration_valid;    /**< 上位が実校正済み、または明示的fixtureと判断。 */
} BLegMotorMap;

/** @brief 2つの機構入力軸の角度と角速度（モータ測定値ではない）。 */
typedef struct {
    float q_rad[2];          /**< 左右それぞれの機構軸角度 [rad]。 */
    float qdot_rad_s[2];     /**< 左右それぞれの機構軸角速度 [rad/s]。 */
} BLegJointState;

/** @brief 機構入力角から仮想脚座標への解析ヤコビアン。 */
typedef struct {
    float dlength_dq_m[2]; /**< 脚長の機構角微分 [m/rad]。 */
    float dtheta_dq[2];    /**< 仮想脚角の機構角微分 [rad/rad]。 */
} BLegJacobian;

/** @brief 左右固定軸の中点を原点とする仮想脚状態。 */
typedef struct {
    float foot_x_m;           /**< 仮想足先の平面内水平位置 [m]。 */
    float foot_y_m;           /**< 仮想足先の平面内上向き位置 [m]。 */
    float length_m;           /**< 固定軸中点から仮想足先までの距離 [m]。 */
    float theta_rad;          /**< atan2(x,-y)で定める前傾正の角度 [rad]。 */
    float length_rate_m_s;    /**< 解析ヤコビアンからの脚長速度 [m/s]。 */
    float theta_rate_rad_s;   /**< 解析ヤコビアンからの仮想脚角速度 [rad/s]。 */
    BLegJacobian jacobian;
} BLegKinematics;

/** @brief 仮想脚の一般化力。 */
typedef struct {
    float axial_force_n;    /**< 脚長の増加方向を正とする力 [N]。 */
    float pitch_torque_nm;  /**< 仮想脚角の増加方向を正とするトルク [N·m]。 */
} BLegVirtualLoad;

/** @brief 左右の機構軸トルク、または理想伝達条件でのモータ軸トルク。 */
typedef struct {
    float torque_nm[2]; /**< それぞれの軸のトルク [N·m]。 */
} BLegTorquePair;

/**
 * @brief 既知の外部歯数比、原点、軸方向からモータ測定角を機構入力角に変換。
 * @spec{CTRL-EST-001}
 * @param[in] map 明示的に検証状態を与えた原点と外部伝達比。
 * @param[in] motor_angle_rad モータ軸の測定角度 [rad]。
 * @param[in] motor_rate_rad_s モータ軸の測定角速度 [rad/s]。
 * @param[out] joint 機構側の角度・速度。失敗時はゼロ。
 * @return 入力・校正情報が有限かつ有効ならtrue。
 * @warning test fixtureで成功しても実機エンコーダ原点を保証しない。
 */
bool b_leg_motor_to_joint(const BLegMotorMap *map, const float motor_angle_rad[2],
                          const float motor_rate_rad_s[2], BLegJointState *joint);

/**
 * @brief 2入力の閉リンク数学fixtureを解き、脚長と仮想角の解析ヤコビアンを計算。
 * @spec{CTRL-EST-001}
 * @param[in] geometry 有効なリンク長と特異姿勢の判定条件。
 * @param[in] branch 組立分岐。未指定分岐へ自動遷移しない。
 * @param[in] joint 機構側の入力角・角速度（SI単位）。
 * @param[out] out 脚長、仮想脚角、速度、ヤコビアン。失敗時ゼロ。
 * @return 成功、不正入力、到達不能、特異姿勢のいずれか。
 * @warning **実機六節リンクの幾何を実装・検証したものではない。**
 */
BLegStatus b_leg_fixture_forward(const BLegFixtureGeometry *geometry,
                                 BLegAssemblyBranch branch,
                                 const BLegJointState *joint,
                                 BLegKinematics *out);

/**
 * @brief 仮想仕事 J^T による脚長方向力・仮想脚トルクから機構軸トルクへの変換。
 * @spec{CTRL-EST-001}
 * @param[in] state 正常計算した脚のヤコビアン。
 * @param[in] load 軸力 [N]と仮想脚トルク [N·m]。
 * @param[out] torque 機構軸のトルク [N·m]。失敗時ゼロ。
 * @return 全入力と計算結果が有限ならtrue。
 * @warning 摩擦、ガスシリンダ、実機トルク上限は含めない。
 */
bool b_leg_virtual_to_joint_torque(const BLegKinematics *state,
                                   const BLegVirtualLoad *load,
                                   BLegTorquePair *torque);

/**
 * @brief 理想的な外部伝達に限り、機構軸トルクからモータ軸トルクへ変換。
 * @spec{CTRL-EST-001}
 * @param[in] map 校正条件と外部減速比。
 * @param[in] joint_torque 機構軸トルク [N·m]。
 * @param[out] motor_torque モータ軸トルク [N·m]。失敗時ゼロ。
 * @return 入力・変換結果が有効ならtrue。
 * @warning 効率・バックラッシュ・電流変換を含まず、実機駆動用ではない。
 */
bool b_leg_joint_to_ideal_motor_torque(const BLegMotorMap *map,
                                       const BLegTorquePair *joint_torque,
                                       BLegTorquePair *motor_torque);

#ifdef __cplusplus
}
#endif
#endif /* BALANCING_LEG_KINEMATICS_H */
