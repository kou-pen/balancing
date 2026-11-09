/**
 * @file documentation_example.h
 * @brief DoxygenのAPI記述例（実機用制御コードではない）。
 *
 * 公開APIのコメント書式だけを示す。実際の倒立制御アルゴリズムや
 * 電流指令を含まず、このまま実機制御へ組み込まないこと。
 */
#ifndef BALANCING_DOCUMENTATION_EXAMPLE_H
#define BALANCING_DOCUMENTATION_EXAMPLE_H

/** @brief 状態推定値の構造体例。実際のファームウェアの構造体ではない。 */
typedef struct {
    float theta_rad;   /**< 世界座標における仮想脚傾斜角 [rad]。 */
    float pitch_rad;   /**< 機体のピッチ角 [rad]。 */
    float wheel_mps;   /**< 車輪の移動速度 [m/s]。 */
} BalanceDocExampleState;

/** @brief 抽象トルクの出力例。CAN電流指令そのものではない。 */
typedef struct {
    float wheel_total_nm; /**< 左右車輪の合計トルク [N·m]。 */
    float hip_total_nm;   /**< 左右仮想股関節の合計トルク [N·m]。 */
} BalanceDocExampleTorques;

/**
 * @brief 検証済みの推定状態から抽象トルクを計算する関数の宣言例。
 *
 * @spec{CTRL-LQR-001}
 * @spec{CTRL-EST-001}
 * @spec{BOARD-F446-001}
 *
 * @pre K_300、座標系、個別保護限界が検証済みであること。
 * @pre 入力が新しく有効で、倒立の許可条件が成立していること。
 * @param[in] state この制御周期における推定状態。
 * @param[out] torques 車輪・仮想股関節トルクの出力先。単位はN·m。
 * @return 出力が有効なら0、計算結果を採用できなければ0以外。
 *
 * @note 書式確認用の例であり、関数実装は存在しない。
 * @note このAPI内ではCANフレームのバイト配列を生成しない。
 * @warning 単純なトルク飽和のみでFAULTとしない。
 */
int balancing_example_lqr_step(
    const BalanceDocExampleState *state,
    BalanceDocExampleTorques *torques
);

#endif /* BALANCING_DOCUMENTATION_EXAMPLE_H */
