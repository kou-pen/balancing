#pragma once

#include "balancing/control/estimator/StateEstimator.hpp"

#include <array>
#include <optional>

namespace balancing {

/** @brief 車輪左右のトルク配分結果。単位は全て[N·m]。 */
struct WheelAllocation {
    float left_nm{};
    float right_nm{};
    float base_each_nm{};
    float yaw_applied_nm{};
    bool saturated{};
};

/** @brief LQRの状態ベクトル順序。行列添字には必ずこの定数を使う。 */
enum LqrStateIndex : std::size_t {
    LqrTheta = 0,       /**< 仮想脚傾斜 [rad]。 */
    LqrThetaRate,      /**< 仮想脚傾斜速度 [rad/s]。 */
    LqrPosition,        /**< 前後位置 [m]。 */
    LqrVelocity,        /**< 前後速度 [m/s]。 */
    LqrPitch,           /**< 機体Pitch [rad]。 */
    LqrPitchRate,      /**< 機体Pitch角速度 [rad/s]。 */
    LqrStateCount
};

/** @brief LQR出力順序。行列の行と平衡トルクの要素に使用。 */
enum LqrOutputIndex : std::size_t {
    LqrWheelTotal = 0, /**< 左右車輪合計 [N·m]。 */
    LqrHipTotal,       /**< 左右仮想股関節合計 [N·m]。 */
    LqrOutputCount
};

/** @brief 推定器から制御器へ渡す6状態。すべてSI単位。 */
struct LqrState {
    float theta_rad{};
    float theta_rate_rad_s{};
    float position_m{};
    float velocity_m_s{};
    float pitch_rad{};
    float pitch_rate_rad_s{};
};

/** @brief 制御器から配分器へ渡す抽象トルク。実機CAN指令ではない。 */
struct LqrTorque {
    float wheel_total_nm{};
    float hip_total_nm{};
};

/** @brief 外部で算出・検証した6状態2出力の離散LQR係数。 */
struct LqrConfig {
    std::array<std::array<float,LqrStateCount>,LqrOutputCount> gain{};
    std::array<float,LqrOutputCount> equilibrium{};
    float max_position_error_m{};
    bool verified{}; /**< 上位設定の実機検証状態。計算からは検証できない。 */
};

/** @brief 左右脚の測定長・伸縮速度。位置[m]、速度[m/s]。 */
struct LegPdFeedback {
    float left_length_m{};
    float right_length_m{};
    float left_velocity_m_s{};
    float right_velocity_m_s{};
    float roll_rad{};
    float roll_rate_rad_s{};
};

/** @brief ARM直前に測定した固定脚長目標。装置固有の可動限界は含まない。 */
struct LegPdReference {
    float common_length_m{}; /**< (左+右)/2 [m]。 */
    float half_difference_m{}; /**< (左-右)/2 [m]。 */
    bool valid{}; /**< 新鮮な観測からDISARMED中に取得できた場合のみtrue。 */
};

/** @brief 物理検証が終わるまで無効とする脚長・Rollゲイン。 */
struct LegPdGains {
    float length_kp_n_m{}; /**< 支持力 / 脚長偏差 [N/m]。 */
    float length_kd_n_s_m{}; /**< 支持力 / 伸縮速度 [N s/m]。 */
    float roll_kp_n_rad{}; /**< 左右差動支持力 / Roll偏差 [N/rad]。 */
    float roll_kd_n_s_rad{}; /**< 左右差動支持力 / Roll角速度 [N s/rad]。 */
    float gravity_feedforward_n{}; /**< 片脚の重力補償支持力 [N]。 */
    bool verified{}; /**< 外部で物理モデル・符号を検証した場合にだけtrue。 */
};

/** @brief VMCへ渡す脚別の抽象支持力。モータ指令ではない。 */
struct LegPdOutput {
    float left_force_n{};
    float right_force_n{};
    float roll_differential_n{};
};

/** @brief Yaw角速度PIゲイン。実機で確認するまではverified=false。 */
struct YawPiGains {
    float kp_nm_s_rad{}; /**< P係数 [N m / (rad/s)]。 */
    float ki_nm_rad{}; /**< 積分係数 [N m / rad]。 */
    float integral_limit_nm{}; /**< 積分成分の絶対値上限 [N m]。 */
    bool verified{}; /**< 外部で実機妥当性を承認した場合のみtrue。 */
};

/** @brief 単一所有者が維持する積分器。ISRで同時更新しない。 */
struct YawPiState {
    float integral_nm{}; /**< トルク単位の積分成分。 */
    bool active{}; /**< 前回演算がBALANCING中だったか。 */
};

/** @brief 一周期の測定と、倒立優先配分後に残ったYaw容量。 */
struct YawPiInput {
    float target_rate_rad_s{}; /**< 操作層が加減速制限したYaw角速度目標。 */
    float measured_rate_rad_s{}; /**< ジャイロから推定したYaw角速度。 */
    float dt_s{}; /**< 実際の制御演算間隔 [s]。 */
    float available_delta_nm{}; /**< 車輪倒立成分確保後の左右差動余力 [N m]。 */
    bool balancing{}; /**< 実制御状態がBALANCINGの場合のみtrue。 */
};

/** @brief 車輪配分器へ渡す左右逆符号の差動トルク要求。 */
struct YawPiOutput {
    float delta_nm{}; /**< 右へ+、左へ-として与える差動 [N m]。 */
    float integral_nm{}; /**< 採用された積分成分 [N m]。 */
    bool saturated{}; /**< 出力容量による差動制限が発生した場合。 */
    bool integral_frozen{}; /**< 飽和悪化を防ぐため積分を停止した場合。 */
};

/** @brief 制御器内の数値演算。物理出力・状態遷移は持たない。 */
namespace balance_math {
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
bool allocateWheels(float total_balance_nm, float yaw_delta_nm,
                      float left_limit_nm, float right_limit_nm,
                      WheelAllocation& result) noexcept;
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
bool computeLqr(const LqrConfig& config, const LqrState& state,
                   const LqrState& target, LqrTorque& torque) noexcept;
/**
 * @brief DISARMED中の新鮮な脚長から左右を保つ固定脚長目標を取得する。
 * @spec{CTRL-LQR-001}
 * @param[out] reference 保存先。失敗時は初期化して無効化する。
 * @param[in] feedback 同一周期で固定した左右の測定値。
 * @param[in] disarmed 真なら操縦系がDISARMEDにあることを示す。
 * @pre 呼出し側が左右RS06の鮮度を安全状態機械で評価済みであること。
 *      この関数では新鮮さを重複判定せず、渡された脚長を数学的に検査する。
 * @return 有限・正の脚長を持ち、安全ゲートを満たした場合だけtrue。
 * @warning 目標を取得しただけでARM許可を出さない。実機可動域も検証しない。
 */
bool captureLegReference(LegPdReference& reference, const LegPdFeedback& feedback,
                      bool disarmed) noexcept;
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
bool computeLegSupport(const LegPdReference& reference, const LegPdGains& gains,
                      const LegPdFeedback& feedback, LegPdOutput& output) noexcept;
/**
 * @brief 積分器をゼロにし、次のARMセッション開始を準備する。
 * @spec{CTRL-LQR-001}
 * @param[in,out] state 積分状態。
 * @warning ARM開始・モード変更・FAULTで呼ぶこと。通常の非BALANCING
 *          ステップでも自動リセットするが、呼出し省略時に備えて明示する。
 */
void resetYaw(YawPiState& state) noexcept;
/**
 * @brief 角速度追従Yaw PIを条件付き積分・出力制限付きで1回更新する。
 * @spec{CTRL-LQR-001}
 * @param[in,out] state 前回までの積分状態。失敗時にはリセットする。
 * @param[in] gains 外部検証済みの非負ゲインと積分上限。
 * @param[in] input 一周期の目標・観測・時間間隔・差動余力・動作状態。
 * @param[out] output 差動要求と飽和・積分停止フラグ。失敗時ゼロ。
 * @return 入力が有効な場合true。BALANCING外ではtrueでゼロを返す。
 * @pre 呼出し元の安全状態機械がセンサ鮮度と通信断を判断済みであること。
 *      本関数ではジャイロ鮮度を重複して監視しない。
 * @warning available_delta_nmはallocateWheelsと同じ倒立優先の
 *          左右容量から呼出し元が求める。ここではCAN/ARM操作を行わない。
 */
bool stepYaw(YawPiState& state, const YawPiGains& gains,
                   const YawPiInput& input, YawPiOutput& output) noexcept;
} // namespace balance_math

/** @brief 今周期の6状態LQR・Yaw PIが追従する数学目標。
 *  @note 位置基準の生成・加減速制限は未実装の操作側の責務。
 *        ここに未合意の位置追従規則を暗黙に追加しない。
 *  @spec{CTRL-LQR-001}
 */
struct BalanceReference {
    float positionM{0.0f};
    float forwardVelocityMps{0.0f};
    float yawRateRadS{0.0f};
};

struct BalanceControlConfig {
    LqrConfig lqr{};
    LegPdGains legPd{};
    YawPiGains yawPi{};
    std::array<float, 2> wheelTorqueLimitNm{};
};

/** @brief 機構側トルクの純粋な数学計算結果でありCAN指令ではない。
 *  @warning 脚のCAD、摩擦、ガスシリンダ、変速比、電流換算、実機限界は未結合。
 *           fixtureOnlyの結果をモータへ配送するAPIは存在しない。
 *  @spec{CTRL-LQR-001}
 */
struct BalanceControlOutput {
    std::array<float, 4> jointTorqueNm{}; // 左A/B、右A/B（機構軸）
    std::array<float, 2> wheelTorqueNm{}; // 左、右（車輪機構軸）
    float wheelBalanceTotalNm{0.0f};
    float yawAppliedNm{0.0f};
    bool wheelSaturated{false};
    bool yawIntegralFrozen{false};
    bool fixtureOnly{true};
};

/** @brief 検証済みのLQR/脚PD/Roll PD/VMC/Yaw PIを順に組み合わせる。
 *  @note 制御目標の生成や実トルク上限の決定は行わない。
 *  @warning SIL数値比較のみ。MotorCommand、MotorService、CAN/HALへ接続しない。
 *  @spec{CTRL-LQR-001}
 */
class BalanceControl final {
public:
    /** @brief 実機値と別に、数学fixtureのゲインを持つ計算器を組み立てる。
     *  @param[in] config LQRとPD/PI係数、車輪の数学上限。
     *  @spec{CTRL-LQR-001}
     */
    explicit BalanceControl(const BalanceControlConfig& config) noexcept;

    /** @brief ARM前の左右脚長を固定目標として保持し、Yaw積分を初期化する。
     *  @param[in] snapshot すべての観測が成立した、DISARMED中の周期コピー。
     *  @return 有効な脚長ならtrue。前の初期化結果は再利用しない。
     *  @spec{CTRL-LQR-001}
     */
    bool initialize(const ControlSnapshot& snapshot) noexcept;

    /** @brief C++数値演算で今周期の機構軸トルクを計算する。
     *  @param[in] snapshot 状態推定済みの周期コピー。
     *  @param[in] reference 速度、Yaw、および上流で明示した位置基準。
     *  @return 数学計算がすべて成立したときだけ値を返す。
     *  @note 失敗後の古い計算結果は再利用しない。CANへ配送しない。
     *  @spec{CTRL-LQR-001}
     */
    std::optional<BalanceControlOutput> compute(
        const ControlSnapshot& snapshot, const BalanceReference& reference) noexcept;

    /** @brief 非BALANCING・FAULTまたはARMセッション変更時に履歴を消す。
     *  @spec{CTRL-LQR-001}
     */
    void reset() noexcept;

private:
    BalanceControlConfig config_;
    LegPdReference legReference_{};
    YawPiState yawState_{};
};

} // namespace balancing
