#ifndef BALANCING_OPERATOR_INPUT_H
#define BALANCING_OPERATOR_INPUT_H

#include "balancing/control/controller/balancing_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 2つの操作経路。CANの数値IDやUSB HIDボタン番号は含めない。 */
typedef enum {
    B_OPERATOR_USB_DUALSENSE = 0,
    B_OPERATOR_CAN_REMOTE = 1
} BOperatorSource;

/**
 * @brief モック入力処理の外部検証済み設定。
 * @spec{BEHAV-001}
 * @spec{PROTO-REMOTE-001}
 * @warning デッドゾーン・通信周期・速度上限は実機未確定である。
 *          ホスト試験では人工的な数値を明示的に注入する。
 */
typedef struct {
    uint64_t input_timeout_us; /**< 最後の新規入力から有効とする期限 [us]。 */
    uint64_t neutral_hold_us; /**< 復旧時ニュートラル継続確認に必要な時間 [us]。 */
    float max_speed_m_s; /**< デコード済み前後速度要求の絶対値上限 [m/s]。 */
    float max_yaw_rate_rad_s; /**< デコード済みYaw角速度要求上限 [rad/s]。 */
    float neutral_speed_m_s; /**< 速度をニュートラルと扱う幅 [m/s]。 */
    float neutral_yaw_rad_s; /**< Yawをニュートラルと扱う幅 [rad/s]。 */
    float speed_slew_m_s2; /**< 速度目標の変化率上限 [m/s^2]。 */
    float yaw_slew_rad_s2; /**< Yaw目標の変化率上限 [rad/s^2]。 */
    bool verified; /**< 実機設定の検証、または明示した人工fixtureの確認。 */
} BOperatorInputConfig;

/** @brief 最後に新規と確認されたデコード済みの入力。 */
typedef struct {
    bool valid;
    uint64_t stamp_us; /**< 上位入力の実測定・受信時刻 [us]。再読出し時刻ではない。 */
    float speed_m_s;
    float yaw_rate_rad_s;
} BOperatorSample;

/** @brief 単一所有者が更新する操作入力状態。ハードウェア受信処理を持たない。 */
typedef struct {
    BOperatorInputConfig config;
    BOperatorSource selected;
    BOperatorSample last_sample;
    bool waiting_neutral;
    bool neutral_start_valid;
    uint64_t neutral_start_us;
    unsigned neutral_sample_count;
    bool tick_valid;
    uint64_t last_tick_us;
    float target_speed_m_s;
    float target_yaw_rate_rad_s;
    bool initialized;
} BOperatorInput;

/** @brief 500 Hzティックが返す操作目標。ARMを要求するフィールドはない。 */
typedef struct {
    BOperatorSource source;
    bool source_fresh;
    bool operator_ready;
    bool waiting_neutral;
    float speed_m_s;
    float yaw_rate_rad_s;
} BOperatorTarget;

/**
 * @brief 入力処理を初期化し、標準入力元USB_DUALSENSEを選択する。
 * @spec{BEHAV-001}
 * @param[out] input 書込み所有者が保持する状態。失敗時は無効状態へゼロ化。
 * @param[in] config 検証済み/人工fixtureとして与えるSI値・時刻の設定。
 * @return 設定が有効な場合のみtrue。
 * @warning 本関数は実HID/CAN通信にもモータARMにも接続しない。
 */
bool b_operator_init(BOperatorInput *input, const BOperatorInputConfig *config);

/**
 * @brief 操作元をDISARMED中に限って明示的に変更する。
 * @spec{PROTO-REMOTE-001}
 * @param[in,out] input 操作入力状態。
 * @param[in] source USBまたはCANの論理ソース。
 * @param[in] mode 呼出元の実状態機械のモード。
 * @return DISARMEDかつ既知の入力元ならtrue。異なる操作元へ切替時は
 *         古い入力・目標を破棄し、再ニュートラル確認を要求する。
 * @warning 通信断を理由に勝手に別ソースへ切り替えない。
 */
bool b_operator_select(BOperatorInput *input, BOperatorSource source, BMode mode);

/**
 * @brief 通信層が新規性を確認したデコード済みの操作量を受け取る。
 * @spec{BEHAV-001}
 * @param[in,out] input 操作入力状態。単一所有者から呼ぶ。
 * @param[in] source 受信データの操作元。
 * @param[in] speed_m_s 前後速度の指令値 [m/s]。
 * @param[in] yaw_rate_rad_s Yaw角速度の指令値 [rad/s]。
 * @param[in] new_sample_proven 真の新規入力のみtrue。再読出しではfalse。
 * @param[in] stamp_us 新規入力の時刻 [us]。
 * @param[in] now_us 同一単調時計における受付時刻 [us]。
 * @return 正当な新規入力を受理した場合だけtrue。
 * @warning HIDレポート番号・CAN IDはここで仮定しない。
 *          ARM/CLEAR_FAULTのイベントは生成・受理しない。
 */
bool b_operator_accept(BOperatorInput *input, BOperatorSource source,
                       float speed_m_s, float yaw_rate_rad_s,
                       bool new_sample_proven, uint64_t stamp_us, uint64_t now_us);

/**
 * @brief 2 ms制御ティック用に操作目標を更新する。
 * @spec{BEHAV-001}
 * @param[in,out] input 操作入力状態。
 * @param[in] mode 実状態機械から受け取ったモード。ARM要求は出力しない。
 * @param[in] now_us 単調増加時計 [us]。
 * @param[out] out 操作目標。拒否時はゼロ化。
 * @return 入力・時刻が有効ならtrue。入力喪失は正常処理としてゼロへ減速。
 * @warning DISARMED/FAULT中は目標を即ゼロにするだけ。
 *          実モータの制御・停止を保証するものではない。
 */
bool b_operator_tick(BOperatorInput *input, BMode mode,
                     uint64_t now_us, BOperatorTarget *out);

#ifdef __cplusplus
}
#endif
#endif /* BALANCING_OPERATOR_INPUT_H */
