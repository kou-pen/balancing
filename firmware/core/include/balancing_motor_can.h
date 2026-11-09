#ifndef BALANCING_MOTOR_CAN_H
#define BALANCING_MOTOR_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CANフレームのバスとID種別を明示した転送データ。 */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    bool extended;
} BMotorCanFrame;

/** @brief RS06 Type 2/24で報告された物理量。実機のFW版により換算範囲の再確認が必要。 */
typedef struct {
    uint8_t motor_id;
    uint8_t host_id;
    uint8_t mode;
    uint8_t fault_bits;
    float angle_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} BRS06Feedback;

/** @brief C620の車輪フィードバック。角度はモータロータの1回転内の値。 */
typedef struct {
    uint8_t motor_id;
    uint16_t encoder_raw;
    int16_t rpm;
    int16_t current_raw;
    uint8_t temperature_c;
} BC620Feedback;

/**
 * @brief RS06のType 3有効化またはType 4通常停止フレームを構築する。
 * @spec{PROTO-RS06-001}
 * @param[in] motor_id 実機で設定された宛先ID（1〜255）。
 * @param[in] host_id 実機で確認した主機ID（0〜255）。
 * @param[in] enable true:Type3有効化、false:Type4通常停止（故障解除は行わない）。
 * @param[out] out 出力CAN1拡張フレーム。
 * @return 有効な引数ならtrue。
 */
bool b_rs06_control_frame(uint8_t motor_id, uint8_t host_id,
                          bool enable, BMotorCanFrame *out);

/**
 * @brief RS06 Type 1の純トルクフィードフォワード指令を構築する。
 * @spec{PROTO-RS06-001}
 * @param[in] motor_id 宛先ID。
 * @param[in] torque_nm モータ側の要求トルク[N·m]。上位で安全上限に制限済みであること。
 * @param[out] out 出力フレーム。
 * @return 有効な引数・メーカー指定の符号化範囲ならtrue。
 * @warning この関数は駆動許可や安全上限を判断しない。呼出し側で行うこと。
 */
bool b_rs06_torque_frame(uint8_t motor_id, float torque_nm, BMotorCanFrame *out);

/**
 * @brief RS06のType 2/24フィードバックを復号する。
 * @spec{PROTO-RS06-001}
 * @param[in] frame 受信したCAN1フレーム。
 * @param[out] out モータID、FAULT、運転モード、角度・速度・トルク・温度。
 * @return 妥当な形式ならtrue。故障状態自体はtrueを返しfault_bitsへ記録する。
 */
bool b_rs06_parse_feedback(const BMotorCanFrame *frame, BRS06Feedback *out);

/**
 * @brief C620のID 1〜4へ共通で送る0x200電流指令を構築する。
 * @spec{PROTO-C620-001}
 * @param[in] current_a ID1〜ID4の要求電流[A]。未接続モータは0とする。
 * @param[in] max_abs_current_a 実機で別途設定した安全な絶対電流上限[A]。
 * @param[out] out 出力CAN2標準フレーム。
 * @return 入力が有効で上限内ならtrue。上限を超える指令は拒否する。
 * @warning プロトコル表現範囲20Aは実機の安全上限ではない。
 */
bool b_c620_current_frame(const float current_a[4], float max_abs_current_a,
                          BMotorCanFrame *out);

/**
 * @brief C620の0x201〜0x204フィードバックを復号する。
 * @spec{PROTO-C620-001}
 * @param[in] frame 受信したCAN2標準フレーム。
 * @param[out] out ロータエンコーダ、RPM、電流生値、温度。
 * @return 正しい識別子・DLC・生値ならtrue。
 */
bool b_c620_parse_feedback(const BMotorCanFrame *frame, BC620Feedback *out);

#ifdef __cplusplus
}
#endif

#endif
