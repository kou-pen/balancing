#include "balancing/communication/balancing_motor_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * ここにある量子化の最大幅は通信フォーマット用の範囲で、実機トルクの
 * 許可上限ではない。RS06の速度範囲はFW版で差があるため、
 * 実機で使用する版と一致するまでは試験値としてのみ使う。
 */
#define RS_TORQUE_RANGE_NM 36.0f
#define RS_SPEED_RANGE_RAD_S 50.0f
#define RS_ANGLE_RANGE_RAD 12.566370614359172f
#define C620_PROTOCOL_CURRENT_A 20.0f
#define C620_PROTOCOL_CURRENT_RAW 16384.0f

/* CANペイロードの上位バイト先行16bitをCPUのエンディアンと
 * 無関係に組み立てる。メモリ上の直接キャストはアラインメントに依存する。 */
static uint16_t be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/* 受信した符号付き16bit生値を明示的に符号拡張する。
 * uint16_t→int16_tの実装依存キャストを使わず、負の値は65536を引く。 */
static int16_t signed_be16(const uint8_t *data)
{
    const uint16_t bits = be16(data);
    const int32_t signed_value = bits <= 32767u ? (int32_t)bits : (int32_t)bits - 65536;
    return (int16_t)signed_value;
}

/* CANの2バイト整数はビッグエンディアン。送信と受信を対称に保つ。 */
static void put_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

/*
 * 量子化値0..65535を指定された物理レンジへ線形復元する。
 * 速度や角度のレンジはハードウェア校正値を代替しない。
 */
static float scale_u16(uint16_t raw, float minimum, float maximum)
{
    return minimum + ((float)raw / 65535.0f) * (maximum - minimum);
}

/*
 * 物理値を16bitへ変換。0のような対称レンジの中心は
 * 32767.5に対応するため丸め規則が重要。入力の範囲・有限性は
 * 呼出し元が検証し、ここでの量子化を飽和制御と混同しない。
 */
static uint16_t unscale_u16(float value, float minimum, float maximum)
{
    const double midpoint = ((double)minimum + (double)maximum) * 0.5;
    const double quantized = 32767.5 + ((double)value - midpoint) *
                             (65535.0 / ((double)maximum - (double)minimum));
    return (uint16_t)lround(quantized);
}

/*
 * RS06 Type3（有効化）/Type4（無効化）は拡張IDで識別する。
 * DATAは8バイトのゼロ。生成に成功しても実CAN送信完了や
 * Type3後の正常フィードバックを確認したことにはならない。
 */
bool b_rs06_control_frame(uint8_t motor_id, uint8_t host_id,
                          bool enable, BMotorCanFrame *out)
{
    if (out == NULL || motor_id == 0u) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->id = (enable ? (3u << 24) : (4u << 24))
            | ((uint32_t)host_id << 8) | motor_id;
    out->dlc = 8u;
    out->extended = true;
    return true;
}

/*
 * RS06 Type1では拡張IDにトルクの16bit量子値を格納する。
 * 角度/速度のゼロ指令・Kp/Kdゼロは純トルクモードのテスト契約。
 * 通信上の±36 N·mは実機のトルク安全上限として使ってはいけない。
 */
bool b_rs06_torque_frame(uint8_t motor_id, float torque_nm, BMotorCanFrame *out)
{
    if (out == NULL || motor_id == 0u ||
        !isfinite(torque_nm) || fabsf(torque_nm) > RS_TORQUE_RANGE_NM) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->id = (1u << 24)
            | ((uint32_t)unscale_u16(torque_nm, -RS_TORQUE_RANGE_NM,
                                     RS_TORQUE_RANGE_NM) << 8)
            | motor_id;
    /* 位置・速度の中心量子値、Kp=Kd=0。純トルク制御として使用。 */
    put_be16(&out->data[0], unscale_u16(0.0f, -RS_ANGLE_RANGE_RAD, RS_ANGLE_RANGE_RAD));
    put_be16(&out->data[2], unscale_u16(0.0f, -RS_SPEED_RANGE_RAD_S, RS_SPEED_RANGE_RAD_S));
    out->dlc = 8u;
    out->extended = true;
    return true;
}

/*
 * 受信したType2/24だけを解析する。拡張形式、DLC、IDの範囲を確認し、
 * 不正フレームでは呼出し元の値を変更しない。
 * Type24のFW差・実際のDATA構造は未検証のため、実運転での受理可否は
 * 別途実機通信ログと照合する必要がある。
 */
bool b_rs06_parse_feedback(const BMotorCanFrame *frame, BRS06Feedback *out)
{
    if (frame == NULL || out == NULL || !frame->extended || frame->dlc != 8u ||
        frame->id > 0x1fffffffu) {
        return false;
    }
    const uint8_t type = (uint8_t)((frame->id >> 24) & 0x1fu);
    if (type != 2u && type != 24u) {
        return false;
    }
    const uint8_t motor_id = (uint8_t)(frame->id >> 8);
    if (motor_id == 0u) {
        return false;
    }
    BRS06Feedback parsed = {0};
    parsed.motor_id = motor_id;
    parsed.host_id = (uint8_t)frame->id;
    parsed.mode = (uint8_t)((frame->id >> 22) & 3u);
    parsed.fault_bits = (uint8_t)((frame->id >> 16) & 0x3fu);
    /* 通信上の各量子化レンジからSI値を復元する。
     * 実際のモータFWが同一レンジを採用するかは現物確認が必要。 */
    parsed.angle_rad = scale_u16(be16(&frame->data[0]),
                                 -RS_ANGLE_RANGE_RAD, RS_ANGLE_RANGE_RAD);
    parsed.velocity_rad_s = scale_u16(be16(&frame->data[2]),
                                      -RS_SPEED_RANGE_RAD_S, RS_SPEED_RANGE_RAD_S);
    parsed.torque_nm = scale_u16(be16(&frame->data[4]),
                                 -RS_TORQUE_RANGE_NM, RS_TORQUE_RANGE_NM);
    parsed.temperature_c = (float)be16(&frame->data[6]) * 0.1f;
    *out = parsed;
    return true;
}

/*
 * C620へは標準ID 0x200、8バイトで4チャンネル分の符号付き電流を送る。
 * すべてのチャンネルを検証してからoutへ確定するため、
 * 1つの不正値でも部分的な指令フレームは渡さない。
 */
bool b_c620_current_frame(const float current_a[4], float max_abs_current_a,
                          BMotorCanFrame *out)
{
    if (current_a == NULL || out == NULL || !isfinite(max_abs_current_a) ||
        max_abs_current_a <= 0.0f || max_abs_current_a > C620_PROTOCOL_CURRENT_A) {
        return false;
    }
    BMotorCanFrame parsed = {0};
    parsed.id = 0x200u;
    parsed.dlc = 8u;
    parsed.extended = false;
    for (unsigned int i = 0; i < 4u; ++i) {
        if (!isfinite(current_a[i]) || fabsf(current_a[i]) > max_abs_current_a) {
            return false;
        }
        /* 20 Aと16384カウントは通信表現の尺度であり、実機の許容電流ではない。
         * 機械・電源上限は上位で別途検証して設定する。 */
        const int16_t raw = (int16_t)lroundf(
            current_a[i] * C620_PROTOCOL_CURRENT_RAW / C620_PROTOCOL_CURRENT_A);
        put_be16(&parsed.data[i * 2u], (uint16_t)raw);
    }
    *out = parsed;
    return true;
}

/*
 * C620の返信は標準ID 0x201..0x204、8バイト固定。
 * 生エンコーダはロータ1回転内だけなので、車輪位置の積算と
 * 左右の実ID対応はこのデコーダではなく観測層が担当する。
 */
bool b_c620_parse_feedback(const BMotorCanFrame *frame, BC620Feedback *out)
{
    if (frame == NULL || out == NULL || frame->extended || frame->dlc != 8u ||
        frame->id < 0x201u || frame->id > 0x204u) {
        return false;
    }
    BC620Feedback parsed = {0};
    parsed.motor_id = (uint8_t)(frame->id - 0x200u);
    parsed.encoder_raw = be16(&frame->data[0]);
    if (parsed.encoder_raw >= 8192u) {
        return false;
    }
    parsed.rpm = signed_be16(&frame->data[2]);
    parsed.current_raw = signed_be16(&frame->data[4]);
    parsed.temperature_c = frame->data[6];
    *out = parsed;
    return true;
}
