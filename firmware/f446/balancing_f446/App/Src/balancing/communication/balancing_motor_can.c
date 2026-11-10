#include "balancing/communication/balancing_motor_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RS_TORQUE_RANGE_NM 36.0f
#define RS_SPEED_RANGE_RAD_S 50.0f
#define RS_ANGLE_RANGE_RAD 12.566370614359172f
#define C620_PROTOCOL_CURRENT_A 20.0f
#define C620_PROTOCOL_CURRENT_RAW 16384.0f

static uint16_t be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t signed_be16(const uint8_t *data)
{
    const uint16_t bits = be16(data);
    const int32_t signed_value = bits <= 32767u ? (int32_t)bits : (int32_t)bits - 65536;
    return (int16_t)signed_value;
}

static void put_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static float scale_u16(uint16_t raw, float minimum, float maximum)
{
    return minimum + ((float)raw / 65535.0f) * (maximum - minimum);
}

static uint16_t unscale_u16(float value, float minimum, float maximum)
{
    const double midpoint = ((double)minimum + (double)maximum) * 0.5;
    const double quantized = 32767.5 + ((double)value - midpoint) *
                             (65535.0 / ((double)maximum - (double)minimum));
    return (uint16_t)lround(quantized);
}

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
        const int16_t raw = (int16_t)lroundf(
            current_a[i] * C620_PROTOCOL_CURRENT_RAW / C620_PROTOCOL_CURRENT_A);
        put_be16(&parsed.data[i * 2u], (uint16_t)raw);
    }
    *out = parsed;
    return true;
}

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
