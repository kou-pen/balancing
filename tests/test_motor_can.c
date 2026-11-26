#include "balancing/communication/balancing_motor_can.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_rs06_codec(void)
{
    BMotorCanFrame frame;
    assert(b_rs06_control_frame(1u, 0xfdu, true, &frame));
    assert(frame.extended && frame.dlc == 8u && frame.id == 0x0300fd01u);
    for (int i = 0; i < 8; i++) assert(frame.data[i] == 0u);
    assert(b_rs06_control_frame(1u, 0xfdu, false, &frame));
    assert(frame.id == 0x0400fd01u && frame.data[0] == 0u);
    assert(!b_rs06_control_frame(0u, 0xfdu, true, &frame));

    assert(b_rs06_torque_frame(1u, 0.0f, &frame));
    assert(frame.id == 0x01800001u);
    assert(frame.data[0] == 0x80u && frame.data[1] == 0x00u);
    assert(frame.data[2] == 0x80u && frame.data[3] == 0x00u);
    for (int i = 4; i < 8; ++i) assert(frame.data[i] == 0u);
    assert(!b_rs06_torque_frame(1u, 37.0f, &frame));
    assert(!b_rs06_torque_frame(1u, NAN, &frame));
    assert(!b_rs06_torque_frame(0u, 0.0f, &frame));

    memset(&frame, 0, sizeof(frame));
    frame.id = 0x028001fdu; /* Type2 + mode=2 + motor=1 + host=FD */
    frame.extended = true;
    frame.dlc = 8u;
    frame.data[0] = 0x80u;
    frame.data[1] = 0x00u;
    frame.data[2] = 0x80u;
    frame.data[3] = 0x00u;
    frame.data[4] = 0x80u;
    frame.data[5] = 0x00u;
    frame.data[6] = 0x01u;
    frame.data[7] = 0x2cu; /* 30.0 deg C */
    BRS06Feedback fb = {0};
    assert(b_rs06_parse_feedback(&frame, &fb));
    assert(fb.motor_id == 1u && fb.host_id == 0xfdu);
    assert(fb.mode == 2u && fb.fault_bits == 0u);
    assert(fabsf(fb.angle_rad) < 0.001f);
    assert(fabsf(fb.velocity_rad_s) < 0.002f);
    assert(fabsf(fb.torque_nm) < 0.002f);
    assert(fabsf(fb.temperature_c - 30.0f) < 0.001f);
    frame.id = 0x028101fdu;
    assert(b_rs06_parse_feedback(&frame, &fb));
    assert(fb.fault_bits == 1u);
    frame.id = 0x188001fdu; /* Type24 response: only codec format, not HW evidence */
    assert(b_rs06_parse_feedback(&frame, &fb));
    assert(fb.motor_id == 1u && fb.mode == 2u);
    frame.id = 0x028001fdu;
    frame.dlc = 7u;
    assert(!b_rs06_parse_feedback(&frame, &fb));
    frame.dlc = 8u;
    frame.id = 0x018001fdu;
    assert(!b_rs06_parse_feedback(&frame, &fb));
    frame.extended = false;
    assert(!b_rs06_parse_feedback(&frame, &fb));
}

static void test_c620_codec(void)
{
    BMotorCanFrame frame;
    float current[4] = {1.0f, -1.0f, 0.0f, 0.0f};
    assert(b_c620_current_frame(current, 2.0f, &frame));
    assert(frame.id == 0x200u && !frame.extended && frame.dlc == 8u);
    const uint8_t expected[8] = {0x03u,0x33u,0xfcu,0xcdu,0,0,0,0};
    assert(memcmp(frame.data, expected, sizeof(expected)) == 0);
    assert(!b_c620_current_frame(current, 0.5f, &frame));
    assert(!b_c620_current_frame(current, 21.0f, &frame));
    current[1] = NAN;
    assert(!b_c620_current_frame(current, 2.0f, &frame));

    memset(&frame, 0, sizeof(frame));
    frame.id = 0x202u;
    frame.dlc = 8u;
    frame.data[0] = 0x1fu;
    frame.data[1] = 0xffu; /* 8191 */
    frame.data[2] = 0xffu;
    frame.data[3] = 0x9cu; /* -100 RPM */
    frame.data[4] = 0x04u;
    frame.data[5] = 0xd2u; /* +1234 */
    frame.data[6] = 40u;
    BC620Feedback fb = {0};
    assert(b_c620_parse_feedback(&frame, &fb));
    assert(fb.motor_id == 2u && fb.encoder_raw == 8191u);
    assert(fb.rpm == -100 && fb.current_raw == 1234 && fb.temperature_c == 40u);
    frame.data[0] = 0x20u; /* invalid rotor angle */
    assert(!b_c620_parse_feedback(&frame, &fb));
    frame.data[0] = 0x1fu;
    frame.id = 0x1ffu; /* reserved output group, not feedback */
    assert(!b_c620_parse_feedback(&frame, &fb));
    frame.id = 0x205u; /* outside declared codec receive range */
    assert(!b_c620_parse_feedback(&frame, &fb));
    frame.id = 0x202u;
    frame.dlc = 7u;
    assert(!b_c620_parse_feedback(&frame, &fb));
    frame.dlc = 8u;
    frame.extended = true;
    assert(!b_c620_parse_feedback(&frame, &fb));
}

int main(void)
{
    test_rs06_codec();
    test_c620_codec();
    puts("合格：RS06/C620 CAN codec");
    return 0;
}