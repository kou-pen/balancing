#include "balancing/app/balancing_motor_bench.h"
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

static BMotorBenchConfig config(void)
{
    BMotorBenchConfig c = {
        .rs06_motor_id = 1u, .rs06_host_id = 0xfdu,
        .c620_motor_id = 2u, .rs06_limit_nm = 0.3f,
        .c620_limit_a = 0.25f, .pulse_duration_ms = 100u
    };
    return c;
}

static void test_bench_rs06(void)
{
    BMotorBench bench;
    BMotorBenchConfig c = config();
    assert(b_bench_init(&bench, &c));
    BMotorCanFrame frame = {0};
    assert(!b_bench_rs06_frame(&bench, 0u, false, &frame));
    assert(b_bench_c620_frame(&bench, 0u, false, &frame));
    for (unsigned i=0; i<8u; ++i) assert(frame.data[i] == 0u);
    assert(!b_bench_request_rs06(&bench, 0.4f, 0u));
    assert(b_bench_request_rs06(&bench, 0.2f, 1000u));
    assert(!b_bench_request_rs06(&bench, 0.2f, 1001u));
    assert(b_bench_rs06_frame(&bench, 1000u, false, &frame));
    assert((frame.id >> 24) == 3u);
    b_bench_rs06_tx_done(&bench, &frame, 1100u);

    BRS06Feedback fb = {
        .motor_id = 1u, .host_id = 0xfdu, .mode = 2u, .fault_bits = 0u
    };
    b_bench_on_rs06_feedback(&bench, &fb, 1200u);
    assert(b_bench_rs06_frame(&bench, 2000u, true, &frame));
    assert((frame.id >> 24) == 1u);
    assert(bench.state == B_BENCH_RS06_PULSE);

    /* フレーム自身に受信時刻の判定をさせず、ポートの期限超過判定を注入。 */
    assert(b_bench_rs06_frame(&bench, 102000u, false, &frame));
    assert((frame.id >> 24) == 4u); /* stale feedback => fail closed */
    assert(bench.state == B_BENCH_FAILED);
    b_bench_rs06_tx_done(&bench, &frame, 102100u);
    assert(!bench.rs_stop_pending);
    assert(!b_bench_request_rs06(&bench, 0.1f, 103000u));
    b_bench_cancel(&bench);
    assert(bench.state == B_BENCH_FAILED);
}

static void test_bench_wheel(void)
{
    BMotorBench bench;
    BMotorBenchConfig c = config();
    assert(b_bench_init(&bench, &c));
    assert(!b_bench_request_c620(&bench, 0.2f, 1000u, false));
    /* CANポートが新しい対象IDの受信を確認した場合だけtrueを渡す。 */
    assert(b_bench_request_c620(&bench, 0.2f, 1500u, true));
    BMotorCanFrame frame = {0};
    assert(b_bench_c620_frame(&bench, 2000u, true, &frame));
    assert(frame.data[2] == 0u && frame.data[3] != 0u);
    assert(frame.data[0] == 0u && frame.data[1] == 0u);
    /* 期限超過はCANポートが判定し、ベンチはfalseで即ゼロ出力。 */
    assert(b_bench_c620_frame(&bench, 22000u, false, &frame));
    for (int i=0;i<8;++i) assert(frame.data[i] == 0u);
    assert(bench.state == B_BENCH_FAILED);
}

static void test_bench_timeout_and_cancel(void)
{
    BMotorBench bench;
    BMotorBenchConfig c = config();
    c.pulse_duration_ms = 0u;
    assert(!b_bench_init(&bench, &c));
    c = config();
    c.rs06_limit_nm = 0.6f;
    assert(!b_bench_init(&bench, &c));
    c = config();
    c.c620_limit_a = 0.6f;
    assert(!b_bench_init(&bench, &c));
    c = config();
    c.pulse_duration_ms = 201u;
    assert(!b_bench_init(&bench, &c));
    c = config();
    assert(b_bench_init(&bench, &c));
    assert(b_bench_request_rs06(&bench, 0.1f, 100u));
    BMotorCanFrame frame;
    assert(b_bench_rs06_frame(&bench, 200000u, false, &frame));
    assert((frame.id >> 24) == 4u && bench.state == B_BENCH_FAILED);
    assert(b_bench_init(&bench, &c));
    assert(b_bench_request_rs06(&bench, -0.1f, 100u));
    b_bench_cancel(&bench);
    assert(b_bench_rs06_frame(&bench, 200u, false, &frame));
    assert((frame.id >> 24) == 4u);
    b_bench_rs06_tx_done(&bench, &frame, 300u);
    assert(bench.state == B_BENCH_IDLE);
}

int main(void)
{
    test_rs06_codec();
    test_c620_codec();
    test_bench_rs06();
    test_bench_wheel();
    test_bench_timeout_and_cancel();
    puts("合格：RS06/C620 CAN codec + short pulse bench fail-safe");
    return 0;
}