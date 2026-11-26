#include "balancing/communication/Rs06Codec.hpp"
#include "balancing/communication/C620Codec.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
using namespace balancing;

#include <assert.h>
#include <cmath>
#include <limits>
#include <stdio.h>
#include <string.h>

static void test_rs06_codec(void)
{
    CanFrame frame;
    { const auto encoded=Rs06Codec::encodeControl(Rs06Command::Enable,1u,0xfdu);assert(encoded);frame=*encoded; }
    assert(frame.extended && frame.dlc == 8u && frame.id == 0x0300fd01u);
    for (int i = 0; i < 8; i++) assert(frame.data[i] == 0u);
    { const auto encoded=Rs06Codec::encodeControl(Rs06Command::Stop,1u,0xfdu);assert(encoded);frame=*encoded; }
    assert(frame.id == 0x0400fd01u && frame.data[0] == 0u);
    assert(!Rs06Codec::encodeControl(Rs06Command::Enable,0u,0xfdu));

    { const auto encoded=Rs06Codec::encodeTorque(1u,0.0f);assert(encoded);frame=*encoded; }
    assert(frame.id == 0x01800001u);
    assert(frame.data[0] == 0x80u && frame.data[1] == 0x00u);
    assert(frame.data[2] == 0x80u && frame.data[3] == 0x00u);
    for (int i = 4; i < 8; ++i) assert(frame.data[i] == 0u);
    assert(!Rs06Codec::encodeTorque(1u,37.0f));
    assert(!Rs06Codec::encodeTorque(1u,NAN));
    assert(!Rs06Codec::encodeTorque(0u,0.0f));

    frame={};
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
    std::optional<Rs06StateReport> fb;
    fb=Rs06Codec::decodeFeedback(frame);assert(fb);
    assert(fb->motorId == 1u && fb->hostId == 0xfdu);
    assert(fb->mode == Rs06Mode::Running && fb->faultBits == 0u);
    assert(fabsf(fb->angleRad) < 0.001f);
    assert(fabsf(fb->velocityRadPerSec) < 0.002f);
    assert(fabsf(fb->torqueNm) < 0.002f);
    assert(fabsf(fb->temperatureC - 30.0f) < 0.001f);
    frame.id = 0x028101fdu;
    fb=Rs06Codec::decodeFeedback(frame);assert(fb);
    assert(fb->faultBits == 1u);
    frame.id = 0x188001fdu; /* Type24 response: only codec format, not HW evidence */
    fb=Rs06Codec::decodeFeedback(frame);assert(fb);
    assert(fb->motorId == 1u && fb->mode == Rs06Mode::Running);
    frame.id = 0x028001fdu;
    frame.dlc = 7u;
    assert(!Rs06Codec::decodeFeedback(frame));
    frame.dlc = 8u;
    frame.id = 0x018001fdu;
    assert(!Rs06Codec::decodeFeedback(frame));
    frame.extended = false;
    assert(!Rs06Codec::decodeFeedback(frame));
}

static void test_c620_codec(void)
{
    CanFrame frame;
    std::array<float,4> current = {1.0f, -1.0f, 0.0f, 0.0f};
    { const auto encoded=C620Codec::encodeCurrent(current,2.0f);assert(encoded);frame=*encoded; }
    assert(frame.id == 0x200u && !frame.extended && frame.dlc == 8u);
    const std::array<std::uint8_t,8> expected = {0x03u,0x33u,0xfcu,0xcdu,0,0,0,0};
    assert(frame.data==expected);
    assert(!C620Codec::encodeCurrent(current,0.5f));
    assert(!C620Codec::encodeCurrent(current,21.0f));
    current[1] = NAN;
    assert(!C620Codec::encodeCurrent(current,2.0f));

    frame={};
    frame.id = 0x202u;
    frame.dlc = 8u;
    frame.data[0] = 0x1fu;
    frame.data[1] = 0xffu; /* 8191 */
    frame.data[2] = 0xffu;
    frame.data[3] = 0x9cu; /* -100 RPM */
    frame.data[4] = 0x04u;
    frame.data[5] = 0xd2u; /* +1234 */
    frame.data[6] = 40u;
    std::optional<C620Feedback> fb;
    fb=C620Codec::decodeFeedback(frame);assert(fb);
    assert(fb->motorId == 2u && fb->encoderRaw == 8191u);
    assert(fb->rpm == -100 && fb->currentRaw == 1234 && fb->temperatureC == 40u);
    frame.data[0] = 0x20u; /* invalid rotor angle */
    assert(!C620Codec::decodeFeedback(frame));
    frame.data[0] = 0x1fu;
    frame.id = 0x1ffu; /* reserved output group, not feedback */
    assert(!C620Codec::decodeFeedback(frame));
    frame.id = 0x205u; /* outside declared codec receive range */
    assert(!C620Codec::decodeFeedback(frame));
    frame.id = 0x202u;
    frame.dlc = 7u;
    assert(!C620Codec::decodeFeedback(frame));
    frame.dlc = 8u;
    frame.extended = true;
    assert(!C620Codec::decodeFeedback(frame));
}

int main()
{
    test_rs06_codec();
    test_c620_codec();
    // 量子化の正負境界、予約モード、最大ID、全4チャネルと符号付き極値。
    assert(Rs06Codec::encodeTorque(255,36)->id==0x01ffffffu);
    assert(Rs06Codec::encodeTorque(255,-36)->id==0x010000ffu);
    assert(!Rs06Codec::encodeTorque(1,-36.01f));
    assert(!Rs06Codec::encodeTorque(1,std::numeric_limits<float>::infinity()));
    assert(!Rs06Codec::encodeControl(static_cast<Rs06Command>(99),1,0));
    assert(!Rs06Codec::decodeFeedback({0x02c001fdu,{},8,true}));
    assert(!Rs06Codec::decodeFeedback({0x228001fdu,{},8,true}));
    assert(!Rs06Codec::decodeFeedback({0x028000fdu,{},8,true}));
    const auto ends=C620Codec::encodeCurrent({20,-20,10,-10},20);
    assert(ends && (ends->data==std::array<std::uint8_t,8>{0x40,0,0xc0,0,0x20,0,0xe0,0}));
    assert(!C620Codec::encodeCurrent({},0));
    assert(!C620Codec::encodeCurrent({},NAN));
    assert(!C620Codec::encodeCurrent({0,0,0,INFINITY},20));
    const auto edge=C620Codec::decodeFeedback({0x204u,{0,0,0x80,0,0x7f,0xff,0,0},8,false});
    assert(edge && edge->motorId==4 && edge->rpm==-32768 && edge->currentRaw==32767);
    puts("合格：RS06/C620 CAN codec");
    return 0;
}