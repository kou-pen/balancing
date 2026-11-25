#include "balancing/communication/can/ProtocolId.hpp"
#include "balancing/communication/can/ProtocolPayload.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <limits>
#include <cstdio>

using namespace balancing;
using namespace balancing::protocol;

namespace {
struct Golden { std::uint32_t id; std::uint8_t dlc; };
// 実装のpack関数から作らず、承認仕様書の数値を独立に転記したfixture。
constexpr std::array<Golden,35> golden{{
    {0x00047C01u,0},
    {0x0004BC01u,0},
    {0x0004FC01u,0},
    {0x08404801u,2},
    {0x0840C801u,2},
    {0x04404802u,2},
    {0x0440C802u,2},
    {0x0840BC03u,8},
    {0x08408404u,8},
    {0x08408C04u,8},
    {0x0880C401u,8},
    {0x0880C402u,2},
    {0x0480C403u,2},
    {0x0880C404u,2},
    {0x0880C405u,3},
    {0x08804C06u,8},
    {0x0CC04C01u,8},
    {0x0CC04C02u,8},
    {0x0CC04C03u,8},
    {0x0CC04C04u,4},
    {0x0CC04C05u,8},
    {0x0CC04C06u,8},
    {0x0CC04C07u,8},
    {0x0CC04C08u,8},
    {0x0CC04C09u,8},
    {0x0CC04C0Au,8},
    {0x04C04C20u,8},
    {0x04C04C21u,8},
    {0x04C04C22u,8},
    {0x0CC0C480u,8},
    {0x0CC0C481u,4},
    {0x0CC04C82u,8},
    {0x0D00C401u,4},
    {0x0D004C02u,8},
    {0x0D004C03u,6},
}};

void checkIds()
{
    for (std::size_t i=0; i<golden.size(); ++i) {
        const auto g=golden[i];
        for (std::size_t j=0; j<i; ++j) assert(g.id!=golden[j].id);
        const auto fields=decodeId(g.id);
        const auto spec=findMessage(g.id);
        assert(fields && spec && spec->dlc==g.dlc);
        assert(encodeId(*fields)==g.id && encodeId(spec->fields)==g.id);
        assert(!isReservedExtendedId(g.id));
        CanFrame frame{};
        frame.id=g.id;
        frame.extended=true;
        frame.dlc=g.dlc;
        assert(matchesProtocolFrame(frame));
        // IDEとDLCのどちらか一方でも違えば受理しない。
        frame.extended=false;
        assert(!matchesProtocolFrame(frame));
        frame.extended=true;
        for (unsigned length=0;length<=9;++length) {
            frame.dlc=static_cast<std::uint8_t>(length);
            assert(matchesProtocolFrame(frame)==(length==g.dlc));
        }
        frame.id=g.id | 0x20000000u;
        frame.dlc=g.dlc;
        assert(!matchesProtocolFrame(frame));
        assert(!decodeId(frame.id));
    }
    // 同じRegister IDでも送信元・宛先・優先度が違う場合は未登録。
    const ProtocolId good{2,DataType::Remote,NodeId::Pi,NodeId::F446,1};
    assert(encodeId(good)==0x0880c401u);
    for (auto bad : std::array<ProtocolId,4>{{
        {2,DataType::Remote,NodeId::F303,NodeId::F446,1},
        {2,DataType::Remote,NodeId::Pi,NodeId::F303,1},
        {3,DataType::Remote,NodeId::Pi,NodeId::F446,1},
        {2,DataType::Remote,NodeId::Pi,NodeId::F446,0x3ff}
    }}) {
        const auto id=encodeId(bad);
        assert(id && !findMessage(*id));
        CanFrame frame{*id,{},8,true};
        assert(!matchesProtocolFrame(frame));
    }
    for (auto bad : std::array<ProtocolId,6>{{
        {4,DataType::Remote,NodeId::Pi,NodeId::F446,1},
        {8,DataType::Remote,NodeId::Pi,NodeId::F446,1},
        {2,static_cast<DataType>(0x11),NodeId::Pi,NodeId::F446,1},
        {2,DataType::Remote,NodeId::Broadcast,NodeId::F446,1},
        {2,DataType::Remote,NodeId::Pi,static_cast<NodeId>(0),1},
        {2,DataType::Remote,NodeId::Pi,NodeId::F446,0x400}
    }}) assert(!encodeId(bad));
    assert(!decodeId(0xffffffffu));
    assert(!decodeId(0x1080c401u)); // 予約priority
    assert(!decodeId(0x0883c401u)); // Broadcast送信元
    assert(!decodeId(0x0880c001u)); // 宛先0
}

void checkReservations()
{
    // 標準IDの全範囲を走査し、境界の過不足を防ぐ。
    for (std::uint32_t id=0; id<=0x7ffu; ++id) {
        const bool expected=(id>=0x1ffu && id<=0x208u)||id==0x700u;
        assert(isDjiReserved(id)==expected);
        assert(isReservedExtendedId(id)==expected);
        for (auto suffix : {0u,0x3ffffu}) {
            const auto ext=(id<<18)|suffix;
            assert(isReservedExtendedId(ext)==expected);
            if (expected) assert(!decodeId(ext) && !findMessage(ext));
        }
    }
}

void checkPayload()
{
    CanFrame frame{};
    frame.dlc=8;
    // 生バイト列の既知値でエンディアンを検証（駆動指令ではない）。
    assert(writeU16BE(frame,0,0x1234u));
    assert(writeU32BE(frame,2,0x89abcdefu));
    assert((frame.data==std::array<std::uint8_t,8>{0x12,0x34,0x89,0xab,0xcd,0xef,0,0}));
    assert(readU16BE(frame,0)==0x1234u && readU32BE(frame,2)==0x89abcdefu);
    assert(writeU32BE(frame,0,0x12345678u) && readU32BE(frame,0)==0x12345678u);
    assert(writeFloat32BE(frame,0,0.5f) && writeFloat32BE(frame,4,-0.25f));
    assert((frame.data==std::array<std::uint8_t,8>{0x3f,0,0,0,0xbe,0x80,0,0}));
    assert(readFloat32BE(frame,0)==0.5f && readFloat32BE(frame,4)==-0.25f);
    assert(writeFloat32BE(frame,0,-0.0f) && readU32BE(frame,0)==0x80000000u);
    assert(writeFloat32BE(frame,0,std::numeric_limits<float>::max()));
    assert(readFloat32BE(frame,0)==std::numeric_limits<float>::max());
    for (auto bits : {0x7f800000u,0xff800000u,0x7fc00000u,0x7f800001u}) {
        assert(writeU32BE(frame,0,bits));
        assert(!readFloat32BE(frame,0));
    }
    const auto before=frame.data;
    for (float value : {std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::infinity(),
                         -std::numeric_limits<float>::infinity()}) {
        assert(!writeFloat32BE(frame,0,value));
        assert(frame.data==before);
    }
    // DLC=0〜9、巨大なoffset、境界ちょうどを全て試す。
    for (unsigned dlc=0;dlc<=9;++dlc) {
        frame.dlc=static_cast<std::uint8_t>(dlc);
        for (std::size_t offset : {std::size_t{0},std::size_t{1},std::size_t{4},
                                   std::size_t{6},std::size_t{7},std::size_t{8},
                                   std::size_t{9},std::numeric_limits<std::size_t>::max()}) {
            const bool fits16=dlc<=8 && offset<=dlc && dlc-offset>=2;
            const bool fits32=dlc<=8 && offset<=dlc && dlc-offset>=4;
            auto old=frame.data;
            assert(writeU16BE(frame,offset,0x1234u)==fits16);
            if (!fits16) assert(frame.data==old);
            assert(bool(readU16BE(frame,offset))==fits16);
            old=frame.data;
            assert(writeU32BE(frame,offset,0x12345678u)==fits32);
            if (!fits32) assert(frame.data==old);
            assert(bool(readU32BE(frame,offset))==fits32);
            old=frame.data;
            assert(writeFloat32BE(frame,offset,0.5f)==fits32);
            if (!fits32) assert(frame.data==old);
            assert(bool(readFloat32BE(frame,offset))==fits32);
        }
    }
}
}

int main()
{
    checkIds();
    checkReservations();
    checkPayload();
    std::puts("CAN2 protocol: 35 golden IDs, reservations, IDE/DLC, BE payload PASS");
}
