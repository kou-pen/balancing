#include "balancing/communication/can/ProtocolPayload.hpp"
#include <cstring>
#include <limits>

namespace balancing::protocol {
namespace {

static_assert(sizeof(float)==sizeof(std::uint32_t)
              && std::numeric_limits<float>::is_iec559
              && std::numeric_limits<float>::digits==24
              && std::numeric_limits<float>::max_exponent==128,
              "CAN2 requires IEEE754 binary32");

bool fits(const CanFrame& frame, std::size_t offset, std::size_t bytes) noexcept
{
    // offset+bytesはsize_tの周回で誤判定し得るため、引き算で検査。
    // 配列容量ではなく実際のDLCを境界とする。
    return frame.dlc <= frame.data.size() && offset <= frame.dlc
        && bytes <= frame.dlc-offset;
}

bool finiteBits(std::uint32_t bits) noexcept
{
    // 指数部が全1ならNaN/Inf。浮動小数点比較や型の生コピーに依存しない。
    return (bits & 0x7f800000u) != 0x7f800000u;
}

} // namespace

bool writeU16BE(CanFrame& frame, std::size_t offset, std::uint16_t value) noexcept
{
    if (!fits(frame,offset,2)) return false;
    frame.data[offset]=static_cast<std::uint8_t>(value >> 8);
    frame.data[offset+1]=static_cast<std::uint8_t>(value);
    return true;
}

bool writeU32BE(CanFrame& frame, std::size_t offset, std::uint32_t value) noexcept
{
    if (!fits(frame,offset,4)) return false;
    // ホストのバイト順によらず上位バイトを先に置く。
    for (std::size_t i=0; i<4; ++i)
        frame.data[offset+i]=static_cast<std::uint8_t>(value >> (8*(3-i)));
    return true;
}

bool writeFloat32BE(CanFrame& frame, std::size_t offset, float value) noexcept
{
    std::uint32_t bits{};
    // aliasing違反を避けて表現ビットを取得。非有限値は書き込む前に拒否。
    std::memcpy(&bits,&value,sizeof(bits));
    return finiteBits(bits) && writeU32BE(frame,offset,bits);
}

std::optional<std::uint16_t> readU16BE(const CanFrame& frame, std::size_t offset) noexcept
{
    if (!fits(frame,offset,2)) return std::nullopt;
    return static_cast<std::uint16_t>((std::uint32_t{frame.data[offset]} << 8)
                                     | frame.data[offset+1]);
}

std::optional<std::uint32_t> readU32BE(const CanFrame& frame, std::size_t offset) noexcept
{
    if (!fits(frame,offset,4)) return std::nullopt;
    std::uint32_t value=0;
    for (std::size_t i=0; i<4; ++i)
        value=(value << 8) | frame.data[offset+i];
    return value;
}

std::optional<float> readFloat32BE(const CanFrame& frame, std::size_t offset) noexcept
{
    const auto bits=readU32BE(frame,offset);
    // NaN/Infはfloatに変換する前に拒否する。物理量の範囲は個別Codecの責務。
    if (!bits || !finiteBits(*bits)) return std::nullopt;
    float value{};
    std::memcpy(&value,&*bits,sizeof(value));
    return value;
}

} // namespace balancing::protocol
