#include "balancing/communication/C620Codec.hpp"
#include <cmath>
namespace balancing {
namespace {
constexpr float kProtocolCurrentRaw = 16384.0f;
constexpr float kProtocolCurrentA = 20.0f;

std::uint16_t be16(const std::uint8_t* data) noexcept
{
    return static_cast<std::uint16_t>((std::uint16_t{data[0]}<<8u)|data[1]);
}
std::int16_t signedBe16(const std::uint8_t* data) noexcept
{
    // unsignedからsignedへの範囲外変換に依存せず符号拡張する。
    const auto bits=be16(data);
    const auto value=bits<=32767u?static_cast<std::int32_t>(bits):static_cast<std::int32_t>(bits)-65536;
    return static_cast<std::int16_t>(value);
}
}
std::optional<CanFrame> C620Codec::encodeCurrent(const std::array<float,4>& currentA,
                                                  float maxAbsCurrentA) noexcept
{
    if (!std::isfinite(maxAbsCurrentA) || maxAbsCurrentA<=0 || maxAbsCurrentA>kProtocolCurrentA)
        return std::nullopt;
    CanFrame parsed{};
    parsed.id = 0x200u;
    parsed.dlc = 8u;
    parsed.extended = false;
    // 1軸でも不正ならフレーム全体を拒否。旧float演算と丸めを保持。
    for (std::size_t i=0;i<currentA.size();++i) {
        if (!std::isfinite(currentA[i]) || std::fabs(currentA[i])>maxAbsCurrentA) return std::nullopt;
        const auto raw=static_cast<std::uint16_t>(static_cast<std::int16_t>(
            std::lround(currentA[i]*kProtocolCurrentRaw/kProtocolCurrentA)));
        parsed.data[2*i]=static_cast<std::uint8_t>(raw>>8u);
        parsed.data[2*i+1]=static_cast<std::uint8_t>(raw);
    }
    return parsed;
}

std::optional<C620Feedback> C620Codec::decodeFeedback(const CanFrame& frame) noexcept
{
    if (frame.extended || frame.dlc != 8u || frame.id < 0x201u || frame.id > 0x204u)
        return std::nullopt;
    const auto encoder=be16(&frame.data[0]);
    if (encoder>=8192u) return std::nullopt;
    // 積算・符号・減速比・実機ID割当は機器/推定層が所有。
    return C620Feedback{static_cast<std::uint8_t>(frame.id-0x200u),encoder,
                         signedBe16(&frame.data[2]),signedBe16(&frame.data[4]),frame.data[6]};
}
} // namespace balancing
