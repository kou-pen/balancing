#include "balancing/communication/Rs06Codec.hpp"
#include <cmath>

namespace balancing {
namespace {
// 通信表現用レンジ。実機の安全上限・FW版の確認とは別。
constexpr float kTorqueRangeNm = 36.0f;
constexpr float kSpeedRangeRadS = 50.0f;
constexpr float kAngleRangeRad = 12.566370614359172f;

std::uint16_t be16(const std::uint8_t* data) noexcept
{
    return static_cast<std::uint16_t>((std::uint16_t{data[0]} << 8u) | data[1]);
}
float scale(std::uint16_t raw,float minimum,float maximum) noexcept
{
    return minimum + (static_cast<float>(raw) / 65535.0f) * (maximum - minimum);
}
std::uint16_t quantize(float value,float minimum,float maximum) noexcept
{
    // 旧実装と同じdouble演算・四捨五入。対称レンジの中心は32768。
    const double midpoint=(static_cast<double>(minimum)+static_cast<double>(maximum))*0.5;
    const double raw=32767.5+(static_cast<double>(value)-midpoint)*
                     (65535.0/(static_cast<double>(maximum)-static_cast<double>(minimum)));
    return static_cast<std::uint16_t>(std::lround(raw));
}
void putBe16(std::uint8_t* data,std::uint16_t raw) noexcept
{
    data[0]=static_cast<std::uint8_t>(raw>>8u);
    data[1]=static_cast<std::uint8_t>(raw);
}
}

std::optional<Rs06StateReport> Rs06Codec::decodeFeedback(const CanFrame& frame) noexcept
{
    // Type24は診断用報告として保持。駆動確認への使用可否はRs06Motorが判断。
    if (!frame.extended || frame.dlc != 8u || frame.id > 0x1fffffffu) return std::nullopt;
    const auto type=static_cast<std::uint8_t>(frame.id>>24u);
    const auto motor=static_cast<std::uint8_t>(frame.id>>8u);
    const auto mode=static_cast<std::uint8_t>((frame.id>>22u)&3u);
    if ((type != 2u && type != 24u) || motor==0 ||
        mode>static_cast<std::uint8_t>(Rs06Mode::Running)) return std::nullopt;
    return Rs06StateReport{motor,static_cast<std::uint8_t>(frame.id),
        static_cast<Rs06Report>(type),static_cast<Rs06Mode>(mode),
        static_cast<std::uint8_t>((frame.id>>16u)&0x3fu),
        scale(be16(&frame.data[0]),-kAngleRangeRad,kAngleRangeRad),
        scale(be16(&frame.data[2]),-kSpeedRangeRadS,kSpeedRangeRadS),
        scale(be16(&frame.data[4]),-kTorqueRangeNm,kTorqueRangeNm),
        static_cast<float>(be16(&frame.data[6]))*0.1f};
}

std::optional<CanFrame> Rs06Codec::encodeControl(
    Rs06Command command,std::uint8_t motorId,std::uint8_t hostId) noexcept
{
    // 未定義コマンドは停止へ暗黙変換しない。Type4で故障解除ビットは立てない。
    if (motorId==0 || (command!=Rs06Command::Enable && command!=Rs06Command::Stop))
        return std::nullopt;
    const bool enable=command==Rs06Command::Enable;
    CanFrame frame{};
    frame.id = (enable ? (3u << 24) : (4u << 24)) | (std::uint32_t{hostId}<<8u) | motorId;
    frame.dlc = 8u;
    frame.extended = true;
    return frame;
}

std::optional<CanFrame> Rs06Codec::encodeTorque(std::uint8_t motorId,float torqueNm) noexcept
{
    if (motorId==0 || !std::isfinite(torqueNm) || std::fabs(torqueNm)>kTorqueRangeNm)
        return std::nullopt;
    CanFrame frame{};
    frame.id = (1u << 24) | (std::uint32_t{quantize(torqueNm,-kTorqueRangeNm,kTorqueRangeNm)}<<8u) | motorId;
    // 純トルク指令：位置・速度中心値、Kp/Kdゼロ。
    putBe16(&frame.data[0],quantize(0,-kAngleRangeRad,kAngleRangeRad));
    putBe16(&frame.data[2],quantize(0,-kSpeedRangeRadS,kSpeedRangeRadS));
    frame.dlc = 8u;
    frame.extended = true;
    return frame;
}
} // namespace balancing
