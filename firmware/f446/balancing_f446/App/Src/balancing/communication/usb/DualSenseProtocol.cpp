#include "balancing/communication/usb/DualSenseProtocol.hpp"
#include <cmath>
namespace balancing {
namespace {
bool axisValid(const HidAxis& a,unsigned n) noexcept {
    return a.offset>0 && a.offset<n && a.min<a.center && a.center<a.max &&
        std::isfinite(a.limit) && a.limit>0;
}
bool buttonValid(const HidButton& b,unsigned n) noexcept {
    return b.offset>0 && b.offset<n && b.mask && !(b.mask&(b.mask-1));
}
std::optional<float> axis(const HidAxis& a,std::uint8_t v) noexcept {
    if (v<a.min || v>a.max) return {};
    // 正負で異なるADC幅を個別に正規化。中心を厳密に0とし、範囲外は飽和で隠さない。
    const int delta=int(v)-a.center;
    const float value=float(delta)/float(delta<0?a.center-a.min:a.max-a.center)*a.limit;
    return a.inverted?-value:value;
}
}
DualSenseHidCodec::DualSenseHidCodec(DualSenseHidConfig c) noexcept : config_{c} {
    // USB転送境界の固定上限。ボタン同士は別ビットなら同一バイトでもよい。
    valid_=c.verified && c.reportId && c.length<=64 && axisValid(c.forward,c.length) &&
        axisValid(c.yaw,c.length) && c.forward.offset!=c.yaw.offset &&
        buttonValid(c.options,c.length) && buttonValid(c.circle,c.length) &&
        c.options.offset!=c.forward.offset && c.options.offset!=c.yaw.offset &&
        c.circle.offset!=c.forward.offset && c.circle.offset!=c.yaw.offset &&
        (c.options.offset!=c.circle.offset || c.options.mask!=c.circle.mask);
}
std::optional<DualSenseInput> DualSenseHidCodec::decode(const std::uint8_t* d,std::size_t n) const noexcept {
    if (!valid_ || !d || n!=config_.length || d[0]!=config_.reportId) return {};
    const auto f=axis(config_.forward,d[config_.forward.offset]);
    const auto y=axis(config_.yaw,d[config_.yaw.offset]);
    if (!f || !y) return {};
    return DualSenseInput{*f,*y,bool(d[config_.options.offset]&config_.options.mask),
                                bool(d[config_.circle.offset]&config_.circle.mask)};
}

namespace {
std::uint16_t le16(const std::uint8_t* p) noexcept
{ return static_cast<std::uint16_t>(p[0] | (static_cast<unsigned>(p[1])<<8u)); }
bool validInAddress(std::uint8_t address) noexcept
{ return (address & 0xf0u)==0x80u && (address & 0x0fu)!=0u; }
}
bool validUsbHostConfig(const UsbHostConfig& c) noexcept
{
    return c.verified && c.vendorId && c.productId && validInAddress(c.endpointAddress) &&
           c.packetBytes && c.packetBytes<=64u && c.reportLength && c.reportLength<=c.packetBytes &&
           c.intervalMs && c.transferTimeoutUs>=static_cast<std::uint64_t>(c.intervalMs)*1000u;
}
bool matchesDualSenseInterface(const UsbHostConfig& c,const UsbHostInterface& i) noexcept
{
    return validUsbHostConfig(c) && i.vendorId==c.vendorId && i.productId==c.productId &&
           i.fullSpeed && i.number==c.interfaceNumber && i.alternate==0u &&
           i.interfaceClass==3u && i.subclass==0u && i.protocol==0u &&
           i.endpointAddress==c.endpointAddress && (i.attributes&3u)==3u &&
           i.packetBytes==c.packetBytes && i.intervalMs==c.intervalMs;
}

std::optional<UsbHostInterface> findDualSenseInterface(const UsbHostConfig& c,
    std::uint16_t vendorId,std::uint16_t productId,bool fullSpeed,
    const std::uint8_t* data,std::size_t length) noexcept
{
    // USBデバイス由来の長さを信用しない。固定512バイトの列挙境界を超えない。
    if (!validUsbHostConfig(c) || vendorId!=c.vendorId || productId!=c.productId ||
        !fullSpeed || !data || length<9u || length>512u || data[0]!=9u ||
        data[1]!=2u || le16(data+2)!=length || !data[4] || !data[5]) return {};
    UsbHostInterface candidate{};
    candidate.vendorId=vendorId; candidate.productId=productId; candidate.fullSpeed=fullSpeed;
    bool selected=false,seen=false,found=false;
    unsigned expectedEndpoints=0,actualEndpoints=0;
    for (std::size_t offset=9u;offset<length;) {
        const auto* d=data+offset;
        // bLength=0で無限ループにしない。末尾の短いdescriptorも拒否する。
        if (length-offset<2u || d[0]<2u || d[0]>length-offset) return {};
        if (d[1]==4u) {
            if (d[0]!=9u || (selected && actualEndpoints!=expectedEndpoints)) return {};
            selected=d[2]==c.interfaceNumber && d[3]==0u;
            if (selected) {
                if (seen || d[5]!=3u || d[6]!=0u || d[7]!=0u) return {};
                seen=true; expectedEndpoints=d[4]; actualEndpoints=0;
                candidate.number=d[2]; candidate.alternate=d[3];
                candidate.interfaceClass=d[5]; candidate.subclass=d[6]; candidate.protocol=d[7];
            }
        } else if (d[1]==5u) {
            if (d[0]<7u) return {};
            if (selected) {
                if (++actualEndpoints>expectedEndpoints) return {};
                if (d[2]==c.endpointAddress) {
                    if (found) return {};
                    candidate.endpointAddress=d[2]; candidate.attributes=d[3];
                    candidate.packetBytes=le16(d+4); candidate.intervalMs=d[6];
                    found=true;
                }
            }
        } else if (d[1]==2u) {
            // 一つの構成descriptor内に別の構成を混ぜない。
            return {};
        }
        offset+=d[0];
    }
    if ((selected && actualEndpoints!=expectedEndpoints) || !seen || !found ||
        !matchesDualSenseInterface(c,candidate)) return {};
    return candidate;
}
}
