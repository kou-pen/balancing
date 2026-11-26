#include "balancing/communication/usb/DualSenseHostClass.hpp"

namespace balancing {
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
