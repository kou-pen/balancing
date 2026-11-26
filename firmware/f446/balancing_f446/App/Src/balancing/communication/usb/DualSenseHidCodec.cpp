#include "balancing/communication/usb/DualSenseHidCodec.hpp"
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
}
