#include "balancing/device/UsbHidInput.hpp"
#include <limits>
namespace balancing {
void UsbHidInput::invalidate() noexcept {
    last_.reset();
    // 世代番号の再利用を防ぐ。枯渇時は永続無効とする。
    if (continuity_!=std::numeric_limits<std::uint64_t>::max()) ++continuity_;
    else connected_=false;
}
bool UsbHidInput::connect(std::uint64_t g,std::uint64_t at) noexcept {
    if (!enabled() || !g || g<=generation_ || at<boundary_ ||
        (last_ && at<*last_)) return false;
    generation_=g; serial_=0; boundary_=at; connected_=true; invalidate();
    return connected_;
}
bool UsbHidInput::disconnect(std::uint64_t g,std::uint64_t at) noexcept {
    if (!connected_ || g!=generation_ || at<boundary_ || (last_ && at<*last_)) return false;
    connected_=false; boundary_=at; invalidate(); return true;
}
bool UsbHidInput::fresh(std::uint64_t now) const noexcept {
    return connected_ && last_ && now>=*last_ && now-*last_<=timeout_;
}
std::optional<DualSenseInput> UsbHidInput::receive(std::uint64_t g,std::uint64_t serial,
    const std::uint8_t* data,std::size_t n,std::uint64_t at,std::uint64_t now) noexcept {
    if (!connected_ || g!=generation_ || !serial || serial<=serial_ ||
        at<boundary_ || (serial_ && at==boundary_) || at>now || (last_ && at<=*last_)) return {};
    // 転送番号の欠落や時間空白を長押しの連続性として数えない。
    if (serial_ && (serial-serial_!=1 || !fresh(at))) invalidate();
    serial_=serial; boundary_=at;
    const auto decoded=codec_.decode(data,n);
    if (!decoded || now-at>timeout_ || !connected_) { invalidate(); return {}; }
    last_=at; return decoded;
}
}
