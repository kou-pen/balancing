#include "balancing/app/DualSenseService.hpp"
#include <limits>
namespace balancing {
DualSenseService::DualSenseService(RemoteControlService& remote,DualSenseConfig c) noexcept
    : remote_{remote},config_{c},input_{c.hid,c.reportTimeoutUs},source_{remote.selectedSource()} {}
bool DualSenseService::Hold::update(bool pressed,std::uint64_t at,std::uint64_t duration) noexcept {
    // 接続時に押されたままのボタンは無視。明示解放から新たな押下を必要とする。
    if (!pressed) { released=true; fired=false; since.reset(); return false; }
    if (!released || fired) return false;
    if (!since) since=at;
    if (at<*since || at-*since<duration) return false;
    fired=true; return true;
}
void DualSenseService::resetInput() noexcept {
    arm_={}; disarm_={}; pendingArm_=false;
    // 既に成立したDISARMは切断で取り消さない。
    remote_.invalidateUsbInput();
}
void DualSenseService::syncSource() noexcept {
    const auto s=remote_.selectedSource();
    if (s!=source_) { arm_={}; pendingArm_=false; source_=s; }
}
bool DualSenseService::connect(std::uint64_t g,std::uint64_t at) noexcept {
    if (!enabled() || !input_.connect(g,at)) return false;
    resetInput(); return true;
}
bool DualSenseService::disconnect(std::uint64_t g,std::uint64_t at) noexcept {
    if (!input_.disconnect(g,at)) return false;
    resetInput(); return true;
}
bool DualSenseService::receive(std::uint64_t g,std::uint64_t serial,const std::uint8_t* data,
    std::size_t n,std::uint64_t at,std::uint64_t now) noexcept {
    if (!enabled()) return false;
    syncSource();
    const auto before=input_.continuity();
    const auto decoded=input_.receive(g,serial,data,n,at,now);
    if (before!=input_.continuity()) resetInput();
    if (!decoded) return false;
    // DISARMは速度設定や選択元の制約に従属させない。
    pendingDisarm_ |= disarm_.update(decoded->circle,at,config_.disarmHoldUs);
    if (source_==B_OPERATOR_USB_DUALSENSE) {
        if (remote_.acceptDecoded(B_OPERATOR_USB_DUALSENSE,{true,at,decoded->speedMps,decoded->yawRadS},true,now))
            pendingArm_ |= arm_.update(decoded->options,at,config_.armHoldUs);
        else { arm_={}; pendingArm_=false; }
    }
    if (pendingDisarm_) pendingArm_=false;
    return true;
}
OperatorEvents DualSenseService::takeEvents(std::uint64_t now) noexcept {
    if (!enabled()) return {};
    syncSource();
    if (!input_.fresh(now)) resetInput();
    OperatorEvents out{};
    out.disarm=pendingDisarm_;
    out.arm=pendingArm_ && !out.disarm && source_==B_OPERATOR_USB_DUALSENSE;
    pendingArm_=pendingDisarm_=false;
    return out;
}
bool DualSenseService::selectSource(BOperatorSource target,RobotState state,std::uint64_t now) noexcept {
    if (!enabled() || !input_.fresh(now) || !remote_.selectSource(target,state)) return false;
    syncSource(); return true;
}

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
