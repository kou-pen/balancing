#include "balancing/device/IndicatorOutput.hpp"
#include "balancing/app/StatusIndicator.hpp"
#include "balancing/device/StatusIndicatorPort.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace balancing;
namespace {
struct Pin { unsigned calls{0}; bool high{false}, fail{false}; };
void write(void* ctx,bool high) noexcept
{
    auto& p=*static_cast<Pin*>(ctx);
    ++p.calls;p.high=high;
}
bool checked(void* ctx,bool high) noexcept {
    auto& p=*static_cast<Pin*>(ctx);
    ++p.calls;
    if (p.fail) return false;
    p.high=high;
    return true;
}
}
int main()
{
    Pin pin;
    Led led{{write,&pin,false,true}};
    assert(pin.calls==0 && !led.toggle());
    assert(led.on() && !pin.high && pin.calls==1);
    assert(led.on() && pin.calls==1);
    assert(led.toggle() && pin.high && pin.calls==2);
    Led unverified{{write,&pin,true,false}};
    assert(!unverified.on() && pin.calls==2);
    RgbLed partial{{write,&pin,true,true},{},{}};
    assert(!partial.setRgb(true,true,true) && pin.calls==2);
    Pin red,green,blue;
    RgbLed rgb{{write,&red,true,true},{write,&green,false,true},{write,&blue,true,true}};
    assert(rgb.setRgb(true,false,false) && red.high && green.high && !blue.high);
    Pin sound;
    Buzzer buzzer{{write,&sound,true,true}};
    assert(buzzer.beep(5,1000) && sound.high);
    buzzer.update(5999);assert(buzzer.active() && sound.calls==1);
    buzzer.update(6000);assert(!buzzer.active() && !sound.high && sound.calls==2);
    assert(buzzer.beep(5,7000));buzzer.update(6999);
    assert(!buzzer.active() && !sound.high);
    assert(!buzzer.beep(5,6998));
    Buzzer disabled{{write,&sound,true,false}};
    const auto count=sound.calls;
    assert(!disabled.beep(5,10000) && sound.calls==count);
    StatusIndicator unset{};
    assert(!unset.update(RobotState::Disarmed,{},0).configured);
    IndicatorConfig cfg{};
    cfg.verified=true;
    // 全て人工表示パターン。実機の色や鳴動時間を確定しない。
    cfg.patterns[0]={false,true,false,0,0};
    cfg.patterns[2]={false,false,true,0,0};
    cfg.patterns[3]={true,false,false,4,1000};
    StatusIndicator status{cfg};
    assert(status.update(RobotState::Disarmed,{},1000).green);
    FaultRecord fault{FaultFlags::BusOff,FaultFlags::BusOff,FaultFlags::None,2000};
    auto f=status.update(RobotState::Balancing,fault,2000);
    assert(f.state==RobotState::Fault && f.red && f.buzzer && !f.blue);
    f=status.update(RobotState::Fault,fault,3000);
    assert(!f.red && f.buzzer);
    f=status.update(RobotState::Fault,fault,6000);
    assert(f.red && !f.buzzer); // 同状態で単発鳴動を再起動しない
    assert(!status.update(RobotState::Fault,fault,5999).configured);
    assert(!status.update(RobotState::Fault,fault,7000).configured);
    std::array<Pin,4> pins{};
    IndicatorOutputConfig io{};
    for (unsigned i=0;i<4;++i) io.pins[i]={nullptr,&pins[i],i%2==0,true,checked};
    StatusIndicatorPort disabledPort{io}; // 全体未検証ならOFFも書き込まない
    disabledPort.disable(); assert(!disabledPort.apply(f));
    for (const auto& p:pins) assert(!p.calls);
    io.verified=true;
    auto partialIo=io; partialIo.pins[2].verified=false;
    StatusIndicatorPort partialPort{partialIo};
    assert(!partialPort.apply(f));
    for (const auto& p:pins) assert(!p.calls);
    StatusIndicatorPort port{io};
    StatusIndicator logical{cfg};
    auto state=logical.update(RobotState::Disarmed,{},1000);
    assert(port.apply(state) && !pins[0].high && !pins[1].high && !pins[2].high && pins[3].high);
    for (const auto& p:pins) assert(p.calls==1);
    assert(port.apply(state));
    for (const auto& p:pins) assert(p.calls==1);
    state=logical.update(RobotState::Balancing,fault,2000);
    assert(port.apply(state) && pins[0].high && pins[1].high && !pins[3].high);
    assert(port.apply(logical.update(RobotState::Fault,fault,3000)) && !pins[0].high);
    assert(port.apply(logical.update(RobotState::Fault,fault,6000)) && pins[3].high);
    // 時刻逆行で論理設定無効→消灯。新しい時刻でも旧鳴動を再開しない。
    assert(!port.apply(logical.update(RobotState::Fault,fault,5999)));
    assert(!port.summary().applied && !pins[0].high && pins[1].high && pins[3].high);
    assert(!port.apply(logical.update(RobotState::Fault,fault,7000)));
    // 出力失敗は表示診断へ。全OFFは短絡せず、同じ故障を無限再試行しない。
    StatusIndicatorPort failing{io};
    pins[0].fail=true;
    assert(!failing.apply(state));
    assert(failing.summary().failed && failing.summary().writeFailures==2);
    assert(!pins[2].high && pins[1].high && pins[3].high);
    const auto calls=pins[0].calls;
    assert(!failing.apply(state)); failing.disable(); assert(pins[0].calls==calls);
    pins[0].fail=false;
    StatusIndicatorPort restarted{io};
    assert(restarted.apply(state) && pins[0].high && !restarted.summary().failed);
    restarted.disable(); assert(!pins[0].high && pins[1].high && pins[3].high);

}
