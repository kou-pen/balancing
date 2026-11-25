#include "balancing/device/Buzzer.hpp"
#include "balancing/app/StatusIndicator.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
using namespace balancing;
namespace {
struct Pin { unsigned calls{0}; bool high{false}; };
void write(void* ctx,bool high) noexcept
{
    auto& p=*static_cast<Pin*>(ctx);
    ++p.calls;p.high=high;
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
}
