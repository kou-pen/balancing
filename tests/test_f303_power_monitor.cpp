#include "balancing/power/PowerMonitor.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

using namespace balancing::power;
#define CHECK(x) do { if (!(x)) { std::cerr << "failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while (false)

// 人工fixtureのみ。ADC分解能・換算係数・物理定格の実機設定ではない。
// 2進数で厳密に表せる係数を選び、境界の丸め誤差に試験を依存させない。
constexpr PowerMonitorConfig cfg{true, {0,4095,0.125F,0.0F},
    {0,4095,0.125F,0.0F}, {0,4095,0.25F,-10.0F},
    40.0F,52.0F,40.0F,54.0F,10.0F,2000,3000};
PowerAdcSample sample(std::uint64_t t, std::uint16_t bus=384,
                      std::uint16_t current=40, std::uint16_t battery=384) {
    return {true,battery,bus,current,t};
}
void check(PowerObservation o, bool valid, bool safe) {
    CHECK(o.valid == valid && o.protectionOk == safe);
    if (!valid) CHECK(o.batteryVoltageV==0 && o.driveBusVoltageV==0 &&
                      o.currentA==0 && o.observedAtUs==0);
}
PowerOutput tick(PowerController& c, PowerMonitor& m, PowerAdcSample s,
                 PowerCommand cmd=PowerCommand::None, bool confirmed=false) {
    const auto obs=m.evaluate(s,c.output().state,s.observedAtUs);
    PowerInputs in{};
    in.command=cmd; in.estopOk=true; in.mainConfirmed=confirmed;
    in.observationValid=obs.valid; in.protectionOk=obs.protectionOk;
    in.batteryVoltageV=obs.batteryVoltageV; in.driveBusVoltageV=obs.driveBusVoltageV;
    in.observedAtUs=obs.observedAtUs;
    return c.step(in,s.observedAtUs);
}
int main() {
    // 未検証設定、範囲矛盾、非有限値、換算オーバーフローを一括拒否。
    for (int i=0;i<8;++i) {
        auto bad=cfg;
        switch(i) {
        case 0: bad.verified=false; break;
        case 1: bad.battery.scale=0; break;
        case 2: bad.current.offset=std::numeric_limits<float>::quiet_NaN(); break;
        case 3: bad.maxBatteryV=bad.minBatteryV; break;
        case 4: bad.driveBus.maxCount=0; break;
        case 5: bad.overcurrentHoldUs=0; break;
        case 6: bad.observationMaxAgeUs=0; break;
        case 7: bad.battery.scale=std::numeric_limits<float>::max(); break;
        }
        PowerMonitor m(bad); check(m.evaluate(sample(0),PowerState::Off,0),false,false);
    }
    {
        PowerMonitor m(cfg);
        auto o=m.evaluate(sample(100,376,60),PowerState::Off,100);
        check(o,true,true);
        CHECK(o.batteryVoltageV==48 && o.driveBusVoltageV==47 && o.currentA==5 && o.observedAtUs==100);
    }
    {
        auto reverse=cfg; reverse.current.scale=-0.25F; reverse.current.offset=10;
        PowerMonitor m(reverse);
        auto o=m.evaluate(sample(0,384,60),PowerState::Off,0);
        check(o,true,true); CHECK(o.currentA==-5);
    }
    // 充電前・途中・FAULTでは低バス電圧を正常と扱い、ONでは拒否。
    for (auto state : {PowerState::Off,PowerState::Precharge,PowerState::On,PowerState::Fault}) {
        PowerMonitor m(cfg);
        check(m.evaluate(sample(0,0),state,0),true,state!=PowerState::On);
        check(m.evaluate(sample(1000,320,40,320),state,1000),true,true);
        check(m.evaluate(sample(2000,432,40,416),state,2000),true,true);
        check(m.evaluate(sample(3000,433),state,3000),true,false);
        check(m.evaluate(sample(4000,384,40,319),state,4000),true,false);
        check(m.evaluate(sample(5000,384,40,417),state,5000),true,false);
    }
    {
        PowerMonitor m(cfg);
        check(m.evaluate(sample(0,319),PowerState::On,0),true,false);
        check(m.evaluate(sample(1000,320,80),PowerState::On,1000),true,true);
        check(m.evaluate(sample(2000,320,81),PowerState::On,2000),true,true);
        // 同一サンプルは成立時間を進めない。次の新規観測で等号到達。
        check(m.evaluate(sample(2000,320,81),PowerState::On,2999),true,true);
        check(m.evaluate(sample(3000,320,81),PowerState::On,3000),true,true);
        check(m.evaluate(sample(4000,320,81),PowerState::On,4000),true,false);
        check(m.evaluate(sample(5000,320,80),PowerState::On,5000),true,true);
        check(m.evaluate(sample(6000,320,81),PowerState::On,6000),true,true);
    }
    {
        // 符号逆の電流も絶対値で判定。OFF/FAULTでは持続猶予なし。
        auto signedCfg=cfg; signedCfg.current.offset=-20;
        PowerMonitor m(signedCfg);
        check(m.evaluate(sample(0,384,40),PowerState::Off,0),true,true);
        check(m.evaluate(sample(1000,384,39),PowerState::Off,1000),true,false);
        check(m.evaluate(sample(2000,384,39),PowerState::Fault,2000),true,false);
        check(m.evaluate(sample(3000,384,39),PowerState::Precharge,3000),true,true);
        check(m.evaluate(sample(4000,384,39),PowerState::On,4000),true,true);
        check(m.evaluate(sample(5000,384,39),PowerState::On,5000),true,false);
    }
    {
        PowerMonitor m(cfg);
        check(m.evaluate(sample(0),PowerState::On,0),true,true);
        check(m.evaluate(sample(0),PowerState::On,2999),true,true);
        check(m.evaluate(sample(0),PowerState::On,3000),false,false);
        check(m.evaluate(sample(3000),PowerState::On,3000),true,false);
        check(m.evaluate(sample(4000),PowerState::On,4000),true,true);
        // 未来、同時刻改変、観測/評価時計逆行を棄却。
        check(m.evaluate(sample(5001),PowerState::On,5000),false,false);
        check(m.evaluate(sample(4000,380),PowerState::On,5000),false,false);
        check(m.evaluate(sample(3999),PowerState::On,5000),false,false);
        check(m.evaluate(sample(4500),PowerState::On,4999),false,false);
        check(m.evaluate(sample(5000),PowerState::On,5000),true,true);
        auto invalid=sample(6000); invalid.valid=false;
        check(m.evaluate(invalid,PowerState::On,6000),false,false);
        check(m.evaluate(sample(6000,4096),PowerState::On,6000),false,false);
        check(m.evaluate(sample(6000,384,4096),PowerState::On,6000),false,false);
        check(m.evaluate(sample(6000,384,40,0),PowerState::On,6000),false,false);
    }
    {
        // 実際のControllerにMonitorの結果を入力。物理出力・HALはなし。
        PowerController c({true,1.0F,2000,10000,3000,3000},ResetCause::BatteryConnection);
        PowerMonitor m(cfg);
        CHECK(tick(c,m,sample(0,0)).state==PowerState::Precharge);
        CHECK(tick(c,m,sample(1000,376)).state==PowerState::Precharge);
        CHECK(tick(c,m,sample(2000,376)).state==PowerState::Precharge);
        auto o=tick(c,m,sample(3000,376));
        CHECK(o.state==PowerState::Precharge && o.mainRequested && o.prechargeRequested);
        CHECK(tick(c,m,sample(4000,376),PowerCommand::None,true).state==PowerState::On);
        CHECK(tick(c,m,sample(5000,376,81)).state==PowerState::On);
        CHECK(tick(c,m,sample(6000,376,81)).state==PowerState::On);
        o=tick(c,m,sample(7000,376,81));
        CHECK(o.state==PowerState::Fault && !o.mainRequested && !o.prechargeRequested);
        CHECK(tick(c,m,sample(8000,376,81),PowerCommand::On).commandResult==PowerCommandResult::SafetyRejected);
        CHECK(tick(c,m,sample(9000,0)).state==PowerState::Fault);
        o=tick(c,m,sample(10000,0),PowerCommand::On);
        CHECK(o.state==PowerState::Precharge && !o.mainRequested && o.prechargeRequested);
        // 再投入後のADC欠測もControllerで遮断する。
        auto missing=sample(11000); missing.valid=false;
        o=tick(c,m,missing); CHECK(o.state==PowerState::Fault && !o.mainRequested && !o.prechargeRequested);
    }
    std::cout<<"F303 PowerMonitor conversion, protection and Controller integration PASS\n";
}
