#include "balancing/power/BoardIO.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace balancing::power;
#define CHECK(x) do { if (!(x)) { std::cerr << "failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while(false)

// 合成条件のみ。実機の確認済みフラグを立てるものではない。
constexpr BoardInterlock safe{true,true,false,false};
constexpr PowerOutput off{PowerState::Off,false,false};
constexpr PowerOutput charging{PowerState::Precharge,true,false};
constexpr PowerOutput overlap{PowerState::Precharge,true,true};
constexpr PowerOutput on{PowerState::On,false,true};
constexpr PowerOutput fault{PowerState::Fault,false,false};
void expect(BoardOutput o,bool pre,bool main,bool led,bool accepted) {
    CHECK(o.prechargeOn==pre && o.mainOn==main && o.ledOn==led && o.accepted==accepted);
}
int main() {
    {
        BoardIO b;
        // 未検証でもOFFは妨げない。能動出力を拒否したときはFAULT表示。
        expect(b.evaluate(off,{},0),false,false,false,true);
        expect(b.evaluate(charging,{},1000),false,false,true,false);
        expect(b.evaluate(charging,{},101000),false,false,false,false);
        expect(b.evaluate(on,{},201000),false,false,true,false);
        expect(b.evaluate(off,{},202000),false,false,false,true);
    }
    {
        BoardIO b;
        // PRECHARGE内部段階の変化では表示位相を変えない。
        expect(b.evaluate(charging,safe,1000),true,false,true,true);
        expect(b.evaluate(charging,safe,250999),true,false,true,true);
        expect(b.evaluate(overlap,safe,251000),true,true,false,true);
        expect(b.evaluate(overlap,safe,500999),true,true,false,true);
        expect(b.evaluate(overlap,safe,501000),true,true,true,true);
        expect(b.evaluate(on,safe,502000),false,true,true,true);
        expect(b.evaluate(on,safe,2000000),false,true,true,true);
        expect(b.evaluate(fault,safe,2001000),false,false,true,true);
        expect(b.evaluate(fault,safe,2100999),false,false,true,true);
        expect(b.evaluate(fault,safe,2101000),false,false,false,true);
        expect(b.evaluate(fault,safe,2201000),false,false,true,true);
        expect(b.evaluate(off,safe,2202000),false,false,false,true);
        expect(b.evaluate(charging,safe,2203000),true,false,true,true);
    }
    // 各能動段階、各遮断条件の単独故障でも両出力を同時に拒否する。
    for (auto req : {charging,overlap,on}) {
        for(int i=0;i<4;++i) {
            BoardIO b; auto gate=safe;
            if(i==0) gate.powerPinsVerified=false;
            if(i==1) gate.estopOk=false;
            if(i==2) gate.hardwareFault=true;
            if(i==3) gate.shutdownPending=true;
            expect(b.evaluate(req,safe,0),req.prechargeRequested,req.mainRequested,true,true);
            expect(b.evaluate(req,gate,1000),false,false,true,false);
            expect(b.evaluate(req,gate,101000),false,false,false,false);
            expect(b.evaluate(off,gate,102000),false,false,false,true);
        }
    }
    // 状態と要求の不整合を全組合せで検査。未知enumも拒否。
    for(auto state : {PowerState::Off,PowerState::Fault,PowerState::Precharge,PowerState::On,
                       static_cast<PowerState>(255)}) {
        for(bool pre:{false,true}) for(bool main:{false,true}) {
            BoardIO b;
            const bool valid=(state==PowerState::Off || state==PowerState::Fault)?!pre&&!main:
                state==PowerState::Precharge?pre:state==PowerState::On?!pre&&main:false;
            const auto out=b.evaluate({state,pre,main},safe,0);
            CHECK(out.accepted==valid);
            CHECK(out.prechargeOn==(valid&&pre) && out.mainOn==(valid&&main));
        }
    }
    {
        BoardIO b;
        expect(b.evaluate(on,safe,1000),false,true,true,true);
        expect(b.evaluate(on,safe,999),false,false,true,false);
        expect(b.evaluate(on,safe,998),false,false,true,false);
        expect(b.evaluate(off,safe,1000),false,false,false,true);
        expect(b.evaluate(charging,safe,std::numeric_limits<std::uint64_t>::max()),true,false,true,true);
    }
    std::cout<<"F303 BoardIO logical interlock and LED pattern PASS\n";
}
