#include "balancing/device/F446Watchdog.hpp"
#include "balancing/bench/MotorBench.hpp"
#include "balancing_f446_bench_bridge.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstring>
#include <cstdio>

using namespace balancing;
namespace {
CAN_TypeDef can1{},can2{};
TestDwtRegisters dwt{};
TestCoreDebugRegisters debug{};
std::uint32_t primask{};
std::array<CAN_FilterTypeDef,28> filters{};
std::array<unsigned,2> notifications{};
unsigned filterCalls{},startCalls{},stopCalls{};
int failFilter=-1;
bool failStart{},failNotification{},failRollback{};
unsigned index(CAN_HandleTypeDef* h) { return h->Instance==&can1?0u:1u; }
CAN_TypeDef& regs(unsigned bus) { return bus==1?can1:can2; }
constexpr MotorBenchConfig rsCfg{BenchAxis::Rs06,11,0xfd,0.2f,8,10000,true};
constexpr MotorBenchConfig wheelCfg{BenchAxis::C620,4,0,0.2f,8,10000,true};

void rx(unsigned bus,std::uint32_t id,bool extended,unsigned dlc=8) {
    auto& r=regs(bus);
    assert(r.RF0R.count<3);
    r.sFIFOMailBox[r.RF0R.count++]={extended?(id<<3u)|CAN_RI0R_IDE:id<<21u,dlc,0,0};
}
void rs(unsigned motor=11,unsigned host=0xfd,unsigned mode=0) {
    rx(1,(2u<<24u)|(mode<<22u)|(motor<<8u)|host,true);
}
void advance(unsigned us) { dwt.CYCCNT+=us*168u; }
unsigned pending(unsigned bus) {
    unsigned n=0;
    for (auto& box:regs(bus).sTxMailBox) if (box.TIR&CAN_TI0R_TXRQ) ++n;
    return n;
}
void expectStop(unsigned bus) {
    assert(pending(bus)==1 && !pending(3-bus));
    for (auto& box:regs(bus).sTxMailBox) if (box.TIR&CAN_TI0R_TXRQ) {
        assert(box.TDTR==8 && box.TDLR==0 && box.TDHR==0);
        if (bus==1) assert((box.TIR>>3u)==0x0400fd0bu && (box.TIR&CAN_TI0R_IDE));
        else assert((box.TIR>>21u)==0x200u && !(box.TIR&CAN_TI0R_IDE));
    }
}
void service(std::uint32_t seq) {
    balancing_f446_bench_service(seq);
    // Simulate the mailbox ownership bit which hardware changes on TXRQ.
    for (unsigned bus:{1u,2u}) for (unsigned i=0;i<3;++i)
        if (regs(bus).sTxMailBox[i].TIR&CAN_TI0R_TXRQ)
            regs(bus).TSR.bits&=~(CAN_TSR_TME0<<i);
}
void finish(unsigned bus,bool sent=true) {
    auto& r=regs(bus);
    for (unsigned i=0;i<3;++i) if (r.sTxMailBox[i].TIR&CAN_TI0R_TXRQ) {
        r.sTxMailBox[i].TIR&=~CAN_TI0R_TXRQ;
        r.TSR.bits|=(CAN_TSR_TME0<<i)|((CAN_TSR_RQCP0|(sent?CAN_TSR_TXOK0:CAN_TSR_TERR0))<<(8*i));
    }
    assert(balancing_f446_bench_capture_irq(bus));
}
bool accepts(unsigned bank,std::uint32_t word) {
    const auto& f=filters[bank];
    const auto id=(f.FilterIdHigh<<16u)|f.FilterIdLow;
    const auto mask=(f.FilterMaskIdHigh<<16u)|f.FilterMaskIdLow;
    return f.FilterActivation==ENABLE && ((word^id)&mask)==0;
}
void testFilters(unsigned bus) {
    const auto first=bus==1?0u:14u;
    assert(filterCalls==14);
    for (unsigned i=first;i<first+14;++i) {
        assert(filters[i].SlaveStartFilterBank==14 && filters[i].FilterFIFOAssignment==CAN_FILTER_FIFO0);
        assert(filters[i].FilterActivation==(i<first+(bus==1?2u:1u)?ENABLE:DISABLE));
    }
    if (bus==1) {
        for (unsigned type:{2u,24u}) for(unsigned mode=0;mode<4;++mode) {
            const auto id=((type<<24u)|(mode<<22u)|(11u<<8u)|0xfdu)<<3u;
            const auto bank=type==2?0u:1u;
            assert(accepts(bank,id|4u));
            assert(!accepts(bank,id|6u) && !accepts(bank,id));
            assert(!accepts(bank,(id^8u)|4u) && !accepts(bank,(id^2048u)|4u));
        }
    } else for(unsigned id=0;id<=0x7ff;++id) {
        assert(accepts(14,id<<21u)==(id==0x204));
        assert(!accepts(14,(id<<21u)|2u));
        assert(!accepts(14,(id<<21u)|4u));
    }
}
}
TestDwtRegisters* DWT=&dwt;
TestCoreDebugRegisters* CoreDebug=&debug;
CAN_TypeDef* CAN1=&can1;
CAN_TypeDef* CAN2=&can2;
std::uint32_t SystemCoreClock=168000000;
std::uint32_t __get_PRIMASK(){return primask;}
void __disable_irq(){primask=1;}
void __enable_irq(){primask=0;}
void __set_PRIMASK(std::uint32_t v){primask=v;}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* h,const CAN_FilterTypeDef* f) {
    assert(h->State==HAL_CAN_STATE_READY && (f->FilterBank<14)==(index(h)==0));
    ++filterCalls;filters[f->FilterBank]=*f;
    return int(f->FilterBank)==failFilter?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h) {
    ++startCalls;h->State=HAL_CAN_STATE_LISTENING;
    return failStart?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* h) {
    ++stopCalls;assert(!notifications[index(h)]);
    if(failRollback)return HAL_ERROR;
    h->State=HAL_CAN_STATE_READY;return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* h,std::uint32_t mask) {
    auto i=index(h);notifications[i]=mask;
    // IRQ during activation has a valid port but no published main-loop state.
    assert(!canBenchSummary());
    if(i==0)rs();else rx(2,0x204,false);
    assert(balancing_f446_bench_capture_irq(i+1));
    assert(!balancing_f446_bench_capture_irq(2-i));
    return failNotification?HAL_ERROR:HAL_OK;
}
void __HAL_CAN_DISABLE_IT(CAN_HandleTypeDef* h,std::uint32_t mask){notifications[index(h)]&=~mask;}

int main(int argc,char** argv) {
    assert(argc==2);
    const auto mode=argv[1];
    CAN_HandleTypeDef h1{CAN1,HAL_CAN_STATE_READY},h2{CAN2,HAL_CAN_STATE_READY};
    // Checked-in default is intentionally disabled. No filter, clock, or bus activity.
    assert(!balancing_f446_bench_start(&h1,&h2));
    assert(!startCanBench(&h2,&h1,rsCfg));
    assert(!canBenchSummary() && !balancing_f446_bench_capture_irq(1));
    service(1);assert(!filterCalls && !startCalls && !pending(1) && !pending(2));
    if(!std::strcmp(mode,"--iwdg")) {
        RCC->CSR=RCC_CSR_IWDGRSTF;
        balancing_f446_watchdog_capture_reset_flags();
        assert(startCanBench(&h1,&h2,wheelCfg));
        assert(startF446Watchdog({true,true,32,499})==BF446_WATCHDOG_STARTED);
        assert(iwdgInitCalls==1 && iwdgRefreshCalls==0 && resetClearCalls==1);
        service(1);assert(iwdgRefreshCalls==1);expectStop(2);
        advance(100);finish(2);assert(iwdgRefreshCalls==1); // IRQだけでは更新しない
        advance(1900);service(2);assert(iwdgRefreshCalls==2);
        service(3);assert(iwdgRefreshCalls==2); // DWT停止
        assert(canBenchSummary()->state==BenchState::Fault);
        assert(!bindBenchWatchdog({nullptr,[](void*) noexcept{return true;},true}));
        advance(4000);service(4);assert(iwdgRefreshCalls==2); // 欠落番号
        advance(2000);service(5);assert(iwdgRefreshCalls==3); // FAULT停止処理完了
        iwdgRefreshResult=HAL_ERROR;
        advance(2000);service(6);assert(iwdgRefreshCalls==4 && f446WatchdogSummary().refreshFailed);
        iwdgRefreshResult=HAL_OK;
        advance(2000);service(7);assert(iwdgRefreshCalls==4);
    } else if(!std::strcmp(mode,"--failures")) {
        dwt.CTRL=DWT_CTRL_NOCYCCNT_Msk;
        assert(!startCanBench(&h1,&h2,rsCfg));dwt.CTRL=0;
        failFilter=1;assert(!startCanBench(&h1,&h2,rsCfg) && !startCalls);
        failFilter=-1;failStart=true;assert(!startCanBench(&h1,&h2,rsCfg) && stopCalls==1);
        failStart=false;failNotification=true;assert(!startCanBench(&h1,&h2,rsCfg) && stopCalls==2);
        assert(!notifications[0] && !canBenchSummary() && !balancing_f446_bench_capture_irq(1));
        failNotification=false;assert(startCanBench(&h1,&h2,rsCfg));
        service(1);expectStop(1);
    } else if(!std::strcmp(mode,"--rollback")) {
        failNotification=failRollback=true;
        assert(!startCanBench(&h1,&h2,wheelCfg) && stopCalls==1 && !notifications[1]);
        failNotification=failRollback=false;h2.State=HAL_CAN_STATE_READY;
        assert(!startCanBench(&h1,&h2,wheelCfg) && startCalls==1);
        service(1);assert(!pending(1)&&!pending(2));
    } else {
        const bool gap=!std::strcmp(mode,"--gap");
        const unsigned bus=!std::strcmp(mode,"--rs")?1u:2u;
        // Start near the 32-bit DWT wrap and use an artificial verified configuration.
        dwt.CYCCNT=0xffff0000u;
        assert(startCanBench(&h1,&h2,bus==1?rsCfg:wheelCfg));
        assert(!startCanBench(&h1,&h2,bus==1?wheelCfg:rsCfg));
        assert(startCalls==1 && notifications[bus-1] && !notifications[2-bus]);
        assert((bus==1?h2:h1).State==HAL_CAN_STATE_READY);
        testFilters(bus);
        assert(!pending(bus));
        service(0xffffffffu);expectStop(bus);
        advance(100);finish(bus);
        advance(100);
        if(bus==1)rs();else rx(2,0x204,false);
        assert(balancing_f446_bench_capture_irq(bus));
        advance(1800);service(0);
        auto result=canBenchSummary();
        assert(result && result->state==BenchState::Idle && result->stopSent && result->feedbackFresh);
        if(bus==2 && pending(bus))expectStop(bus);
        if (gap) {
            if(pending(bus)){expectStop(bus);advance(100);finish(bus);}
            advance(3900);service(2); // one lost event, <6ms
            assert(canBenchSummary()->state==BenchState::Idle);
            if(pending(bus)){expectStop(bus);advance(100);finish(bus);}
            advance(5900);service(5); // 3 intervals, 6ms
            assert(canBenchSummary()->state==BenchState::Fault);
        } else {
            // A stalled DWT with a new TIM7 sequence latches the bench fault.
            service(1);
            assert(canBenchSummary()->state==BenchState::Fault);
            if(pending(bus)){expectStop(bus);advance(100);finish(bus);}
            advance(1900);service(2);
            if(pending(bus))expectStop(bus);
            regs(bus).RF0R.flags=CAN_RF0R_FOVR0;regs(bus).ESR=CAN_ESR_BOFF;
            assert(balancing_f446_bench_capture_irq(bus));
            advance(2000);service(3);
            assert(canBenchSummary()->state==BenchState::Fault && canBenchSummary()->busOff);
        }
        if(pending(bus))expectStop(bus);
        assert(!pending(3-bus));
    }
    std::puts("PASS: isolated bench startup, IRQ, rollback and zero-only service");
}
