#include "balancing_f446_can1_bridge.h"
#include "balancing_app_bridge.h"
#include "balancing/app/ApplicationObservation.hpp"
#include "balancing/app/DriveRuntime.hpp"
#include "balancing/device/ImuObservation.hpp"
#include "balancing/device/F446Watchdog.hpp"
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
std::uint32_t mask{};
unsigned starts{},filterCalls{};
std::array<unsigned,2> notifications{};
unsigned index(CAN_HandleTypeDef* h){return h->Instance==&can1?0u:1u;}
CAN_TypeDef& reg(unsigned bus){return bus==1?can1:can2;}
void advance(unsigned us){dwt.CYCCNT+=168u*us;}
bool irq(unsigned bus){return bus==1?balancing_f446_can1_capture_irq():balancing_f446_can2_capture_irq();}
unsigned pending(unsigned bus,bool motorOnly=true){
    unsigned n=0;
    for (auto& m:reg(bus).sTxMailBox) if (m.TIR&CAN_TI0R_TXRQ) {
        // Validate all motor frames at the register boundary, not using the same Codec.
        if (bus==1) {
            assert(m.TIR&CAN_TI0R_IDE);
            const auto id=m.TIR>>3u;
            assert((id>>24u)==4u && ((id>>8u)&0xffu)==0x5a);
            assert((id&0xffu)>=11 && (id&0xffu)<=14);
            assert(m.TDTR==8 && m.TDLR==0 && m.TDHR==0);
            ++n;
        } else if (!(m.TIR&CAN_TI0R_IDE)) {
            assert((m.TIR>>21u)==0x200 && m.TDTR==8 && !m.TDLR && !m.TDHR);
            ++n;
        } else {
            // Diagnostic/remote response only; no power request frames.
            const auto id=m.TIR>>3u;
            assert(id==0x08804c06u || (id>=0x0cc04c01u && id<=0x0cc04c0au) ||
                (id>=0x04c04c20u && id<=0x04c04c22u));
            if(!motorOnly)++n;
        }
    }
    return n;
}
void own(){
    for(unsigned bus:{1u,2u}) for(unsigned i=0;i<3;++i)
        if(reg(bus).sTxMailBox[i].TIR&CAN_TI0R_TXRQ) reg(bus).TSR.bits&=~(CAN_TSR_TME0<<i);
    (void)pending(1);(void)pending(2,false);
}
void finish(unsigned bus,bool sent=true){
    auto& r=reg(bus);
    for(unsigned i=0;i<3;++i) if(r.sTxMailBox[i].TIR&CAN_TI0R_TXRQ){
        r.sTxMailBox[i].TIR&=~CAN_TI0R_TXRQ;
        r.TSR.bits|=(CAN_TSR_TME0<<i)|((CAN_TSR_RQCP0|(sent?CAN_TSR_TXOK0:CAN_TSR_TERR0))<<(8*i));
    }
    assert(irq(bus));
}
void resetReport(unsigned motor){
    const std::uint32_t id=(2u<<24u)|(motor<<8u)|0x5au;
    can1.sFIFOMailBox[0]={(id<<3u)|CAN_RI0R_IDE,8,0,0};
    can1.RF0R.count=1;
    assert(irq(1));
}
void rx2(std::uint32_t id,unsigned dlc,std::uint32_t low=0){
    can2.sFIFOMailBox[0]={(id<<3u)|CAN_RI0R_IDE,dlc,low,0};
    can2.RF0R.count=1;
    assert(irq(2));
}
void tick(unsigned seq,bool collect=true,unsigned dt=2000){
    advance(dt);balancing_app_begin_control_tick(seq);
    if(collect)balancing_f446_can_service();
    balancing_app_control_tick();own();
}
StopOutputSummary summary(){return applicationObservation()->stopOutput;}
}
TestDwtRegisters* DWT=&dwt;
TestCoreDebugRegisters* CoreDebug=&debug;
CAN_TypeDef* CAN1=&can1;
CAN_TypeDef* CAN2=&can2;
std::uint32_t SystemCoreClock=168000000u;
std::uint32_t __get_PRIMASK(){return mask;}
void __disable_irq(){mask=1;}
void __enable_irq(){mask=0;}
void __set_PRIMASK(std::uint32_t v){mask=v;}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* h,const CAN_FilterTypeDef* f){
    assert(h->State==HAL_CAN_STATE_READY && (f->FilterBank<14)==(index(h)==0));
    ++filterCalls;return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* h){++starts;h->State=HAL_CAN_STATE_LISTENING;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* h){h->State=HAL_CAN_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* h,std::uint32_t value){notifications[index(h)]=value;return HAL_OK;}
void __HAL_CAN_DISABLE_IT(CAN_HandleTypeDef* h,std::uint32_t value){notifications[index(h)]&=~value;}
ImuAttitude balancing::imuAttitude(std::uint64_t) noexcept{return {};}

int main(int argc,char** argv){
    assert(argc==2);
    const char* mode=argv[1];
    CAN_HandleTypeDef h1{CAN1,HAL_CAN_STATE_READY},h2{CAN2,HAL_CAN_STATE_READY};
    MotorObservationConfig cfg{{{11,12,13,14},0x5a},true,{}, {true,true}};
    if(!std::strcmp(mode,"--default")){
        balancing_app_init();
        assert(balancing_f446_can_start(&h1,&h2));
        for(unsigned i=1;i<10;++i){tick(i);balancing_app_poll();own();}
        assert(!summary().enabled && !pending(1) && !pending(2,false));
        assert(!startCanObservation(&h1,&h2,cfg,true));
        return 0;
    }
    const bool collision=!std::strcmp(mode,"--collision");
    Can2ObservationProfile profile{};
    profile.communication={200000,100000,20000,20,1000,4,collision?0xD001u:0xCA2u,false};
    profile.operatorConfig={100000,20000,2.0f,3.0f,0.01f,0.01f,1.0f,1.0f,true};
    unsigned watchdog=0;
    profile.watchdog={&watchdog,[](void* c) noexcept{++*static_cast<unsigned*>(c);return true;},true};
    assert(initializeCan2Observation(profile));
    if(collision){
        assert(!startCanObservation(&h1,&h2,cfg,true) && !starts && !filterCalls);
        return 0;
    }
    auto bad=cfg;bad.stopOutput.verified=false;
    assert(!startCanObservation(&h1,&h2,bad,true));
    bad=cfg;bad.rs06MappingVerified=false;
    assert(!startCanObservation(&h1,&h2,bad,true));
    bad=cfg;bad.rs06.motorIds[1]=11;
    assert(!startCanObservation(&h1,&h2,bad,true));
    bad=cfg;bad.rs06.motorIds[0]=0;
    assert(!startCanObservation(&h1,&h2,bad,true));
    assert(!startCanObservation(&h2,&h1,cfg,true));
    assert(!startCanObservation(&h1,&h2,cfg,false));
    assert(!starts&&!filterCalls);
    assert(startCanObservation(&h1,&h2,cfg,true));
    assert(!startCanObservation(&h1,&h2,cfg,true));
    assert(!startCanObservation(&h1,&h2,{},true));
    balancing_app_poll();own();assert(!pending(1)&&!pending(2));
    // No collection / no fresh TIM7 evidence must not generate output.
    balancing_app_control_tick();assert(!pending(1)&&!pending(2));
    tick(1);
    assert(pending(1)==3 && pending(2)==1 && watchdog==1);
    assert(summary().enabled && summary().zeroQueued==1 && summary().zeroAccepted==1 && !summary().zeroSent);
    if(!std::strcmp(mode,"--retry")){
        // Failed stops must not be counted as delivered, even with fresh Reset feedback.
        advance(100);finish(1,false);finish(2);
        for (unsigned id=11;id<=14;++id) resetReport(id);
        tick(2);
        assert(pending(1)==3 && summary().rs06[0].stopPending &&
               summary().rs06[0].stopTxFailed && !summary().rs06[0].disableConfirmed);
        assert(applicationObservation()->cycle->sequence.state==RobotState::Fault);
        // A terminal for the wrong bus / old zero token cannot change its count.
        drive_runtime::onTxTerminal(CanBus::Can1,{2,0xD001u},CanTxOutcome::Sent,*observationClockUs());
        drive_runtime::onTxTerminal(CanBus::Can2,{1,0xD001u},CanTxOutcome::Sent,*observationClockUs());
        advance(100);finish(1);finish(2);
        tick(3);
        assert(pending(1)==1 && summary().zeroSent==2);
        advance(100);finish(1);finish(2);
        advance(100);
        for (unsigned id=11;id<=14;++id) resetReport(id);
        tick(4);
        assert(!pending(1));
        for (const auto& status:summary().rs06)
            assert(!status.stopPending && !status.stopTxFailed && status.disableConfirmed);
        assert(applicationObservation()->cycle->sequence.state==RobotState::Fault);
        return 0;
    }
    if(!std::strcmp(mode,"--timing")){
        // A new generation may be queued, but must not be accepted before the old terminal.
        tick(2);
        assert(summary().zeroQueued==2 && summary().zeroAccepted==1 && summary().zeroSent==0);
        tick(3);
        assert(summary().zeroQueued==3 && summary().zeroDropped==1);
        // Expiration at the exact deadline drops the latest pending frame.
        advance(2000);balancing_app_poll();own();
        finish(2);finish(1);
        tick(4);
        assert(summary().zeroSent==1 && summary().zeroDropped==2 && summary().zeroAccepted==2);
        const auto n=summary().zeroQueued;
        tick(4);assert(summary().zeroQueued==n);
        tick(6);assert(summary().zeroQueued==n);
        tick(7,false);assert(summary().zeroQueued==n);
        tick(8);assert(summary().zeroQueued==n); // invalid previous cycle, no backfill
        tick(9);assert(summary().zeroQueued==n+1);
        assert(watchdog<9);
        return 0;
    }
    advance(100);finish(1);finish(2);
    // Terminal IRQ alone cannot send or update IWDG.
    assert(watchdog==1 && !pending(1)&&!pending(2));
    tick(2);
    assert(pending(1)==1 && pending(2)==1 && summary().zeroSent==1);
    advance(100);finish(1);finish(2);
    tick(3);
    assert(!pending(1)&&pending(2)==1 && summary().zeroSent==2);
    for(const auto& status:summary().rs06) assert(!status.stopPending && !status.disableConfirmed);
    // User operation requests do not enable any motor; diagnostic may share one mailbox.
    rx2(0x0880c405u,3,0x00010100u);rx2(0x0880c402u,2,0x0200u);
    advance(100);finish(2);
    tick(4);balancing_app_poll();own();
    assert(pending(2)==1 && pending(2,false)<=2);
    assert(applicationObservation()->cycle->sequence.desiredMotorState==DesiredMotorState::StopAll);
    if(!std::strcmp(mode,"--faults")){
        advance(100);finish(2,false);
        tick(5);
        assert(summary().zeroFailed==1);
        assert(summary().outputFaults==FaultFlags::ExternalFault);
        assert(applicationObservation()->cycle->sequence.state==RobotState::Fault);
        // Bus-off and RX loss are observed by the existing device service, not hidden by zeros.
        can2.ESR=CAN_ESR_BOFF;can1.RF0R.flags=CAN_RF0R_FOVR0;
        assert(irq(1)&&irq(2));
        tick(6);
        assert(applicationObservation()->motors->can2BusOff);
        assert(applicationObservation()->fault);
        can2.ESR=0;
        assert(irq(2));
        advance(100);finish(2);
        tick(7);
        assert(!applicationObservation()->motors->can2BusOff);
    }
    assert(!pending(1));
    assert(applicationObservation()->cycle->sequence.desiredMotorState==DesiredMotorState::StopAll);
    assert(!applicationObservation()->cycle->motorCommandPreview);
    std::puts("PASS: production stop-only runtime, isolated from SIL");
}
