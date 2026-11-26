#include "balancing/bench/MotorBench.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
#include <limits>

namespace { std::uint32_t primask{}; }
std::uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask=1; }
void __enable_irq() { primask=0; }

namespace {
using namespace balancing;
struct Fixture {
    CAN_TypeDef regs{};
    CAN_HandleTypeDef handle{&regs,HAL_CAN_STATE_LISTENING};
    CanPort port{handle};
    MotorBench bench;
    explicit Fixture(MotorBenchConfig cfg):bench{port,cfg} {}
    unsigned pending() const {
        unsigned n=0;
        for (const auto& box:regs.sTxMailBox) if (box.TIR&CAN_TI0R_TXRQ) ++n;
        return n;
    }
    unsigned mailbox() const {
        for (unsigned i=0;i<3;++i) if (regs.sTxMailBox[i].TIR&CAN_TI0R_TXRQ) return i;
        assert(false); return 0;
    }
    void expectRs(unsigned type) const {
        assert(pending()==1);
        const auto& m=regs.sTxMailBox[mailbox()];
        assert((m.TIR&CAN_TI0R_IDE) && (m.TIR>>27u)==type && m.TDTR==8u);
        assert(((m.TIR>>3u)&0xffu)==1u);
    }
    void expectWheel(bool nonzero) const {
        assert(pending()==1);
        const auto& m=regs.sTxMailBox[mailbox()];
        assert(!(m.TIR&CAN_TI0R_IDE) && (m.TIR>>21u)==0x200u && m.TDTR==8u);
        // ID3だけを駆動する人工試験。他3チャネルは必ずゼロ。
        assert(m.TDLR==0u && (m.TDHR&0xffff0000u)==0u);
        assert((m.TDHR!=0u)==nonzero);
    }
    void finish(std::uint64_t at,std::uint32_t flags=CAN_TSR_TXOK0) {
        const auto i=mailbox();
        regs.TSR.bits&=~(CAN_TSR_TME0<<i);
        regs.sTxMailBox[i].TIR&=~CAN_TI0R_TXRQ;
        regs.TSR.bits|=(CAN_TSR_TME0<<i)|((CAN_TSR_RQCP0|flags)<<(8*i));
        port.captureInterrupt(at);
    }
    void rx(std::uint32_t id,bool extended,std::uint64_t at,unsigned dlc=8,
            std::uint32_t low=0x00800080u,std::uint32_t high=0x2c010080u) {
        assert(regs.RF0R.count<3);
        regs.sFIFOMailBox[regs.RF0R.count++]={extended?(id<<3u)|CAN_RI0R_IDE:id<<21u,
                                            dlc,low,high};
        port.captureInterrupt(at);
    }
    void rs(std::uint64_t at,unsigned mode,unsigned type=2,unsigned motor=1) {
        rx((type<<24u)|(mode<<22u)|(motor<<8u)|0xfdu,true,at);
    }
    void wheel(std::uint64_t at,unsigned id=3,unsigned dlc=8) {
        rx(0x200u+id,false,at,dlc,0,0);
    }
    void poll(std::uint64_t now) {
        bench.poll(now);
        // 実機ではTXRQでTMEがクリアされる。レジスタスタブにも反映する。
        for (unsigned i=0;i<3;++i)
            if (regs.sTxMailBox[i].TIR&CAN_TI0R_TXRQ) regs.TSR.bits&=~(CAN_TSR_TME0<<i);
        assert(pending()<=1);
    }
    BenchState state(std::uint64_t now) const { return bench.summary(now).state; }
    void bootRs() {
        poll(0);expectRs(4);assert(!bench.requestPulse(0.2f,0));
        finish(1);rs(2,0);poll(2);assert(state(2)==BenchState::Idle);
        assert(bench.summary(2).stopSent);
    }
    void bootWheel() {
        poll(0);expectWheel(false);finish(1);wheel(2);poll(2);
        assert(state(2)==BenchState::Idle && pending()==0);
    }
};
constexpr MotorBenchConfig rsCfg{BenchAxis::Rs06,1,0xfd,0.3f,8,10000,true};
constexpr MotorBenchConfig wheelCfg{BenchAxis::C620,3,0,0.25f,8,10000,true};

void invalidConfig()
{
    Fixture disabled{{}};
    disabled.poll(0);assert(disabled.pending()==0 && disabled.state(0)==BenchState::Disabled);
    for (auto cfg:{MotorBenchConfig{BenchAxis::Rs06,1,0xfd,0.6f,10,10000,true},
                  MotorBenchConfig{BenchAxis::C620,5,0,0.2f,10,10000,true},
                  MotorBenchConfig{BenchAxis::Rs06,1,0xfd,0.2f,201,10000,true},
                  MotorBenchConfig{BenchAxis::Rs06,1,0xfd,0.2f,10,0,true}}) {
        Fixture f{cfg};f.poll(0);
        assert(f.state(0)==BenchState::Disabled && !f.pending());
    }
}
void rsPulse()
{
    Fixture f{rsCfg};
    // 構築では送信なし。初回停止と新しいType2 Resetが必要。
    assert(f.pending()==0);f.poll(0);f.expectRs(4);
    f.rs(1,0);f.poll(1);assert(!f.bench.requestPulse(0.2f,1));
    f.finish(2);f.poll(2);assert(f.state(2)==BenchState::Stopping);
    f.rs(3,0);f.poll(3);assert(f.state(3)==BenchState::Idle);
    assert(!f.bench.requestPulse(0.4f,3));
    assert(!f.bench.requestPulse(std::numeric_limits<float>::quiet_NaN(),3));
    assert(f.bench.requestPulse(0.2f,3));
    assert(!f.bench.requestPulse(0.1f,3));
    f.poll(3);f.expectRs(3);
    f.rs(4,2);f.poll(4);f.expectRs(3); // 受付前後ではなく送信完了後のType2が必要
    f.finish(5);f.poll(5);assert(f.state(5)==BenchState::Enabling && !f.pending());
    f.rs(6,2,24);f.poll(6);assert(f.state(6)==BenchState::Enabling && !f.pending());
    f.rs(7,2,2,2);f.poll(7);assert(f.state(7)==BenchState::Enabling && !f.pending());
    f.rs(8,2);f.poll(8);f.expectRs(1);assert(f.state(8)==BenchState::Pulsing);
    f.finish(9);f.poll(9);
    f.poll(4007);assert(!f.pending());f.poll(4008);f.expectRs(1);
    f.finish(4009);f.poll(4009);f.poll(8008);f.expectRs(4);
    f.finish(8009);f.poll(8009);assert(f.state(8009)==BenchState::Stopping);
    f.rs(8010,0);f.poll(8010);assert(f.state(8010)==BenchState::Idle);
}
void wheelPulseAndCancel()
{
    Fixture f{wheelCfg};f.bootWheel();
    assert(!f.bench.requestPulse(0.3f,2));
    assert(f.bench.requestPulse(-0.2f,2));
    f.poll(2);f.expectWheel(true);
    f.bench.cancel();assert(f.regs.TSR.aborts!=0);
    f.poll(3);f.expectWheel(true); // 旧送信の終端未確認中はゼロを追い越させない
    f.finish(4,0);f.poll(4);f.expectWheel(false);
    f.finish(5);f.poll(5);assert(f.state(5)==BenchState::Idle && f.bench.summary(5).stopSent);
    f.poll(2004);f.expectWheel(false);f.finish(2005);f.poll(2005);
    assert(f.bench.requestPulse(0.2f,2005));f.poll(2005);f.expectWheel(true);
    f.finish(2006);f.poll(2006);
    for (std::uint64_t at=4005;at<10005;at+=2000) {
        f.wheel(at);f.poll(at);f.expectWheel(true);f.finish(at+1);f.poll(at+1);
    }
    f.poll(10005);f.expectWheel(false);f.finish(10006);f.poll(10006);
    assert(f.state(10006)==BenchState::Idle);
}
void receiveAndDeadlineFaults()
{
    auto longPulse=wheelCfg;longPulse.pulseDurationMs=100;
    Fixture lost{longPulse};lost.bootWheel();
    assert(lost.bench.requestPulse(0.2f,2));
    lost.poll(2);lost.expectWheel(true);lost.finish(3);lost.poll(3);
    for (std::uint64_t at=2002;at<20002;at+=2000) {
        lost.wheel(at,2); // 別軸だけ届いても対象の鮮度を回復しない
        lost.poll(at);lost.expectWheel(true);lost.finish(at+1);lost.poll(at+1);
    }
    lost.poll(20002);lost.expectWheel(false);lost.finish(20003);lost.poll(20003);
    assert(lost.state(20003)==BenchState::Fault);
    lost.bench.cancel();assert(!lost.bench.requestPulse(0.1f,20003));

    Fixture timeout{longPulse};timeout.bootWheel();assert(timeout.bench.requestPulse(0.2f,2));
    timeout.poll(2);timeout.expectWheel(true);
    for (const auto at:{2002u,4002u,6002u,8002u,10002u}) { timeout.wheel(at);timeout.poll(at); }
    assert(timeout.state(10002)==BenchState::Fault && timeout.regs.TSR.aborts!=0);
    timeout.finish(10003);timeout.poll(10003);timeout.expectWheel(false);
    timeout.finish(10004);timeout.poll(10004);assert(timeout.bench.summary(10004).stopSent);

    Fixture gap{wheelCfg};gap.bootWheel();assert(gap.bench.requestPulse(0.2f,2));
    gap.poll(2);gap.expectWheel(true);gap.finish(3);gap.poll(3);
    gap.wheel(6003);gap.poll(6003);gap.expectWheel(false);
    assert(gap.state(6003)==BenchState::Fault);

    Fixture busOff{wheelCfg};busOff.bootWheel();assert(busOff.bench.requestPulse(0.2f,2));
    busOff.poll(2);busOff.expectWheel(true);
    busOff.regs.ESR=CAN_ESR_BOFF;busOff.port.captureInterrupt(3);busOff.poll(3);
    assert(busOff.state(3)==BenchState::Fault && busOff.bench.summary(3).busOff);
}
void rsEnableTimeoutAndStopRetry()
{
    Fixture f{rsCfg};f.bootRs();assert(f.bench.requestPulse(0.2f,2));
    f.poll(2);f.expectRs(3);f.finish(3);f.poll(3);
    for (std::uint64_t at=2002;at<150002;at+=2000) {
        f.rs(at,0);f.poll(at);assert(f.state(at)==BenchState::Enabling && !f.pending());
    }
    f.poll(150002);f.expectRs(4);assert(f.state(150002)==BenchState::Fault);
    f.finish(150003,CAN_TSR_TERR0);f.poll(150003);f.expectRs(4);
    f.finish(150004);f.poll(150004);assert(f.bench.summary(150004).stopSent);
    f.rs(150005,0);f.poll(150005);assert(!f.bench.requestPulse(0.1f,150005));
}
}
int main()
{
    invalidConfig();rsPulse();wheelPulseAndCancel();receiveAndDeadlineFaults();rsEnableTimeoutAndStopRetry();
    std::puts("PASS: isolated C++ motor bench, real CAN scheduler/codec, handshake, pulse, abort, stop, faults");
}
