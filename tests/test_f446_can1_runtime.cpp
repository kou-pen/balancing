#include "balancing_f446_can1_bridge.h"
#include "balancing/communication/can/Can2FilterPlan.hpp"
#include "balancing_app_bridge.h"
#include "balancing/app/ApplicationObservation.hpp"
#include "sil/DriveSil.hpp"
#include "balancing/device/ImuObservation.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <array>
#include <cmath>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr auto notifications = CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING |
                               CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR;
CAN_TypeDef can1{};
CAN_TypeDef can2{};
TestDwtRegisters dwt{};
TestCoreDebugRegisters debug{};
std::uint32_t primask{};
std::array<std::uint32_t, 2> enabledNotifications{};
std::array<CAN_FilterTypeDef, 28> installedFilters{};
std::size_t filterCount{};
std::size_t startCount{};
std::size_t stopCount{};
int filterFailsBank=-1;
int startFailsBus = -1;
bool startTimeout{};
int irqEnableFailsBus = -1;
bool expectSelfDefinedCan2{};

void pushReceive(unsigned motor = 11, unsigned host = 0x5a)
{
    // Type 2の運転モード報告。実機確認前のためfixtureのノードIDを使う。
    const std::uint32_t frameId = (2u << 24u) | (2u << 22u) | (motor << 8u) | host;
    can1.sFIFOMailBox[0] = {(frameId << 3u) | CAN_RI0R_IDE, 8u, 0u, 0u};
    can1.RF0R.count = 1u;
}

void pushWheel(std::uint32_t id, std::uint32_t dlc = 8u, unsigned raw = 0)
{
    can2.sFIFOMailBox[0] = {id << 21u, dlc, (raw >> 8u) | ((raw & 0xffu) << 8u), 0u};
    can2.RF0R.count = 1u;
}

std::size_t busIndex(const CAN_HandleTypeDef* handle)
{
    assert(handle->Instance == &can1 || handle->Instance == &can2);
    return handle->Instance == &can1 ? 0u : 1u;
}

// bxCAN 32bit Listは二つの照合ワードを完全一致で比較する。
// このホスト模型はHALレジスタ書込みの検査とは独立し、実機フィルタの代用ではない。
bool listAccepts(const balancing::Can2FilterPlan& plan,std::uint32_t word)
{
    for (std::size_t i=0;i<plan.count;++i) {
        const auto& pair=plan.entries[i];
        if (pair.first==word || pair.second==word) return true;
    }
    return false;
}

void testCan2FilterPlan()
{
    const auto motor=balancing::makeCan2FilterPlan(false);
    const auto full=balancing::makeCan2FilterPlan(true);
    assert(motor.count==1 && full.count==7 && full.entries.size()==7);
    assert(full.entries[0].first==0x40200000u && full.entries[0].second==0x40400000u);
    assert(motor.entries[0].first==full.entries[0].first &&
           motor.entries[0].second==full.entries[0].second);
    for (std::size_t i=1;i<motor.entries.size();++i)
        assert(motor.entries[i].first==0 && motor.entries[i].second==0);

    // 全標準ID・RTRを走査。拡張IDの同じ数値や同じ上位11bitも別物。
    for (std::uint32_t id=0;id<=0x7ffu;++id) {
        const bool expected=id==0x201u || id==0x202u;
        for (const auto& plan : {motor,full}) {
            assert(listAccepts(plan,id<<21u)==expected);
            assert(!listAccepts(plan,(id<<21u)|2u));
            assert(!listAccepts(plan,(id<<3u)|4u));
            assert(!listAccepts(plan,(id<<21u)|4u));
        }
    }

    struct Case { std::uint32_t id; bool receive; };
    // 独立した仕様表の35個から、F446の12受信IDだけを選択する。
    const Case cases[]{
        {0x00047c01u,false},{0x0004bc01u,true},{0x0004fc01u,true},
        {0x08404801u,false},{0x0840c801u,false},{0x04404802u,false},{0x0440c802u,false},
        {0x0840bc03u,true},{0x08408404u,true},{0x08408c04u,false},
        {0x0880c401u,true},{0x0880c402u,true},{0x0480c403u,true},{0x0880c404u,true},
        {0x0880c405u,true},{0x08804c06u,false},
        {0x0cc04c01u,false},{0x0cc04c02u,false},{0x0cc04c03u,false},{0x0cc04c04u,false},
        {0x0cc04c05u,false},{0x0cc04c06u,false},{0x0cc04c07u,false},{0x0cc04c08u,false},
        {0x0cc04c09u,false},{0x0cc04c0au,false},
        {0x04c04c20u,false},{0x04c04c21u,false},{0x04c04c22u,false},
        {0x0cc0c480u,true},{0x0cc0c481u,true},{0x0cc04c82u,false},
        {0x0d00c401u,true},{0x0d004c02u,false},{0x0d004c03u,false}
    };
    unsigned expectedCount=0;
    for (const auto& c:cases) {
        const auto word=(c.id<<3u)|4u;
        assert(!listAccepts(motor,word));
        assert(listAccepts(full,word)==c.receive);
        assert(!listAccepts(full,word|2u));
        if (c.receive) {
            ++expectedCount;
            // 一ビット変異が別の正規受信IDになる場合だけ通過を許す。
            for (unsigned bit=0;bit<29;++bit) {
                const auto altered=c.id^(1u<<bit);
                bool expected=false;
                for (const auto& other:cases)
                    if (other.receive && other.id==altered) expected=true;
                assert(listAccepts(full,(altered<<3u)|4u)==expected);
            }
        }
    }
    assert(expectedCount==12);
    // 全フィルタワードを逆引きし、余計なID・重複がないことも確認する。
    for (std::size_t i=1;i<full.count;++i) {
        for (auto word:{full.entries[i].first,full.entries[i].second}) {
            assert((word&7u)==4u);
            unsigned matches=0,duplicates=0;
            for (const auto& c:cases) if (c.receive && c.id==(word>>3u)) ++matches;
            for (const auto& bank:full.entries)
                duplicates+=(bank.first==word)+(bank.second==word);
            assert(matches==1 && duplicates==1);
        }
    }
}

} // namespace

TestDwtRegisters* DWT = &dwt;
TestCoreDebugRegisters* CoreDebug = &debug;
CAN_TypeDef* CAN1 = &can1;
CAN_TypeDef* CAN2 = &can2;
std::uint32_t SystemCoreClock = 168000000u;

std::uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1u; }
void __enable_irq() { primask = 0u; }
void __set_PRIMASK(std::uint32_t value) { primask = value; }

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* handle, const CAN_FilterTypeDef* filter)
{
    assert(handle->State == HAL_CAN_STATE_READY);
    assert((filter->FilterBank < 14u) == (busIndex(handle) == 0u));
    assert(filter->FilterBank < installedFilters.size());
    installedFilters[filter->FilterBank] = *filter;
    ++filterCount;
    return static_cast<int>(filter->FilterBank)==filterFailsBank ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* handle)
{
    assert(handle->State == HAL_CAN_STATE_READY && enabledNotifications[busIndex(handle)] == 0u);
    assert(installedFilters[14].FilterActivation == ENABLE);
    for (unsigned i=15;i<28;++i)
        assert(installedFilters[i].FilterActivation == (expectSelfDefinedCan2 && i<=20 ? ENABLE : DISABLE));
    ++startCount;
    if (static_cast<int>(busIndex(handle)) == startFailsBus) {
        if (startTimeout) handle->State = HAL_CAN_STATE_ERROR;
        return HAL_ERROR;
    }
    handle->State = HAL_CAN_STATE_LISTENING;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* handle)
{
    assert(enabledNotifications[busIndex(handle)] == 0u);
    ++stopCount;
    if (handle->State != HAL_CAN_STATE_LISTENING) return HAL_ERROR;
    handle->State = HAL_CAN_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* handle, std::uint32_t value)
{
    assert(handle->State == HAL_CAN_STATE_LISTENING && value == notifications);
    enabledNotifications[busIndex(handle)] = value;
    if (static_cast<int>(busIndex(handle)) == irqEnableFailsBus) {
        return HAL_ERROR;
    }

    // CANが通知有効化直後に割り込む場合にも、ポートは構築済み。
    BF446Can1Snapshot unpublished1{};
    BF446Can2Snapshot unpublished2{};
    assert(!balancing_f446_can1_snapshot(&unpublished1));
    assert(!balancing_f446_can2_snapshot(&unpublished2));
    if (busIndex(handle) == 0u) {
        pushReceive();
        assert(balancing_f446_can1_capture_irq());
    } else {
        pushWheel(0x201u);
        assert(balancing_f446_can2_capture_irq());
    }
    return HAL_OK;
}

void __HAL_CAN_DISABLE_IT(CAN_HandleTypeDef* handle, std::uint32_t value)
{
    enabledNotifications[busIndex(handle)] &= ~value;
}

namespace { bool silImu=false; }
balancing::ImuAttitude balancing::imuAttitude(std::uint64_t atUs) noexcept
{ return silImu ? ImuAttitude{atUs,0.1f,0,0,0,0,true,true}:ImuAttitude{}; }

int main(int argc, char** argv)
{
    testCan2FilterPlan();
    CAN_HandleTypeDef handle{CAN1, HAL_CAN_STATE_READY};
    CAN_HandleTypeDef handle2{CAN2, HAL_CAN_STATE_READY};
    BF446Can1Snapshot snapshot{};
    BF446Can2Snapshot wheels{};
    if (argc == 2 && std::strcmp(argv[1], "--rollback-failure") == 0) {
        // 実HALのStartタイムアウトはERRORへ遷移し、Stopが拒否される。
        // 復帰不能のCAN2を正常と公開せず、CAN1は停止し、再起動も拒否する。
        startFailsBus = 1;
        startTimeout = true;
        assert(!balancing_f446_can_start(&handle, &handle2));
        assert(handle.State == HAL_CAN_STATE_READY);
        assert(handle2.State == HAL_CAN_STATE_ERROR);
        assert(stopCount == 2u);
        assert(!balancing_f446_can1_snapshot(&snapshot));
        assert(!balancing_f446_can2_snapshot(&wheels));
        assert(!balancing_f446_can1_capture_irq() && !balancing_f446_can2_capture_irq());
        assert(enabledNotifications[0] == 0u && enabledNotifications[1] == 0u);
        // 外からREADYへ変更されても、不明なハード状態で起動し直さない。
        handle2.State = HAL_CAN_STATE_READY;
        startFailsBus = -1;
        const auto attempts = startCount;
        assert(!balancing_f446_can_start(&handle, &handle2));
        assert(startCount == attempts);
        std::puts("PASS: failed CAN2 rollback is latched");
        return 0;
    }

    if (argc==2 && std::strcmp(argv[1],"--usb-input")==0) {
        using namespace balancing;
        // 人工の4バイト形式。実DualSenseの配置を表さない。
        const DualSenseConfig cfg{{1,4,{1,10,128,240,true,2},{2,0,127,255,false,3},
                                     {3,1},{3,2},true},300000,1000000,200000};
        DualSenseHidCodec codec{cfg.hid};
        std::array<std::uint8_t,4> report{1,128,127,0};
        assert(codec.enabled() && codec.decode(report.data(),4)->speedMps==0);
        report[1]=10; report[2]=255;
        auto decoded=codec.decode(report.data(),4);
        assert(decoded && decoded->speedMps==2 && decoded->yawRadS==3);
        report[1]=9; assert(!codec.decode(report.data(),4));
        report={1,128,127,0};
        assert(!codec.decode(report.data(),3) && !codec.decode(nullptr,4));
        report[0]=2; assert(!codec.decode(report.data(),4)); report[0]=1;
        auto bad=cfg.hid; bad.circle=bad.options;
        assert(!DualSenseHidCodec{bad}.enabled());
        RemoteControlService remote{{300000,200000,2,3,0.01f,0.01f,10,10,true}};
        DualSenseService usb{remote,cfg};
        assert(usb.connect(1,0));
        std::uint64_t serial=0;
        const auto send=[&](std::uint64_t t,unsigned buttons) {
            report[3]=static_cast<std::uint8_t>(buttons);
            assert(usb.receive(1,++serial,report.data(),4,t,t));
            return usb.takeEvents(t);
        };
        // 接続前から押したままのOPTIONSでは、1秒以上経過しても発火しない。
        for (std::uint64_t t=100000;t<=1200000;t+=100000) assert(!send(t,1).arm);
        assert(!send(1300000,0).arm);
        for (std::uint64_t t=1400000;t<2400000;t+=100000) assert(!send(t,1).arm);
        assert(send(2400000,1).arm);
        assert(!send(2500000,1).arm);
        assert(!usb.receive(1,serial,report.data(),4,2600000,2600000));
        assert(!usb.takeEvents(2900000).arm);
        assert(send(3000000,0).arm==false);
        // 切替後はUSB ARM無効、停止ボタンだけは有効。
        assert(usb.selectSource(B_OPERATOR_CAN_REMOTE,RobotState::Disarmed,3000000));
        assert(!usb.selectSource(B_OPERATOR_USB_DUALSENSE,RobotState::Balancing,3000000));
        for (std::uint64_t t=3100000;t<=4100000;t+=100000) assert(!send(t,1).arm);
        assert(!send(4200000,0).disarm);
        assert(!send(4300000,2).disarm); assert(!send(4400000,2).disarm);
        assert(send(4500000,2).disarm);
        assert(!send(4600000,2).disarm);
        assert(usb.disconnect(1,4700000));
        assert(!usb.receive(1,++serial,report.data(),4,4800000,4800000));
        assert(!usb.connect(1,4800000) && usb.connect(2,4800000));
        assert(!usb.receive(1,++serial,report.data(),4,4800100,4800100));
        // 転送欠落と不正報告で長押しを再開しない。
        report[3]=0; assert(usb.receive(2,1,report.data(),4,4900000,4900000));
        assert(usb.selectSource(B_OPERATOR_USB_DUALSENSE,RobotState::Disarmed,4900000));
        assert(usb.receive(2,2,report.data(),4,5000000,5000000));
        report[3]=1;
        assert(usb.receive(2,3,report.data(),4,5100000,5100000));
        assert(usb.receive(2,5,report.data(),4,5200000,5200000));
        for (std::uint64_t i=6;i<18;++i) {
            auto at=5200000+(i-5)*100000;
            assert(usb.receive(2,i,report.data(),4,at,at));
            assert(!usb.takeEvents(at).arm);
        }
        assert(!usb.receive(2,18,report.data(),3,6500000,6500000));
        assert(!usb.takeEvents(6500000).arm);
        assert(remote.update(RobotState::Balancing,6500000)->waiting_neutral);
        std::puts("PASS: USB report validation, generation, continuous holds, source/stop priority");
        return 0;
    }
    if (argc==2 && std::strncmp(argv[1],"--drive-sil",11)==0) {
        using namespace balancing;
        using namespace balancing::protocol;
        const bool usbMode=std::strcmp(argv[1],"--drive-sil-usb")==0;
        const bool timeout=std::strcmp(argv[1],"--drive-sil-timeout")==0;
        const bool stale=std::strcmp(argv[1],"--drive-sil-stale")==0;
        const bool rsTimeout=std::strcmp(argv[1],"--drive-sil-rs-timeout")==0;
        const bool txError=std::strcmp(argv[1],"--drive-sil-tx-error")==0;
        const bool estop=std::strcmp(argv[1],"--drive-sil-estop")==0;
        const bool invalid=std::strcmp(argv[1],"--drive-sil-invalid")==0;
        const bool invalidModel=std::strcmp(argv[1],"--drive-sil-invalid-model")==0;
        const bool rxLoss=std::strcmp(argv[1],"--drive-sil-rx-loss")==0;
        const bool gap=std::strcmp(argv[1],"--drive-sil-gap")==0;
        std::array<DiagnosticSlot,4> faults{};
        DriveSilProfile p{};
        // これらは人工fixture。現物のCADやゲイン、送信期限、モータIDではない。
        p.communication.communication={200000,100000,20000,20,1000,4,0xCA2u,false};
        p.communication.operatorConfig={10000,2000,2,3,0.01f,0.01f,10,10,true};
        p.communication.faultSlots=faults.data(); p.communication.faultCapacity=faults.size();
        if (usbMode) p.communication.usb={{1,4,{1,0,128,255,false,2},{2,0,128,255,false,3},
                                          {3,1},{3,2},true},10000,4000,2000};
        for (auto& leg:p.estimator.legs) {
            leg.motor={{0,0},{0,0},{1,1},1,1,true};
            leg.fixture={0.2f,0.15f,0.15f,0.25f,0.25f,0.02f,0.01f,-1};
        }
        p.estimator.wheels={{{0.1f,10,1,true},{0.1f,10,-1,true}}};
        p.controller.lqr.verified=true;
        p.controller.lqr.max_position_error_m=0.2f;
        p.controller.lqr.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_PITCH]=2;
        p.controller.lqr.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_POSITION]=1;
        p.controller.lqr.gain[B_OUTPUT_HIP_TOTAL][B_STATE_PITCH]=3;
        p.controller.legPd={100,10,2,0.5f,4,true};
        p.controller.yawPi={3,4,2,true};
        p.controller.wheelTorqueLimitNm={5,5};
        for (auto& a:p.converter.rs06) a={1.5f,1,4,true};
        for (auto& a:p.converter.c620) a={0.4f,1,4,true};
        if (invalid) p.converter.c620[1].verified=false;
        if (invalidModel) p.controller.lqr.verified=false;
        p.arm={50000,true}; p.interlocks={true,true,true,true,true,true,MechanismSafety::Safe};
        p.tx={12000,6000};
        assert(initializeDriveSil(p));
        expectSelfDefinedCan2=true;
        silImu=true;
        MotorObservationConfig mc{{{11,12,13,14},0x5a},true,{{{4000,true},{4000,true}}}};
        assert(startCanObservation(&handle,&handle2,mc,true));
        const auto rx=[&](const CanFrame& f) {
            std::uint32_t low=0,high=0;
            for (unsigned i=0;i<4;++i) {
                low|=std::uint32_t{f.data[i]}<<(8*i);
                high|=std::uint32_t{f.data[i+4]}<<(8*i);
            }
            can2.sFIFOMailBox[0]={(f.id<<3u)|CAN_RI0R_IDE,f.dlc,low,high};
            can2.RF0R.count=1;
            assert(balancing_f446_can2_capture_irq());
        };
        const auto request=[&](RemoteRequestKind kind,std::uint16_t id,
                               std::optional<WireInputSource> source=std::nullopt) {
            rx(*encodeRemoteRequest({kind,id,source}));
        };
        const auto feedback=[&](bool running,unsigned wheel=1000) {
            // Type2角度=-pi/2付近、速度・トルク中心。4軸の新規報告をIRQ経由で供給。
            for (unsigned id=11;id<=14;++id) {
                const auto rawId=(2u<<24u)|((running?2u:0u)<<22u)|(id<<8u)|0x5au;
                can1.sFIFOMailBox[0]={(rawId<<3u)|CAN_RI0R_IDE,8u,0x0080ff6fu,0x2c010080u};
                can1.RF0R.count=1;
                assert(balancing_f446_can1_capture_irq());
            }
            pushWheel(0x201,8,wheel); assert(balancing_f446_can2_capture_irq());
            pushWheel(0x202,8,1000); assert(balancing_f446_can2_capture_irq());
            rx(*encodePowerStatus({WirePowerState::On,0,0,1}));
        };
        const auto pre=[&](std::uint64_t at,bool running,unsigned wheel=1000) {
            DWT->CYCCNT=static_cast<std::uint32_t>((at-20)*168u);
            feedback(running,wheel);
        };
        const auto tick=[&](std::uint64_t at) {
            DWT->CYCCNT=static_cast<std::uint32_t>(at*168u);
            balancing_f446_can_service();
            balancing_app_control_tick();
            assert(driveSilObservation());
            return *driveSilObservation();
        };
        struct Counts { unsigned stop{},enable{},torque{},zero{},current{},diagnostic{}; };
        auto classify=[](CAN_TypeDef& bus,unsigned i,Counts& c) {
            const auto& m=bus.sTxMailBox[i];
            if (&bus==CAN1) {
                const auto type=(m.TIR>>27u)&31u;
                if (type==4) ++c.stop;
                else if (type==3) ++c.enable;
                else { assert(type==1); ++c.torque; }
            } else if ((m.TIR&CAN_TI0R_IDE)==0) {
                assert((m.TIR>>21u)==0x200u && m.TDHR==0);
                if (m.TDLR==0) ++c.zero; else ++c.current;
            } else ++c.diagnostic;
        };
        const auto drain=[&](bool holdWheel=false,bool holdTorque=false) {
            Counts counts{};
            // 受付→物理送信完了を区別し、空きができた後の第4軸も回収。
            for (unsigned round=0;round<8;++round) {
                bool any=false;
                for (auto* bus:{CAN1,CAN2}) for (unsigned i=0;i<3;++i) {
                    auto& m=bus->sTxMailBox[i];
                    if (!(m.TIR&CAN_TI0R_TXRQ)) continue;
                    bus->TSR.bits &= ~(CAN_TSR_TME0<<i);
                    if (holdWheel && bus==CAN2 && !(m.TIR&CAN_TI0R_IDE)) continue;
                    if (holdTorque && bus==CAN1 && ((m.TIR>>27u)&31u)==1) continue;
                    classify(*bus,i,counts);
                    any=true;
                    m.TIR &= ~CAN_TI0R_TXRQ;
                    bus->TSR.bits |= (CAN_TSR_TME0<<i)|((CAN_TSR_RQCP0|CAN_TSR_TXOK0)<<(8*i));
                }
                if (!any) break;
                DWT->CYCCNT+=168u;
                assert(balancing_f446_can1_capture_irq()); assert(balancing_f446_can2_capture_irq());
                balancing_f446_can_service(); balancing_app_poll();
            }
            return counts;
        };
        if (usbMode) {
            auto* usb=applicationDualSense();
            assert(usb && usb->connect(1,0));
            std::uint64_t serial=0;
            auto report=[&](std::uint64_t at,unsigned buttons,unsigned speed=128) {
                std::array<std::uint8_t,4> d{1,static_cast<std::uint8_t>(speed),128,static_cast<std::uint8_t>(buttons)};
                assert(usb->receive(1,++serial,d.data(),d.size(),at-20,at-20));
            };
            pre(2000,false); report(2000,0); auto res=tick(2000); drain();
            pre(4000,false); report(4000,0); res=tick(4000); drain();
            // PiのARMはUSB選択中には通らない。
            pre(6000,false); report(6000,1); request(RemoteRequestKind::Arm,1);
            res=tick(6000); assert(res.control.sequence.state==RobotState::Disarmed); drain();
            pre(8000,false); report(8000,1); res=tick(8000); assert(res.control.sequence.state==RobotState::Disarmed); drain();
            pre(10000,false); report(10000,1); res=tick(10000);
            assert(res.control.sequence.state==RobotState::Arming); drain();
            pre(12000,true); report(12000,0); res=tick(12000); drain();
            pre(14000,true); report(14000,0); res=tick(14000); drain();
            assert(res.control.sequence.state==RobotState::Balancing);
            pre(16000,true); report(16000,0,150); res=tick(16000); drain();
            assert(res.control.appliedReference->forwardVelocityMps>0);
            // USB切断はDISARMではなく、既存操作目標の減速を使う。
            assert(usb->disconnect(1,17000));
            pre(18000,true); res=tick(18000); drain();
            assert(res.control.sequence.state==RobotState::Balancing &&
                   res.control.appliedReference->forwardVelocityMps==0);
            assert(usb->connect(2,19000)); serial=0;
            auto second=[&](std::uint64_t at,unsigned buttons) {
                std::array<std::uint8_t,4> d{1,128,128,static_cast<std::uint8_t>(buttons)};
                assert(usb->receive(2,++serial,d.data(),4,at-20,at-20));
            };
            pre(20000,true); second(20000,0); res=tick(20000); drain();
            pre(22000,true); second(22000,2); res=tick(22000); drain();
            pre(24000,true); second(24000,2); res=tick(24000);
            assert(res.control.sequence.state==RobotState::Disarmed && res.outputInhibited);
            auto c=drain(); assert(c.stop==4 && c.zero==1 && c.current==0);
            std::puts("PASS: USB report to Application ARM, motion, disconnect, DISARM in mock CAN");
            return 0;
        }
        pre(2000,false); request(RemoteRequestKind::SelectSource,1,WireInputSource::CanRemote);
        auto res=tick(2000); assert(res.control.sequence.state==RobotState::Disarmed);
        auto c=drain(); assert(c.stop==4 && c.zero==1 && c.torque==0);
        pre(4000,false); rx(*encodeMotionCommand({0,0}));
        res=tick(4000); assert(res.control.sequence.state==RobotState::Disarmed); drain();
        pre(6000,false); rx(*encodeMotionCommand({0,0}));
        res=tick(6000); assert(applicationObservation()->operatorTarget->operator_ready); drain();
        pre(8000,false); request(RemoteRequestKind::Arm,2);
        res=tick(8000);
        if (invalid || invalidModel) {
            assert(res.control.sequence.state==RobotState::Disarmed && res.outputInhibited);
            c=drain(); assert(c.enable==0 && c.torque==0 && c.current==0);
            std::puts("PASS: invalid six-axis conversion or unverified gain prevents SIL ARM");
            return 0;
        }
        assert(res.control.sequence.state==RobotState::Arming && res.outputInhibited);
        c=drain(); assert(c.enable==3 && c.zero==1 && c.torque==0);
        // 先に3軸だけの送信が終わる。第4軸の旧RXで確認を成立させない。
        pre(10000,true);
        res=tick(10000); assert(res.control.sequence.state==RobotState::Arming);
        c=drain(); assert(c.enable==1 && c.zero==1);
        pre(12000,true);
        res=tick(12000); assert(res.control.sequence.state==RobotState::Balancing && res.outputInhibited);
        c=drain(); assert(c.zero==1 && c.torque==0);
        // 通常運転: 500Hzの車輪電流と250Hzの脚トルク。背景診断も同じschedulerを通る。
        unsigned rsCount=0, wheelCount=0;
        float previousRef=0;
        for (std::uint64_t at=14000;at<=24000;at+=2000) {
            pre(at,true);
            rx(*encodeMotionCommand({0.3f,0.1f}));
            res=tick(at);
            assert(res.control.sequence.state==RobotState::Balancing && !res.outputInhibited);
            assert(res.control.appliedReference && res.frames && res.frames->fixtureOnly);
            assert(res.control.appliedReference->forwardVelocityMps>0);
            if(at>14000) assert(res.control.appliedReference->positionM>previousRef);
            previousRef=res.control.appliedReference->positionM;
            c=drain();
            assert(c.torque==((at-14000)%4000==0?4u:0u) && c.current==1);
            rsCount+=c.torque; wheelCount+=c.current;
        }
        assert(rsCount==12 && wheelCount==6);
        if (rxLoss || gap) {
            if (rxLoss) {
                pre(26000,true);
                // 片車輪の壊れた報告で連続性が失われ、部分状態から指令を作らない。
                pushWheel(0x201,7,1000); assert(balancing_f446_can2_capture_irq());
                res=tick(26000);
                assert(res.control.estimateError==StateEstimateError::WheelFeedbackUnavailable);
            } else {
                // CANは継続して回収し、制御周期だけを6ms停止させる。
                for (const auto at:{26000u,28000u}) {
                    pre(at,true); balancing_f446_can_service();
                }
                pre(30000,true); res=tick(30000);
                assert(res.control.estimate);
            }
            assert(res.control.sequence.state==RobotState::Fault && res.outputInhibited);
            assert(applicationObservation()->fault->primaryCause==
                (gap?FaultFlags::ControlTiming:FaultFlags::InterlockLost));
            c=drain(); assert(c.current==0 && c.torque==0 && c.zero==1);
            std::puts("PASS: RX continuity loss or 6ms control gap stops SIL");
            return 0;
        }
        if (txError || estop) {
            pre(26000,true);
            if (estop) rx(*encodeEmergencyStop(NodeId::F303));
            res=tick(26000);
            if (txError) {
                assert(res.control.sequence.state==RobotState::Balancing);
                // 明示的なTXエラーを新規RXとは独立して注入。
                for (unsigned i=0;i<3;++i) {
                    auto& m=can2.sTxMailBox[i];
                    if (!(m.TIR&CAN_TI0R_TXRQ) || (m.TIR&CAN_TI0R_IDE)) continue;
                    m.TIR&=~CAN_TI0R_TXRQ;
                    can2.TSR.bits|=(CAN_TSR_TME0<<i)|((CAN_TSR_RQCP0|CAN_TSR_TERR0)<<(8*i));
                }
                DWT->CYCCNT+=168u;
                assert(balancing_f446_can2_capture_irq());
                balancing_f446_can_service(); balancing_app_poll();
                drain();
                pre(28000,true); res=tick(28000);
            }
            assert(res.control.sequence.state==RobotState::Fault && res.outputInhibited);
            assert(applicationObservation()->fault->primaryCause==
                (estop?FaultFlags::EmergencyStop:FaultFlags::ExternalFault));
            c=drain(); assert(c.current==0 && c.torque==0 && c.zero==1);
            std::puts("PASS: emergency stop or confirmed TX error inhibits normal SIL output");
            return 0;
        }
        if (rsTimeout) {
            // CAN2は正常完了を続け、RS06だけを滞留させる。
            for (std::uint64_t at=26000;at<=36000;at+=2000) {
                pre(at,true); res=tick(at);
                if (at<36000) assert(res.control.sequence.state==RobotState::Balancing);
                drain(false,true);
            }
            assert(res.control.sequence.state==RobotState::Fault && res.outputInhibited);
            assert(applicationObservation()->fault->primaryCause==FaultFlags::ExternalFault);
            assert(can1.TSR.aborts!=0u);
            std::puts("PASS: independent RS06 TX terminal deadline despite fresh RX");
            return 0;
        }
        if (timeout || stale) {
            // 新規のRXを維持し、C620の送信終端だけを失わせる。受信鮮度で代用しない。
            for (std::uint64_t at=26000;at<=32000;at+=2000) {
                pre(at,true); res=tick(at);
                if (at<32000) assert(res.control.sequence.state==RobotState::Balancing);
                drain(true,false);
                if (stale && at==28000) {
                    // 26000の旧世代を正常終端させても現在世代の鮮度にはならない。
                    for (unsigned i=0;i<3;++i) {
                        auto& m=can2.sTxMailBox[i];
                        if (!(m.TIR&CAN_TI0R_TXRQ) || (m.TIR&CAN_TI0R_IDE)) continue;
                        m.TIR&=~CAN_TI0R_TXRQ;
                        can2.TSR.bits|=(CAN_TSR_TME0<<i)|((CAN_TSR_RQCP0|CAN_TSR_TXOK0)<<(8*i));
                    }
                    DWT->CYCCNT+=168u;
                    assert(balancing_f446_can2_capture_irq());
                    balancing_f446_can_service(); balancing_app_poll();
                    drain(true,false);
                }
            }
            assert(res.control.sequence.state==RobotState::Fault && res.outputInhibited);
            assert(applicationObservation()->fault->primaryCause==FaultFlags::ExternalFault);
            assert(can2.TSR.aborts!=0u);
            std::puts("PASS: independent C620 TX terminal deadline despite fresh RX");
            return 0;
        }
        // 中立/通信断で速度が減速して0になり、現在位置基準へ復帰する。
        for (std::uint64_t at=26000;at<=64000;at+=2000) {
            pre(at,true,1000+static_cast<unsigned>((at-24000)/2000));
            res=tick(at); drain();
        }
        assert(res.control.appliedReference->forwardVelocityMps==0);
        assert(res.control.appliedReference->positionM==res.control.estimate->chassis.positionM);
        assert(applicationObservation()->operatorTarget->waiting_neutral);
        // 通信復帰でも非中立入力だけでは再開しない。2回の中立報告後だけ再開。
        pre(66000,true); rx(*encodeMotionCommand({0.3f,0.1f})); res=tick(66000); drain();
        assert(res.control.appliedReference->forwardVelocityMps==0);
        for (const auto at:{68000u,70000u}) {
            pre(at,true); rx(*encodeMotionCommand({0,0})); res=tick(at); drain();
        }
        assert(applicationObservation()->operatorTarget->operator_ready);
        pre(72000,true); rx(*encodeMotionCommand({0.3f,0.1f})); res=tick(72000); drain();
        assert(res.control.appliedReference->forwardVelocityMps>0);
        // 旧非ゼロフレームを保持してDISARM。中止要求だけで終端とみなさない。
        pre(74000,true); res=tick(74000); drain(true,true);
        pre(76000,true); request(RemoteRequestKind::Disarm,3);
        res=tick(76000);
        assert(res.control.sequence.state==RobotState::Disarmed && res.outputInhibited);
        assert(can1.TSR.aborts!=0u && can2.TSR.aborts!=0u);
        for (auto* bus:{CAN1,CAN2}) for (unsigned i=0;i<3;++i) {
            auto& m=bus->sTxMailBox[i];
            if (!(m.TIR&CAN_TI0R_TXRQ)) continue;
            if (bus==CAN1) assert(((m.TIR>>27u)&31u)==1u);
            // 旧周期を中止完了させ、後続のゼロ/停止を解放する。
            bus->TSR.bits &= ~(CAN_TSR_TME0<<i);
            m.TIR &= ~CAN_TI0R_TXRQ;
            bus->TSR.bits|=(CAN_TSR_TME0<<i)|(CAN_TSR_RQCP0<<(8*i));
        }
        DWT->CYCCNT+=168u;
        assert(balancing_f446_can1_capture_irq()); assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service(); balancing_app_poll();
        c=drain(); assert(c.torque==0 && c.current==0 && c.zero==1 && c.stop==4);
        pre(78000,false); res=tick(78000); c=drain();
        assert(res.control.sequence.state==RobotState::Disarmed && c.zero==1 && c.torque==0);
        std::puts("PASS: Application full SIL ARM, 250/500Hz, operation reference, abort/stop");
        return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "--watchdog") == 0) {
        using namespace balancing;
        // 単体: 番号周回、欠落、古い完了、未完了、逆行、更新失敗を区別する。
        CycleHealthMonitor monitor;
        unsigned refreshCount=0;
        const auto refresh=[](void* context) noexcept {
            ++*static_cast<unsigned*>(context);
            return true;
        };
        WatchdogPort disabled;
        WatchdogPort port{{&refreshCount,refresh,true}};
        const CycleTicket first{0xffffffffu,100};
        assert(monitor.begin(first));
        auto proof=monitor.complete(first,110,true);
        assert(proof && !disabled.refresh(*proof) && refreshCount==0);
        assert(port.refresh(*proof) && refreshCount==1);
        assert(!port.refresh(*proof) && port.summary().rejected==1);
        assert(!monitor.complete(first,120,true));
        const CycleTicket wrap{0,2100};
        assert(monitor.begin(wrap));
        assert(!monitor.complete(first,2101,true) && monitor.summary().inProgress);
        proof=monitor.complete(wrap,2110,true);
        assert(proof && port.refresh(*proof) && refreshCount==2);
        assert(!monitor.begin({0,2200}) && monitor.summary().duplicateEvents==1);
        assert(!monitor.begin({0xffffffffu,2200}) && monitor.summary().oldEvents==1);
        const CycleTicket gap{3,8100};
        assert(monitor.begin(gap));
        assert(!monitor.complete(gap,8110,true) && monitor.summary().missedEvents==2);
        const CycleTicket unfinished{4,10100};
        assert(monitor.begin(unfinished));
        const CycleTicket next{5,12100};
        assert(monitor.begin(next));
        assert(!monitor.complete(next,12110,true) && monitor.summary().incompleteCycles==1);
        assert(!monitor.begin({6,12000}) && monitor.summary().clockRegressions==1);
        const CycleTicket failed{6,14100};
        assert(monitor.begin(failed));
        assert(!monitor.complete(failed,14110,false));
        const CycleTicket reversed{7,16100};
        assert(monitor.begin(reversed));
        assert(!monitor.complete(reversed,16099,true));
        const CycleTicket recovered{8,18100};
        assert(monitor.begin(recovered));
        proof=monitor.complete(recovered,18110,true);
        assert(proof && port.refresh(*proof) && refreshCount==3);
        WatchdogPort bad{{nullptr,[](void*) noexcept { return false; },true}};
        assert(!bad.refresh(*proof) && bad.summary().failures==1);
        assert(!bad.refresh(*proof) && bad.summary().rejected==1);
        assert(!port.refresh({{9,18000},18001}));

        // Application実行経路: CAN回収前の開始証跡と処理完了を照合。
        refreshCount=0;
        Can2ObservationProfile profile{};
        profile.communication={200000,100000,20000,20,1000,4,0xCA2u,false};
        profile.operatorConfig={100000,20000,2.0f,3.0f,0.01f,0.01f,1.0f,1.0f,true};
        profile.watchdog={&refreshCount,refresh,true};
        assert(initializeCan2Observation(profile));
        balancing_app_begin_control_tick(1);
        balancing_app_control_tick(); // CAN時計・観測が未成立
        assert(refreshCount==0);
        expectSelfDefinedCan2=true;
        assert(startCanObservation(&handle,&handle2,{},true));
        const auto cycle=[&](std::uint32_t seq,bool collect=true) {
            DWT->CYCCNT+=168u*2000u;
            balancing_app_begin_control_tick(seq);
            if (collect) balancing_f446_can_service();
            balancing_app_control_tick();
        };
        cycle(1); assert(refreshCount==1);
        cycle(2); assert(refreshCount==2);
        for (unsigned i=0;i<10;++i) balancing_app_poll();
        assert(refreshCount==2);
        cycle(2); assert(refreshCount==2);
        cycle(5); assert(refreshCount==2);
        assert(applicationObservation()->cycleHealth.missedEvents==2);
        cycle(6,false); assert(refreshCount==2);
        assert(applicationObservation()->cycleHealth.incompleteCycles==1);
        cycle(7); assert(refreshCount==3);
        // 番号なしの旧APIは診断試験には使えるが更新証跡を発行しない。
        DWT->CYCCNT+=168u*2000u;
        balancing_f446_can_service(); balancing_app_control_tick();
        assert(refreshCount==3 && !applicationObservation()->cycleHealth.refreshEligible);
        // 非常停止でFAULTに遷移しても停止処理が完了すれば更新を継続する。
        can2.sFIFOMailBox[0]={(0x0004fc01u<<3u)|CAN_RI0R_IDE,0,0,0};
        can2.RF0R.count=1;
        assert(balancing_f446_can2_capture_irq());
        cycle(8);
        assert(applicationObservation()->fault->primaryCause==FaultFlags::EmergencyStop);
        assert(refreshCount==4);
        cycle(9); assert(refreshCount==5);
        // 開始だけで停止した周期はpollを繰り返しても更新されない。
        DWT->CYCCNT+=168u*2000u;
        balancing_app_begin_control_tick(10);
        for (unsigned i=0;i<10;++i) balancing_app_poll();
        assert(refreshCount==5 && applicationObservation()->cycleHealth.inProgress);
        cycle(11); assert(refreshCount==5);
        cycle(12); assert(refreshCount==6);
        assert(applicationObservation()->watchdog.successes==6);
        std::puts("PASS: numbered control completion, watchdog gating, FAULT processing and no background refresh");
        return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "--can2-services") == 0) {
        using namespace balancing;
        using namespace balancing::protocol;
        // 全値はホストfixture。未設定のまま自作フィルタだけを開くことはできない。
        assert(!startCanObservation(&handle,&handle2,{},true));
        assert(!applicationCan2Link());
        std::array<DiagnosticSlot,4> important{};
        Can2ObservationProfile profile{};
        profile.communication={200000,100000,20000,20,1000,4,0xCA2u,false};
        profile.operatorConfig={100000,20000,2.0f,3.0f,0.01f,0.01f,1.0f,1.0f,true};
        profile.faultSlots=important.data();
        profile.faultCapacity=important.size();
        assert(initializeCan2Observation(profile));
        expectSelfDefinedCan2=true;
        assert(startCanObservation(&handle,&handle2,{},true));
        assert(!startCanObservation(&handle,&handle2,{},false));
        const auto expected=makeCan2FilterPlan(true);
        for (unsigned i=0;i<expected.count;++i) {
            const auto& f=installedFilters[14+i];
            assert(((f.FilterIdHigh<<16u)|f.FilterIdLow)==expected.entries[i].first);
            assert(((f.FilterMaskIdHigh<<16u)|f.FilterMaskIdLow)==expected.entries[i].second);
        }
        const auto rx=[&](std::uint32_t id,unsigned dlc,std::uint32_t low=0,std::uint32_t high=0) {
            DWT->CYCCNT+=168u*100u;
            can2.sFIFOMailBox[0]={(id<<3u)|CAN_RI0R_IDE,dlc,low,high};
            can2.RF0R.count=1;
            assert(balancing_f446_can2_capture_irq());
        };
        const auto tick=[&] {
            DWT->CYCCNT+=168u*2000u;
            balancing_f446_can_service();
            balancing_app_control_tick();
            assert(applicationObservation()->timing.sample_valid);
        };
        const auto send=[&](std::uint32_t expectedId) {
            balancing_app_poll();
            // 診断/応答でモータ用の全メールボックスを占有しない。
            assert((can2.sTxMailBox[0].TIR&CAN_TI0R_TXRQ)!=0u);
            assert((can2.sTxMailBox[0].TIR>>3u)==expectedId);
            assert((can2.sTxMailBox[1].TIR&CAN_TI0R_TXRQ)==0u);
            assert((can2.sTxMailBox[2].TIR&CAN_TI0R_TXRQ)==0u);
            const auto low=can2.sTxMailBox[0].TDLR, high=can2.sTxMailBox[0].TDHR;
            can2.TSR.bits&=~CAN_TSR_TME0;
            can2.sTxMailBox[0].TIR&=~CAN_TI0R_TXRQ;
            can2.TSR.bits|=CAN_TSR_TME0|CAN_TSR_RQCP0|CAN_TSR_TXOK0;
            DWT->CYCCNT+=168u;
            assert(balancing_f446_can2_capture_irq());
            tick();
            return std::array<std::uint32_t,2>{low,high};
        };
        // 独立した固定IDとワイヤ値で、IRQからApplicationまで実配線以外を確認。
        rx(0x0840bc03,8,0x34128002,0x00000100);
        tick();
        auto obs=applicationObservation();
        assert(obs->power.ready && obs->power.latest->receivedAtUs==100);
        assert(obs->power.latest->wire->faultBits==0x1234);
        // USB選択中のPi ARMはUnauthorized。選択変更後も未検証パラメータでARM不可。
        rx(0x0880c402,2,0x0100);
        tick();
        auto response=send(0x08804c06);
        assert(response[0]==0x02000100 && (response[1]&0xffu)==5);
        rx(0x0880c405,3,0x00010200);
        tick();
        response=send(0x08804c06);
        assert(response[0]==0x05000200 && (response[1]&0xffu)==0);
        assert(applicationObservation()->operatorTarget->source==B_OPERATOR_CAN_REMOTE);
        rx(0x0880c402,2,0x0300);
        tick();
        response=send(0x08804c06);
        assert(response[0]==0x02000300 && (response[1]&0xffu)==3);
        // 同じ周期でARMとE-STOPが来てもFAULTが先。応答満杯と無関係。
        rx(0x0880c402,2,0x0400);
        rx(0x0004fc01,0);
        tick();
        obs=applicationObservation();
        assert(obs->cycle->sequence.state==RobotState::Fault);
        assert(obs->fault && obs->fault->primaryCause==FaultFlags::EmergencyStop);
        assert(obs->diagnostics.importantPending==1 && !obs->cycle->motorCommandPreview);
        // 実ApplicationのFaultRecordを3フレームへ配送し、終端まで保持。
        const auto first=send(0x04c04c20);
        const auto second=send(0x04c04c21);
        const auto third=send(0x04c04c22);
        assert((first[0]&0xffffu)==(second[0]&0xffffu));
        assert((second[0]&0xffffu)==(third[0]&0xffffu));
        assert(applicationObservation()->diagnostics.importantPending==0);
        for (const auto& mailbox:can1.sTxMailBox) assert(!(mailbox.TIR&CAN_TI0R_TXRQ));
        // 解除権限は非駆動profileでは未接続。再ARMもしない。
        rx(0x0880c404,2,0x0500);
        tick();
        assert(applicationObservation()->cycle->sequence.state==RobotState::Fault);
        std::puts("PASS: CAN2 opt-in filters, IRQ, application cycle, remote response and three fault frames");
        return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "--motor-observation") == 0) {
        using namespace balancing;
        MotorObservationConfig config{{{11,12,13,14},0x5a},true,{{{4000,true},{4000,true}}}};
        assert(!motorObservation() && !observationClockUs());
        assert(startCanObservation(&handle,&handle2,config));
        assert(!motorObservation());
        for (unsigned id = 12; id <= 14; ++id) {
            DWT->CYCCNT += 168;
            pushReceive(id);
            assert(balancing_f446_can1_capture_irq());
        }
        pushWheel(0x202);
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        auto snap = motorObservation();
        assert(snap && snap->rs06MappingVerified && snap->motors.sampledAtUs == 3);
        for (const auto& axis : snap->motors.rs06) assert(axis.latest && axis.fresh);
        for (const auto& axis : snap->motors.c620) assert(axis.latest && axis.fresh && axis.position);
        DWT->CYCCNT += 168000;
        pushWheel(0x201,8,3000);
        assert(balancing_f446_can2_capture_irq());
        pushWheel(0x201,8,6000);
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        snap = motorObservation();
        assert(snap->motors.c620[0].position->counts == 6000);
        assert(snap->motors.c620[0].latest->sequence == 3);
        DWT->CYCCNT += 168;
        pushReceive(11,0x33); // 異なる主機IDは既存報告を上書きしない
        assert(balancing_f446_can1_capture_irq());
        can2.ESR = CAN_ESR_BOFF;
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        const auto failed = motorObservation();
        assert(failed->can2BusOff && failed->observedFaults == FaultFlags::BusOff);
        assert(failed->motors.rs06[0].latest->receivedAtUs == 0);
        assert(!failed->motors.c620[0].position && !failed->motors.c620[1].position);
        assert(motorObservation()->observedFaults == FaultFlags::BusOff);
        DWT->CYCCNT += 168;
        can2.ESR = 0;
        assert(balancing_f446_can2_capture_irq());
        balancing_f446_can_service();
        snap = motorObservation();
        assert(!snap->can2BusOff && snap->observedFaults == FaultFlags::BusOff);
        assert(!acknowledgeMotorObservation(failed->generation));
        assert(acknowledgeMotorObservation(snap->generation));
        DWT->CYCCNT += 168u*30000u;
        balancing_f446_can_service();
        snap = motorObservation();
        assert(snap->observedFaults == FaultFlags::None);
        for (const auto& axis : snap->motors.rs06) assert(axis.latest && !axis.fresh);
        for (const auto& axis : snap->motors.c620) assert(axis.latest && !axis.fresh && !axis.position);
        for (const auto* bus : {CAN1,CAN2}) {
            for (const auto& mailbox : bus->sTxMailBox) assert((mailbox.TIR & CAN_TI0R_TXRQ)==0);
        }
        std::puts("PASS: real runtime receive-only six-axis observation");
        return 0;
    }

    assert(!balancing_f446_can_start(nullptr, &handle2));
    assert(!balancing_f446_can_start(&handle2, &handle));
    assert(!balancing_f446_can1_capture_irq());
    assert(!balancing_f446_can2_capture_irq());
    assert(!balancing_f446_can1_snapshot(&snapshot));
    assert(!balancing_f446_can2_snapshot(&wheels));
    balancing_f446_can_service();

    DWT->CTRL = DWT_CTRL_NOCYCCNT_Msk;
    assert(!balancing_f446_can_start(&handle, &handle2));
    DWT->CTRL = 0u;
    assert(filterCount == 0u && startCount == 0u);
    for (int bank : {14,15,27}) {
        filterFailsBank=bank;
        assert(!balancing_f446_can_start(&handle,&handle2) && startCount==0u);
        assert(!balancing_f446_can1_snapshot(&snapshot) && !balancing_f446_can2_snapshot(&wheels));
    }
    filterFailsBank=-1;
    // 古い有効設定が残っていてもCAN2所有バンクを明示的に無効化する。
    for (unsigned bank=15;bank<28;++bank) installedFilters[bank].FilterActivation=ENABLE;
    // CAN2の起動が失敗したら、先に成功したCAN1も停止する。
    startFailsBus = 1;
    assert(!balancing_f446_can_start(&handle, &handle2));
    assert(stopCount == 1u && handle.State == HAL_CAN_STATE_READY);
    startFailsBus = -1;
    irqEnableFailsBus = 1;
    assert(!balancing_f446_can_start(&handle, &handle2) && stopCount == 3u);
    assert(!balancing_f446_can1_capture_irq() && !balancing_f446_can2_capture_irq());
    assert(enabledNotifications[0] == 0u && enabledNotifications[1] == 0u);
    assert(!balancing_f446_can2_snapshot(&wheels));

    irqEnableFailsBus = -1;
    DWT->CYCCNT = 0xfffffff0u;
    assert(balancing_f446_can_start(&handle, &handle2));
    assert((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
    assert((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u);
    const auto count = startCount;
    assert(balancing_f446_can_start(&handle, &handle2) && startCount == count);
    assert(!balancing_f446_can_start(&handle2, &handle));
    assert(!balancing_f446_can1_snapshot(nullptr));
    assert(!balancing_f446_can2_snapshot(nullptr));
    for (std::size_t bank : {0u,1u}) {
        const auto& f=installedFilters[bank];
        assert(f.FilterBank==bank && f.SlaveStartFilterBank==14u);
        assert(f.FilterMode==CAN_FILTERMODE_IDMASK && f.FilterScale==CAN_FILTERSCALE_32BIT);
        assert(f.FilterActivation==ENABLE && f.FilterFIFOAssignment==CAN_FILTER_FIFO0);
        assert(f.FilterIdHigh==((bank==0?2u:24u)<<11u));
        assert(f.FilterIdLow==CAN_RI0R_IDE && f.FilterMaskIdHigh==0xf800u);
        assert(f.FilterMaskIdLow==(CAN_RI0R_IDE|CAN_RI0R_RTR));
    }
    for (std::size_t bank=14;bank<28;++bank) {
        const auto& f=installedFilters[bank];
        assert(f.FilterBank==bank && f.SlaveStartFilterBank==14u);
        assert(f.FilterMode==CAN_FILTERMODE_IDLIST && f.FilterScale==CAN_FILTERSCALE_32BIT);
        assert(f.FilterFIFOAssignment==CAN_FILTER_FIFO0);
        assert(f.FilterActivation==(bank==14?ENABLE:DISABLE));
        assert(f.FilterIdHigh==(bank==14?0x4020u:0u));
        assert(f.FilterMaskIdHigh==(bank==14?0x4040u:0u));
        assert(f.FilterIdLow==0u && f.FilterMaskIdLow==0u);
    }

    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 0u);
    assert(!snapshot.sampled);
    assert(balancing_f446_can2_snapshot(&wheels) && !wheels.sampled);
    balancing_app_init();
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 1u);
    assert(snapshot.last_rx_us == 0u);
    assert(snapshot.sampled && snapshot.sampled_at_us == 0u);
    const auto unconfigured = balancing::motorObservation();
    assert(unconfigured && !unconfigured->rs06MappingVerified);
    assert(unconfigured->observedFaults == balancing::FaultFlags::None);
    for (const auto& axis : unconfigured->motors.rs06) assert(!axis.latest);
    assert(unconfigured->motors.c620[0].latest && !unconfigured->motors.c620[0].position);
    assert(balancing_f446_can2_snapshot(&wheels) && wheels.sampled);
    assert(wheels.left_frames == 1u && wheels.right_frames == 0u);
    assert(wheels.sampled_at_us == snapshot.sampled_at_us);
    balancing_app_control_tick();
    BF446AppTimingSnapshot timing{};
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && !timing.elapsed_valid);
    const auto initialCycle=balancing::applicationObservation();
    assert(initialCycle->motors && initialCycle->cycle);
    assert(initialCycle->cycle->estimateError==balancing::StateEstimateError::InvalidPeriod);
    assert(initialCycle->cycle->sequence.state==balancing::RobotState::Disarmed);

    DWT->CYCCNT = 0x98u;  // 起動時のカウンタから168サイクル、ちょうど1 us。
    can1.ESR = CAN_ESR_BOFF;
    can1.RF0R.flags = CAN_RF0R_FOVR0;
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 1u);
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 2u);
    assert(snapshot.last_rx_us == 1u && snapshot.rx_loss_events == 1u);
    assert(snapshot.bus_off && snapshot.bus_off_events == 1u);
    assert(balancing_f446_can2_snapshot(&wheels));
    assert(!wheels.bus_off && wheels.rx_loss_events == 0u);
    assert(snapshot.sampled_at_us == snapshot.last_rx_us);
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 1u);
    const auto faultCycle=balancing::applicationObservation();
    assert(faultCycle->cycle->sequence.state==balancing::RobotState::Fault);
    assert(faultCycle->fault && faultCycle->fault->primaryCause==balancing::FaultFlags::BusOff);
    assert(!faultCycle->cycle->motorCommandPreview);

    DWT->CYCCNT = 0xffffff00u;
    balancing_f446_can_service();  // IRQがない区間を先にサンプルする。
    DWT->CYCCNT = 0x100u;
    can1.ESR = 0;
    pushReceive();
    assert(balancing_f446_can1_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_frames == 3u);
    assert(!snapshot.bus_off && snapshot.bus_off_events == 1u);
    assert(snapshot.last_rx_us == ((std::uint64_t{1} << 32u) + 0x110u) / 168u);
    assert(snapshot.sampled_at_us == snapshot.last_rx_us);
    const auto afterWrap = snapshot.sampled_at_us;
    // CAN2のIRQも同じ時計を読む。短いフレームは分類件数に含めない。
    DWT->CYCCNT += 168u;
    can2.ESR = CAN_ESR_BOFF;
    can2.RF0R.flags = CAN_RF0R_FOVR0;
    pushWheel(0x202u);
    primask = 1u;
    assert(balancing_f446_can2_capture_irq());
    assert(primask == 1u);
    primask = 0u;
    pushWheel(0x201u, 7u);
    assert(balancing_f446_can2_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can2_snapshot(&wheels));
    assert(wheels.rx_frames == 3u && wheels.left_frames == 1u && wheels.right_frames == 1u);
    assert(wheels.bus_off && wheels.bus_off_events == 1u && wheels.rx_loss_events == 1u);
    assert(wheels.last_rx_us == afterWrap + 1u);
    assert(balancing_f446_can1_snapshot(&snapshot));
    assert(!snapshot.bus_off && snapshot.rx_loss_events == 1u);
    assert(snapshot.sampled_at_us == wheels.sampled_at_us);
    can2.ESR = 0;
    assert(balancing_f446_can2_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can2_snapshot(&wheels) && !wheels.bus_off);
    assert(wheels.bus_off_events == 1u);
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.sample_valid && timing.sampled_at_us == afterWrap + 1u);
    assert(timing.elapsed_valid && timing.elapsed_us == afterWrap);

    // IRQなしでもserviceの時刻は進む。最後のRX時刻は変更しない。
    DWT->CYCCNT += 168u * 2000u;
    balancing_f446_can_service();
    assert(balancing_f446_can1_snapshot(&snapshot));
    assert(snapshot.last_rx_us == afterWrap && snapshot.sampled_at_us == afterWrap + 2001u);
    balancing_app_control_tick();
    assert(balancing_app_timing_snapshot(&timing));
    assert(timing.elapsed_valid && timing.elapsed_us == 2000u);
    for (const auto* bus : {CAN1, CAN2}) {
        for (const auto& mailbox : bus->sTxMailBox) {
            assert((mailbox.TIR & CAN_TI0R_TXRQ) == 0u);
        }
    }
    // 一方のキューを溢れさせても他方の新規受信は回収できる。
    for (unsigned i = 0; i < 40u; ++i) {
        pushReceive();
        assert(balancing_f446_can1_capture_irq());
    }
    pushWheel(0x202u);
    assert(balancing_f446_can2_capture_irq());
    balancing_f446_can_service();
    assert(balancing_f446_can2_snapshot(&wheels) && wheels.right_frames == 2u);
    assert(balancing_f446_can1_snapshot(&snapshot) && snapshot.rx_loss_events > 1u);
    assert(wheels.rx_loss_events == 1u);
    assert(wheels.sampled_at_us == snapshot.sampled_at_us);
    assert(primask == 0u);
    std::puts("PASS: F446 dual CAN boot, IRQ routing, independent monitors and shared clock wrap");
}
