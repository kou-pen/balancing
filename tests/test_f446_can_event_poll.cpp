#include "balancing/app/CanEventPoll.hpp"
#include "balancing/app/PowerBoardService.hpp"
#include "balancing/app/DiagnosticsService.hpp"
#include "balancing/app/ControlCycle.hpp"
#include "balancing/control/filter/ImuAttitudeSource.hpp"
#include "balancing/app/MotorService.hpp"
#include "balancing/app/MotorCanLink.hpp"
#include "balancing/communication/can/CanPort.hpp"
#include "balancing/communication/can/CanRouter.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"
#include "balancing/communication/can/MotorFramePlan.hpp"
#include "balancing/control/controller/MotorCommand.hpp"
#include "balancing/control/estimator/WheelOdometry.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cstdio>

namespace {
std::uint32_t primask{0};
}

std::uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1u; }
void __enable_irq() { primask = 0u; }

namespace {

using namespace balancing;

struct Fixture {
    CAN_TypeDef registers{};
    CAN_HandleTypeDef handle{&registers, HAL_CAN_STATE_LISTENING};
    CanPort port{handle};

    void pushRx(std::uint32_t id, std::uint64_t atUs, bool extended = false, std::uint32_t dlc = 1,
                std::uint32_t dataLow = 0x45u, std::uint32_t dataHigh = 0)
    {
        auto& fifo = registers.RF0R;
        assert(fifo.count < 3u);
        const auto rawId = extended ? (id << 3u) | CAN_RI0R_IDE : id << 21u;
        registers.sFIFOMailBox[fifo.count++] = {rawId, dlc, dataLow, dataHigh};
        port.captureInterrupt(atUs);
    }

    void occupy(std::size_t box)
    {
        assert((registers.sTxMailBox[box].TIR & CAN_TI0R_TXRQ) != 0u);
        registers.TSR.bits &= ~(CAN_TSR_TME0 << box);
    }

    void complete(std::size_t box, std::uint32_t flags, std::uint64_t atUs)
    {
        assert((registers.TSR.bits & (CAN_TSR_TME0 << box)) == 0u);
        registers.TSR.bits |= (CAN_TSR_TME0 << box) | ((CAN_TSR_RQCP0 | flags) << (8u * box));
        registers.sTxMailBox[box].TIR &= ~CAN_TI0R_TXRQ;
        port.captureInterrupt(atUs);
    }

    void finish(TxToken token, std::uint64_t atUs)
    {
        assert(port.trySend({}, token, atUs) == CanTxSubmitResult::Accepted);
        occupy(0);
        complete(0, CAN_TSR_TXOK0, atUs);
    }
};

struct Entry {
    CanBus bus{};
    CanEventKind kind{};
    std::uint64_t atUs{};
    TxToken token{};
    std::uint32_t id{};
    CanRxTarget target{CanRxTarget::Unmatched};
    CanTxOutcome outcome{CanTxOutcome::NotTransmitted};
};

class Recorder final : public CanEventSink {
public:
    std::array<Entry, 2 * CanPort::kMaxEventsPerPoll + 4> entries{};
    std::size_t count{0};
    Fixture* refill1{nullptr};
    Fixture* refill2{nullptr};

    void onCanReceive(CanBus bus, CanRxTarget target, const CanFrame& frame,
                      std::uint64_t atUs) noexcept override
    {
        assert(frame.data[0] == 0x45);
        record({bus, CanEventKind::Rx, atUs, {}, frame.id, target});
        // 配送ごとにIRQが1件追加する場合でも、同じpoll内で追い続けない。
        auto* refill = bus == CanBus::Can1 ? refill1 : refill2;
        if (refill) {
            refill->pushRx(frame.id, atUs + 1u, frame.extended, frame.dlc);
        }
    }

    void onCanTransmit(CanBus bus, TxToken token, CanTxOutcome outcome, std::uint64_t atUs) noexcept override
    {
        Entry entry{bus, CanEventKind::Tx, atUs, token};
        entry.outcome = outcome;
        record(entry);
    }

    void onCanPortStatus(CanBus bus, CanEventKind kind, std::uint64_t atUs) noexcept override
    {
        record({bus, kind, atUs});
    }

private:
    void record(Entry entry) noexcept
    {
        // イベント処理をCanPortのIRQマスク区間に入れると、負荷によってタイマを遅らせる。
        assert(primask == 0u && count < entries.size());
        entries[count++] = entry;
    }
};

void testBusAndEventTypeRemainPaired()
{
    Fixture can1, can2;
    Recorder sink;
    CanRouter router{sink};
    const TxToken token{91, 3};
    can1.pushRx(0x303, 9);
    can1.pushRx(0x402, 10);
    can1.finish(token, 12);
    can2.finish(token, 8);
    can2.registers.ESR = CAN_ESR_BOFF;
    can2.port.captureInterrupt(11);

    // CAN2の異常と終端は、CAN1の受信キューが空く前に配送される。
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 5u);
    const auto& e = sink.entries;
    assert(e[0].bus == CanBus::Can1 && e[0].kind == CanEventKind::Rx && e[0].id == 0x303 && e[0].atUs == 9);
    assert(e[1].bus == CanBus::Can2 && e[1].kind == CanEventKind::BusOff && e[1].atUs == 11);
    assert(e[2].bus == CanBus::Can1 && e[2].kind == CanEventKind::Rx && e[2].id == 0x402 && e[2].atUs == 10);
    assert(e[3].bus == CanBus::Can2 && e[3].kind == CanEventKind::Tx && e[3].token == token && e[3].atUs == 8);
    assert(e[4].bus == CanBus::Can1 && e[4].kind == CanEventKind::Tx && e[4].token == token && e[4].atUs == 12);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 5u);
}

void testBothPortsReturnAtLimitAndResumeLater()
{
    Fixture can1, can2;
    Recorder sink;
    CanRouter router{sink};
    sink.refill1 = &can1;
    sink.refill2 = &can2;
    can1.pushRx(1, 100);
    can2.pushRx(2, 200);
    pollCanEvents(&can1.port, &can2.port, router);

    const auto budget = CanPort::kMaxEventsPerPoll;
    assert(sink.count == 2u * budget);
    for (std::size_t i = 0; i < budget; ++i) {
        const auto& a = sink.entries[2 * i];
        const auto& b = sink.entries[2 * i + 1];
        assert(a.bus == CanBus::Can1 && a.id == 1 && a.atUs == 100 + i);
        assert(b.bus == CanBus::Can2 && b.id == 2 && b.atUs == 200 + i);
    }

    sink.refill1 = nullptr;
    sink.refill2 = nullptr;
    // 片側接続と両側未接続の場合も、残件を無効なポインタへ配送しない。
    pollCanEvents(nullptr, nullptr, router);
    assert(sink.count == 2u * budget);
    pollCanEvents(nullptr, &can2.port, router);
    assert(sink.count == 2u * budget + 1u && sink.entries[2 * budget].atUs == 200 + budget);
    pollCanEvents(&can1.port, nullptr, router);
    assert(sink.count == 2u * budget + 2u && sink.entries[2 * budget + 1].atUs == 100 + budget);
}

void testRoutingUsesBusIdentifierFormatAndDlc()
{
    Recorder sink;
    CanRouter router{sink};

    struct Case {
        CanBus bus;
        std::uint32_t id;
        bool extended;
        std::uint8_t dlc;
        CanRxTarget target;
    };
    // ID下位の主機・軸や状態ビットは配送先の判断に含めない。
    constexpr std::uint32_t rsStatus = 0x02b4015au;
    constexpr std::uint32_t rsPeriodic = 0x1800045au;
    const Case cases[]{
        {CanBus::Can1, rsStatus, true, 8, CanRxTarget::Rs06Status},
        {CanBus::Can1, rsPeriodic, true, 8, CanRxTarget::Rs06Status},
        {CanBus::Can1, rsStatus, true, 7, CanRxTarget::Unmatched},
        {CanBus::Can1, rsStatus, true, 9, CanRxTarget::Unmatched},
        {CanBus::Can1, rsStatus, false, 8, CanRxTarget::Unmatched},
        {CanBus::Can1, rsStatus | (1u << 29u), true, 8, CanRxTarget::Unmatched},
        {CanBus::Can1, 0x0300015a, true, 8, CanRxTarget::Unmatched},
        {CanBus::Can2, rsStatus, true, 8, CanRxTarget::Unmatched},
        {CanBus::Can2, 0x201, false, 8, CanRxTarget::C620LeftFeedback},
        {CanBus::Can2, 0x202, false, 8, CanRxTarget::C620RightFeedback},
        {CanBus::Can2, 0x201, true, 8, CanRxTarget::Unmatched},
        {CanBus::Can2, 0x202, false, 7, CanRxTarget::Unmatched},
        {CanBus::Can1, 0x201, false, 8, CanRxTarget::Unmatched},
        {CanBus::Can2, 0x200, false, 8, CanRxTarget::Unmatched},
        {CanBus::Can2, 0x203, false, 8, CanRxTarget::Unmatched},
        {CanBus::Can2, 0x600, false, 8, CanRxTarget::Unmatched},
    };
    for (const auto& test : cases) {
        const auto previousCount = sink.count;
        const auto atUs = 100u + previousCount;
        CanEvent event{CanEventKind::Rx, atUs};
        event.frame = {test.id, {0x45}, test.dlc, test.extended};
        router.dispatch(test.bus, event);
        assert(sink.count == previousCount + 1u);
        const auto& received = sink.entries[previousCount];
        assert(received.bus == test.bus && received.id == test.id && received.atUs == atUs);
        assert(received.target == test.target);
    }
}

void testCan2ProtocolReceiveDirections()
{
    // 仕様表から独立に転記した35個の具体ID。F446の入力12個だけを分類する。
    struct Case { std::uint32_t id; std::uint8_t dlc; CanRxTarget target; };
    const Case cases[]{
        {0x47c01u,0,CanRxTarget::Unmatched},
        {0x4bc01u,0,CanRxTarget::EmergencyStop},
        {0x4fc01u,0,CanRxTarget::EmergencyStop},
        {0x8404801u,2,CanRxTarget::Unmatched},
        {0x840c801u,2,CanRxTarget::Unmatched},
        {0x4404802u,2,CanRxTarget::Unmatched},
        {0x440c802u,2,CanRxTarget::Unmatched},
        {0x840bc03u,8,CanRxTarget::PowerStatus},
        {0x8408404u,8,CanRxTarget::PowerResult},
        {0x8408c04u,8,CanRxTarget::Unmatched},
        {0x880c401u,8,CanRxTarget::RemoteMotion},
        {0x880c402u,2,CanRxTarget::RemoteRequest},
        {0x480c403u,2,CanRxTarget::RemoteRequest},
        {0x880c404u,2,CanRxTarget::RemoteRequest},
        {0x880c405u,3,CanRxTarget::RemoteRequest},
        {0x8804c06u,8,CanRxTarget::Unmatched},
        {0xcc04c01u,8,CanRxTarget::Unmatched},
        {0xcc04c02u,8,CanRxTarget::Unmatched},
        {0xcc04c03u,8,CanRxTarget::Unmatched},
        {0xcc04c04u,4,CanRxTarget::Unmatched},
        {0xcc04c05u,8,CanRxTarget::Unmatched},
        {0xcc04c06u,8,CanRxTarget::Unmatched},
        {0xcc04c07u,8,CanRxTarget::Unmatched},
        {0xcc04c08u,8,CanRxTarget::Unmatched},
        {0xcc04c09u,8,CanRxTarget::Unmatched},
        {0xcc04c0au,8,CanRxTarget::Unmatched},
        {0x4c04c20u,8,CanRxTarget::Unmatched},
        {0x4c04c21u,8,CanRxTarget::Unmatched},
        {0x4c04c22u,8,CanRxTarget::Unmatched},
        {0xcc0c480u,8,CanRxTarget::DiagnosticRequest},
        {0xcc0c481u,4,CanRxTarget::DiagnosticRequest},
        {0xcc04c82u,8,CanRxTarget::Unmatched},
        {0xd00c401u,4,CanRxTarget::DiagnosticRequest},
        {0xd004c02u,8,CanRxTarget::Unmatched},
        {0xd004c03u,6,CanRxTarget::Unmatched},
    };
    unsigned accepted=0;
    for (const auto& c:cases) {
        Recorder sink;
        CanRouter router{sink};
        const auto check=[&](CanBus bus,CanFrame frame,CanRxTarget expected) {
            const auto n=sink.count;
            CanEvent event{CanEventKind::Rx,1000u+n};
            event.frame=frame;
            router.dispatch(bus,event);
            assert(sink.count==n+1);
            const auto& e=sink.entries[n];
            assert(e.bus==bus && e.id==frame.id && e.atUs==event.atUs && e.target==expected);
        };
        const CanFrame frame{c.id,{0x45},c.dlc,true};
        check(CanBus::Can2,frame,c.target);
        if (c.target!=CanRxTarget::Unmatched) ++accepted;
        // CAN1は独立したRS06バス。自作CAN2の定義を流用しない。
        check(CanBus::Can1,frame,CanRxTarget::Unmatched);
        auto bad=frame;bad.extended=false;
        check(CanBus::Can2,bad,CanRxTarget::Unmatched);
        for (unsigned dlc=0;dlc<=9;++dlc) {
            if (dlc==c.dlc) continue;
            bad=frame;bad.dlc=static_cast<std::uint8_t>(dlc);
            check(CanBus::Can2,bad,CanRxTarget::Unmatched);
        }
        for (auto id:{0x700u,0x1c000001u,c.id|0x20000000u,c.id^0x100u}) {
            bad=frame;bad.id=id;check(CanBus::Can2,bad,CanRxTarget::Unmatched);
        }
    }
    assert(accepted==12);
    Recorder sink;CanRouter router{sink};
    // IRQ由来のDLC0データは未使用バイトに意味がない。
    for (auto id:{0x1ffu,0x200u,0x203u,0x208u,0x700u}) {
        CanEvent event{CanEventKind::Rx,99};
        event.frame={id,{0x45},8,false};
        router.dispatch(CanBus::Can2,event);
        assert(sink.entries[sink.count-1].target==CanRxTarget::Unmatched);
    }
}

void testRoutingFromBothPortQueues()
{
    Fixture can1, can2;
    Recorder sink;
    CanRouter router{sink};
    can1.pushRx(0x0200015a, 20, true, 8);
    can2.pushRx(0x201, 21, false, 8);
    can2.pushRx(0x202, 22, false, 8);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 3);
    assert(sink.entries[0].target == CanRxTarget::Rs06Status);
    assert(sink.entries[1].target == CanRxTarget::C620LeftFeedback);
    assert(sink.entries[2].target == CanRxTarget::C620RightFeedback);
}

// 本番のRouterとCanPortを利用するが、実機の配送先・フィルタは変更しない。
void testPowerStatusThroughCanQueue()
{
    class PowerSink final : public CanEventSink {
    public:
        PowerBoardService power;
        std::uint64_t nowUs{};
        void onCanReceive(CanBus bus, CanRxTarget target, const CanFrame& frame,
                          std::uint64_t atUs) noexcept override
        {
            assert(primask == 0u);
            (void)power.acceptCanStatus(bus, target, frame, atUs, nowUs);
        }
        void onCanTransmit(CanBus, TxToken, CanTxOutcome, std::uint64_t) noexcept override {}
        void onCanPortStatus(CanBus, CanEventKind, std::uint64_t) noexcept override {}
    } sink;
    Fixture can1, can2;
    CanRouter router{sink};
    // 仕様表の独立した固定IDとバイト列を使う。CAN1の同IDは採用しない。
    // 電源ON=02、flags=80、faultBits=1234、sequence=00AB、予約=0000。
    can1.pushRx(0x0840bc03u,100,true,8,0x34128002u,0x0000ab00u);
    sink.nowUs=110;
    pollCanEvents(&can1.port,&can2.port,router);
    assert(!sink.power.snapshot(110).latest);
    can2.pushRx(0x0840bc03u,120,true,8,0x34128002u,0x0000ab00u);
    sink.nowUs=130;
    pollCanEvents(&can1.port,&can2.port,router);
    const auto report=sink.power.snapshot(130);
    assert(report.ready && report.latest->receivedAtUs==120 && report.latest->wire);
    assert(report.latest->wire->flags==0x80 && report.latest->wire->faultBits==0x1234);
    assert(report.latest->wire->sequence==0xab);
    // RouterでPowerStatusと分類されても、予約違反はCodecで拒否する。
    can2.pushRx(0x0840bc03u,140,true,8,0x00000003u,0x01000000u);
    sink.nowUs=150;
    pollCanEvents(&can1.port,&can2.port,router);
    assert(sink.power.snapshot(150).latest->receivedAtUs==120);
    assert(sink.power.snapshot(500119).ready && !sink.power.snapshot(500120).fresh);
    // POWER_RESULTに含む状態は最新のPOWER_STATUSを置き換えない。
    can2.pushRx(0x08408404u,160,true,8,0x01000100u,0x00000200u);
    sink.nowUs=170;
    pollCanEvents(&can1.port,&can2.port,router);
    assert(sink.power.snapshot(170).latest->receivedAtUs==120);
}

void testTerminalOnlyReleasesTheOriginatingBus()
{
    Fixture can1, can2;
    Recorder sink;
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    CanRouter router{sink, &tx1, &tx2};
    for (std::size_t box = 0; box < 3; ++box) {
        // 発行元も番号も同じにして、バスの区別だけで別々に管理できるかを見る。
        const TxToken token{box + 1u, 9};
        assert(tx1.trySubmit({}, token, 10) == CanTxSubmitResult::Accepted);
        assert(tx2.trySubmit({}, token, 10) == CanTxSubmitResult::Accepted);
        can1.occupy(box);
        can2.occupy(box);
    }

    const TxToken fourth{4, 9};
    can2.complete(0, CAN_TSR_TERR0, 12);
    assert(tx2.trySubmit({}, fourth, 13) == CanTxSubmitResult::Busy);
    assert(sink.count == 0u);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 1u && sink.entries[0].bus == CanBus::Can2);
    const TxToken first{1, 9};
    assert(sink.entries[0].token == first && sink.entries[0].atUs == 12u);
    assert(sink.entries[0].outcome == CanTxOutcome::TransmitError);

    // CAN2のエラー終端でCAN2だけに空きができる。CAN1の同値トークンは依然として有効。
    assert(tx2.trySubmit({}, fourth, 13) == CanTxSubmitResult::Accepted);
    assert(tx1.trySubmit({}, fourth, 13) == CanTxSubmitResult::Busy);
    can1.complete(0, CAN_TSR_TXOK0, 14);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 2u && sink.entries[1].bus == CanBus::Can1);
    assert(sink.entries[1].token == first && sink.entries[1].atUs == 14u);
    assert(sink.entries[1].outcome == CanTxOutcome::Sent);
    assert(tx1.trySubmit({}, fourth, 15) == CanTxSubmitResult::Accepted);
}

void testSchedulerIgnoresUnacceptedAndStaleEvents()
{
    Fixture can1;
    Recorder sink;
    CanTxScheduler scheduler{can1.port};
    CanRouter router{sink, &scheduler};
    const TxToken token{1, 9};
    CanEvent event{CanEventKind::Tx, 11};
    event.token = token;
    event.outcome = CanTxOutcome::Sent;

    can1.handle.State = HAL_CAN_STATE_READY;
    assert(scheduler.trySubmit({}, token, 10) == CanTxSubmitResult::NotReady);
    router.dispatch(CanBus::Can1, event);
    can1.handle.State = HAL_CAN_STATE_LISTENING;
    CanFrame invalid{};
    invalid.dlc = 9;
    assert(scheduler.trySubmit(invalid, token, 10) == CanTxSubmitResult::Invalid);
    router.dispatch(CanBus::Can1, event);
    can1.registers.TSR.bits = 0;
    assert(scheduler.trySubmit({}, token, 10) == CanTxSubmitResult::Busy);
    router.dispatch(CanBus::Can1, event);
    assert(sink.count == 0u);

    can1.registers.TSR.bits = CAN_TSR_TME0;
    assert(scheduler.trySubmit({}, token, 12) == CanTxSubmitResult::Accepted);
    can1.occupy(0);
    router.dispatch(CanBus::Can1, event); // 一致するトークンでも受付より前なら無効。
    event.atUs = 12;
    event.token.origin = 10;
    router.dispatch(CanBus::Can1, event); // 番号だけが同じで発行元が違う。
    assert(scheduler.trySubmit({}, token, 12) == CanTxSubmitResult::Busy);
    assert(sink.count == 0u);

    can1.complete(0, CAN_TSR_TXOK0, 12);
    pollCanEvents(&can1.port, nullptr, router);
    assert(sink.count == 1u && sink.entries[0].atUs == 12u);
    event.token = token;
    router.dispatch(CanBus::Can1, event);
    assert(sink.count == 1u);
}

void testNonSendTerminalEventsReleaseTheEntry()
{
    Fixture can2;
    Recorder sink;
    CanTxScheduler scheduler{can2.port};
    CanRouter router{sink, nullptr, &scheduler};
    const std::array<std::uint32_t, 2> flags{CAN_TSR_ALST0, 0u};
    const std::array<CanTxOutcome, 2> outcomes{CanTxOutcome::ArbitrationLost, CanTxOutcome::NotTransmitted};
    for (std::size_t i = 0; i < 8u; ++i) {
        // 各失敗を3枠より多く再現する。枠を返し忘れた実装では途中で満杯になる。
        const auto kind = i % flags.size();
        const TxToken token{i + 1u, 9};
        assert(scheduler.trySubmit({}, token, 20u + i * 2u) == CanTxSubmitResult::Accepted);
        can2.occupy(0);
        can2.complete(0, flags[kind], 21u + i * 2u);
        pollCanEvents(nullptr, &can2.port, router);
        assert(sink.count == i + 1u);
        assert(sink.entries[i].token == token && sink.entries[i].outcome == outcomes[kind]);
    }
}

PeriodicTxBatch periodicBatch(std::uint64_t generation, std::uint64_t deadline)
{
    PeriodicTxBatch batch{generation, deadline};
    for (std::size_t i = 0; i < batch.requests.size(); ++i) {
        // 仮想データの先頭バイトを世代と一致させ、投入されたペイロードから新旧を識別する。
        const CanFrame frame{static_cast<std::uint32_t>(0x100u + i),
                             {static_cast<std::uint8_t>(generation)}, 1, false};
        batch.requests[i] = PeriodicTxRequest{frame, {4u * generation + i, 9}};
    }
    return batch;
}

void testFullCan1ReplacesTheFourthPeriodicFrame()
{
    Fixture can1;
    Recorder sink;
    CanTxScheduler scheduler{can1.port};
    CanRouter router{sink, &scheduler};
    const auto first = periodicBatch(1, 4000);
    assert(scheduler.replacePeriodic(first, 0) == PeriodicBatchResult::Queued);
    for (std::size_t i = 0; i < 3u; ++i) {
        const auto tx = scheduler.service(0);
        assert(tx.result == ScheduledTxResult::Accepted && tx.token == first.requests[i]->token);
        can1.occupy(i);
    }
    assert(scheduler.service(3999).result == ScheduledTxResult::Busy);

    auto second = periodicBatch(2, 8000);
    // 次周期は4番目の対象だけを更新し、旧周期の4番目がFIFOに残らないことを確認する。
    second.requests[0].reset();
    second.requests[1].reset();
    second.requests[2].reset();
    assert(scheduler.replacePeriodic(second, 4000) == PeriodicBatchResult::Queued);
    can1.complete(0, CAN_TSR_TXOK0, 4001);
    pollCanEvents(&can1.port, nullptr, router);
    // 空きがあっても、旧周期の受理済みフレームが残る間は新旧を混在させない。
    assert(sink.count == 1u && sink.entries[0].token == first.requests[0]->token);
    assert(scheduler.service(4001).result == ScheduledTxResult::Busy);

    can1.complete(1, CAN_TSR_TERR0, 4010);
    can1.complete(2, CAN_TSR_TXOK0, 4010);
    assert(scheduler.service(4010).result == ScheduledTxResult::Busy);
    pollCanEvents(&can1.port, nullptr, router);
    assert(sink.count == 3u && sink.entries[1].outcome == CanTxOutcome::TransmitError);
    const auto latest = scheduler.service(4010);
    assert(latest.result == ScheduledTxResult::Accepted && latest.token == second.requests[3]->token);
    assert((can1.registers.sTxMailBox[0].TIR >> 21u) == second.requests[3]->frame.id);
    assert(can1.registers.sTxMailBox[0].TDLR == 2u);
    assert(scheduler.service(4010).result == ScheduledTxResult::Idle);
}

void testFourthPeriodicFrameCanUseTheFirstReleasedMailbox()
{
    Fixture can1;
    Recorder sink;
    CanTxScheduler scheduler{can1.port};
    CanRouter router{sink, &scheduler};
    const auto batch = periodicBatch(1, 4000);
    assert(scheduler.replacePeriodic(batch, 0) == PeriodicBatchResult::Queued);
    for (std::size_t i = 0; i < 3u; ++i) {
        assert(scheduler.service(0).result == ScheduledTxResult::Accepted);
        can1.occupy(i);
    }

    auto invalidNext = periodicBatch(2, 8000);
    invalidNext.requests[0]->token = batch.requests[0]->token;
    assert(scheduler.replacePeriodic(invalidNext, 3998) == PeriodicBatchResult::Invalid);
    // 同じ周期なら、ほかの2枠が送信中でも最初に空いた枠で4台目を送れる。
    can1.complete(0, CAN_TSR_TXOK0, 3999);
    pollCanEvents(&can1.port, nullptr, router);
    const auto fourth = scheduler.service(3999);
    assert(fourth.result == ScheduledTxResult::Accepted && fourth.token == batch.requests[3]->token);
    assert(can1.registers.sTxMailBox[0].TDLR == 1u);
}

void testPeriodicDeadlineAndAtomicValidation()
{
    Fixture can1;
    CanTxScheduler scheduler{can1.port};
    const auto first = periodicBatch(1, 4000);
    assert(scheduler.replacePeriodic(first, 0) == PeriodicBatchResult::Queued);
    auto second = periodicBatch(2, 8000);
    second.requests[3]->frame.dlc = 9;
    assert(scheduler.replacePeriodic(second, 1) == PeriodicBatchResult::Invalid);
    second.requests[3] = second.requests[0];
    assert(scheduler.replacePeriodic(second, 1) == PeriodicBatchResult::Invalid);
    second.requests[3] = first.requests[3];
    assert(scheduler.replacePeriodic(second, 1) == PeriodicBatchResult::Invalid);
    assert(scheduler.replacePeriodic(first, 2) == PeriodicBatchResult::StaleGeneration);
    assert(scheduler.trySubmit(first.requests[0]->frame, first.requests[0]->token, 2) == CanTxSubmitResult::Busy);

    can1.handle.State = HAL_CAN_STATE_READY;
    assert(scheduler.service(3999).result == ScheduledTxResult::NotReady);
    can1.handle.State = HAL_CAN_STATE_LISTENING;
    can1.registers.TSR.bits = 0;
    assert(scheduler.service(3999).result == ScheduledTxResult::Busy);
    can1.registers.TSR.bits = CAN_TSR_TME0;
    // ポートが空いた時刻が期限ちょうどなら送信しない。4件すべて元のトークンで期限切れを返す。
    for (const auto& request : first.requests) {
        const auto expired = scheduler.service(4000);
        assert(expired.result == ScheduledTxResult::Expired && expired.token == request->token);
    }
    assert(scheduler.service(4000).result == ScheduledTxResult::Idle);
    assert(can1.registers.sTxMailBox[0].TIR == 0u);

    second = periodicBatch(2, 8000);
    assert(scheduler.replacePeriodic(second, 4000) == PeriodicBatchResult::Queued);
    const auto late = periodicBatch(3, 6000);
    assert(scheduler.replacePeriodic(late, 6000) == PeriodicBatchResult::Expired);
    assert(scheduler.replacePeriodic(second, 6000) == PeriodicBatchResult::StaleGeneration);
    assert(scheduler.service(6000).result == ScheduledTxResult::Idle);
}

void testPeriodicGenerationIsIndependentOnCan2()
{
    Fixture can1, can2;
    Recorder sink;
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    CanRouter router{sink, &tx1, &tx2};
    const auto legs = periodicBatch(1, 4000);
    assert(tx1.replacePeriodic(legs, 0) == PeriodicBatchResult::Queued);
    assert(tx1.service(0).result == ScheduledTxResult::Accepted);
    can1.occupy(0);

    // C620共通フレームは左右を別々に待たせず、4バイトを含んだ1件として差し替える。
    const CanFrame wheels{0x200, {0x11, 0x22, 0x33, 0x44}, 8, false};
    PeriodicTxBatch wheelBatch{1, 2000};
    wheelBatch.requests[0] = PeriodicTxRequest{wheels, {4, 9}};
    assert(tx2.replacePeriodic(wheelBatch, 0) == PeriodicBatchResult::Queued);
    assert(tx2.service(0).result == ScheduledTxResult::Accepted);
    can2.occupy(0);

    wheelBatch = {2, 4000};
    wheelBatch.requests[0] = PeriodicTxRequest{{0x200, {0x55, 0x66, 0x77, 0x88}, 8, false}, {8, 9}};
    assert(tx2.replacePeriodic(wheelBatch, 2000) == PeriodicBatchResult::Queued);
    assert(tx2.service(2000).result == ScheduledTxResult::Busy);
    can1.complete(0, CAN_TSR_TXOK0, 2001);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(tx2.service(2001).result == ScheduledTxResult::Busy);
    can2.complete(0, CAN_TSR_TXOK0, 2002);
    pollCanEvents(&can1.port, &can2.port, router);
    const auto wheel = tx2.service(2002);
    assert(wheel.result == ScheduledTxResult::Accepted && wheel.token == wheelBatch.requests[0]->token);
    assert(can2.registers.sTxMailBox[0].TDLR == 0x88776655u);
    assert(tx1.service(2002).token == legs.requests[1]->token);
}

EventTxRequest eventRequest(std::uint32_t id, std::uint64_t serial, std::uint64_t deadline)
{
    return {{id, {static_cast<std::uint8_t>(serial)}, 1, false}, {serial, 19}, deadline};
}

void testNormalEventsFinishInOrderBeforePeriodicTx()
{
    Fixture can1, can2;
    Recorder sink;
    std::array<EventTxRequest, 2> fifo1{}, fifo2{};
    CanTxScheduler tx1{can1.port, fifo1}, tx2{can2.port, fifo2};
    CanRouter router{sink, &tx1, &tx2};
    const auto cycle = periodicBatch(1, 4000);
    assert(tx1.replacePeriodic(cycle, 0) == PeriodicBatchResult::Queued);

    // 小さいIDを後ろに入れる。同時にTXRQを立てる実装だと、後続が先に送られ得る。
    const auto first = eventRequest(0x700, 1, 1000);
    const auto second = eventRequest(0x101, 2, 1000);
    assert(tx1.enqueueEvent(first, 0) == EventEnqueueResult::Queued);
    assert(tx1.enqueueEvent(second, 0) == EventEnqueueResult::Queued);
    assert(tx2.enqueueEvent(first, 0) == EventEnqueueResult::Queued);
    assert(tx1.trySubmit({}, {99, 19}, 0) == CanTxSubmitResult::Busy);
    assert(tx1.service(0).token == first.token);
    assert((can1.registers.sTxMailBox[0].TIR >> 21u) == first.frame.id);
    can1.occupy(0);
    assert(tx2.service(0).token == first.token);
    can2.occupy(0);
    can2.complete(0, CAN_TSR_TXOK0, 1);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 1u && sink.entries[0].bus == CanBus::Can2);
    assert(tx1.service(1).result == ScheduledTxResult::Busy);
    assert(can1.registers.sTxMailBox[1].TIR == 0u);

    can1.complete(0, CAN_TSR_TERR0, 2);
    assert(tx1.service(2).result == ScheduledTxResult::Busy);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 2u && sink.entries[1].bus == CanBus::Can1);
    assert(sink.entries[1].token == first.token && sink.entries[1].outcome == CanTxOutcome::TransmitError);
    assert(tx1.service(2).token == second.token);
    assert((can1.registers.sTxMailBox[0].TIR >> 21u) == second.frame.id);
    can1.occupy(0);
    // FIFOが空になっても、通常イベントが実際に終端するまで周期を待たせる。
    assert(tx1.service(3).result == ScheduledTxResult::Busy);
    assert(can1.registers.sTxMailBox[1].TIR == 0u);
    can1.complete(0, CAN_TSR_TXOK0, 4);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 3u && sink.entries[2].token == second.token);
    assert(tx1.service(4).token == cycle.requests[0]->token);
}

void testNormalEventFifoFullExpiryAndWrap()
{
    Fixture can1, can2;
    Recorder sink;
    std::array<EventTxRequest, 2> fifo{};
    CanTxScheduler tx{can1.port, fifo}, withoutQueue{can2.port};
    CanRouter router{sink, &tx};
    const auto first = eventRequest(0x503, 1, 20);
    const auto second = eventRequest(0x502, 2, 60);
    const auto third = eventRequest(0x501, 3, 70);
    assert(withoutQueue.enqueueEvent(first, 10) == EventEnqueueResult::QueueFull);
    assert(tx.enqueueEvent(first, 10) == EventEnqueueResult::Queued);
    assert(tx.enqueueEvent(second, 10) == EventEnqueueResult::Queued);
    assert(tx.enqueueEvent(second, 11) == EventEnqueueResult::Invalid);
    assert(tx.enqueueEvent(third, 11) == EventEnqueueResult::QueueFull);

    auto collision = periodicBatch(1, 4000);
    collision.requests[0]->token = second.token;
    assert(tx.replacePeriodic(collision, 11) == PeriodicBatchResult::Invalid);

    can1.handle.State = HAL_CAN_STATE_READY;
    assert(tx.service(19).result == ScheduledTxResult::NotReady);
    const auto expired = tx.service(20);
    assert(expired.result == ScheduledTxResult::Expired && expired.token == first.token);
    assert(tx.enqueueEvent(first, 20) == EventEnqueueResult::Expired);
    assert(tx.enqueueEvent(third, 20) == EventEnqueueResult::Queued);

    can1.handle.State = HAL_CAN_STATE_LISTENING;
    can1.registers.TSR.bits = 0u;
    assert(tx.service(21).result == ScheduledTxResult::Busy);
    can1.registers.TSR.bits = CAN_TSR_TME0;
    // 期限切れと満杯で後続の順序が変わらず、末尾を循環再利用できる。
    assert(tx.service(22).token == second.token);
    assert(can1.registers.sTxMailBox[0].TDLR == 2u);
    can1.occupy(0);
    const auto overdue = tx.service(70);
    assert(overdue.result == ScheduledTxResult::Expired && overdue.token == third.token);
    can1.complete(0, CAN_TSR_TXOK0, 71);
    pollCanEvents(&can1.port, nullptr, router);
    assert(sink.count == 1u && sink.entries[0].token == second.token);
    assert(tx.service(72).result == ScheduledTxResult::Idle);
}

void testFourStopsHavePriorityWhileTheNormalFifoIsFull()
{
    Fixture can1;
    Recorder sink;
    std::array<EventTxRequest, 2> fifo{};
    CanTxScheduler tx{can1.port, fifo};
    CanRouter router{sink, &tx};
    const auto oldCycle = periodicBatch(1, 4000);
    assert(tx.replacePeriodic(oldCycle, 0) == PeriodicBatchResult::Queued);
    const auto normal = eventRequest(0x110, 1, 1000);
    const auto waiting = eventRequest(0x111, 2, 1000);
    const auto secondWaiting = eventRequest(0x112, 3, 1000);

    // 通常イベントを1つだけハードへ置き、残りの通常FIFOも満杯にする。
    // 停止はFIFOの残量にも、受付済み通常イベントの終端にも依存せず空き枠を使う。
    assert(tx.enqueueEvent(normal, 0) == EventEnqueueResult::Queued);
    assert(tx.service(0).token == normal.token);
    can1.occupy(0);
    assert(tx.enqueueEvent(waiting, 1) == EventEnqueueResult::Queued);
    assert(tx.enqueueEvent(secondWaiting, 1) == EventEnqueueResult::Queued);
    assert(tx.enqueueEvent(eventRequest(0x113, 4, 1000), 1) == EventEnqueueResult::QueueFull);
    assert(tx.enqueueStop(0, normal.frame, waiting.token) == StopEnqueueResult::Invalid);
    assert(tx.enqueueStop(0, normal.frame, oldCycle.requests[0]->token) == StopEnqueueResult::Invalid);
    assert(tx.enqueueStop(0, normal.frame, normal.token) == StopEnqueueResult::Invalid);

    const std::array<TxToken, 4> stops{{{21, 9}, {22, 9}, {23, 9}, {24, 9}}};
    for (std::size_t i = stops.size(); i > 0; --i) {
        // 実機のIDを仮定せず、対象番号と投入順がペイロードからも区別できるようにする。
        const auto axis = i - 1;
        const CanFrame frame{0x210u, {static_cast<std::uint8_t>(axis)}, 1, false};
        assert(tx.enqueueStop(axis, frame, stops[axis]) == StopEnqueueResult::Queued);
    }
    assert(tx.enqueueStop(stops.size(), {}, {25, 9}) == StopEnqueueResult::Invalid);
    assert(tx.enqueueStop(0, {}, {25, 9}) == StopEnqueueResult::TargetBusy);
    assert(tx.enqueueStop(0, {}, stops[0]) == StopEnqueueResult::Invalid);
    assert(tx.enqueueEvent({{}, stops[0], 1000}, 2) == EventEnqueueResult::Invalid);
    auto collision = periodicBatch(2, 4000);
    collision.requests[0]->token = stops[3];
    assert(tx.replacePeriodic(collision, 2) == PeriodicBatchResult::Invalid);
    const auto duringStop = periodicBatch(2, 4000);
    assert(tx.replacePeriodic(duringStop, 2) == PeriodicBatchResult::StopPending);

    for (std::size_t i = 0; i < 2; ++i) {
        assert(tx.service(2).token == stops[i]);
        assert(can1.registers.sTxMailBox[i + 1].TDLR == i);
        can1.occupy(i + 1);
    }
    assert(tx.service(2).result == ScheduledTxResult::Busy);
    can1.complete(1, CAN_TSR_TXOK0, 3);
    assert(tx.service(3).result == ScheduledTxResult::Busy); // IRQの採取だけではまだ枠を再利用しない。
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(3).token == stops[2]);
    can1.occupy(1);
    can1.complete(0, CAN_TSR_TXOK0, 4);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(4).token == stops[3]);
    can1.occupy(0);

    // 二つの停止が先に終端しても、最後の対象が終端するまで通常FIFOを進めない。
    can1.complete(1, CAN_TSR_TXOK0, 5);
    can1.complete(2, CAN_TSR_TXOK0, 5);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(5).result == ScheduledTxResult::Busy);
    can1.complete(0, CAN_TSR_TXOK0, 6);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.replacePeriodic(duringStop, 6) == PeriodicBatchResult::StaleGeneration);

    assert(tx.service(6).token == waiting.token);
    can1.occupy(0);
    can1.complete(0, CAN_TSR_TXOK0, 7);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(7).token == secondWaiting.token);
    can1.occupy(0);
    can1.complete(0, CAN_TSR_TXOK0, 8);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(8).result == ScheduledTxResult::Idle);
    const auto nextCycle = periodicBatch(3, 4000);
    assert(tx.replacePeriodic(nextCycle, 8) == PeriodicBatchResult::Queued);
    assert(tx.service(8).token == nextCycle.requests[0]->token);
}

void testFailedStopsRequireNewTokensAndAreIndependentBetweenBuses()
{
    Fixture can1, can2;
    Recorder sink;
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    CanRouter router{sink, &tx1, &tx2};
    const TxToken shared{1, 31};
    assert(tx1.enqueueStop(0, {}, shared) == StopEnqueueResult::Queued);
    assert(tx2.enqueueStop(0, {}, shared) == StopEnqueueResult::Queued);
    assert(tx1.service(10).token == shared);
    assert(tx2.service(10).token == shared);
    can1.occupy(0);
    can2.occupy(0);
    can2.complete(0, CAN_TSR_TXOK0, 11);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 1u && sink.entries[0].bus == CanBus::Can2);
    assert(tx1.enqueueStop(0, {}, {2, 31}) == StopEnqueueResult::TargetBusy);
    assert(tx1.trySubmit({}, {2, 31}, 11) == CanTxSubmitResult::Busy);
    assert(tx2.service(11).result == ScheduledTxResult::Idle);
    assert(tx2.trySubmit({}, {2, 31}, 11) == CanTxSubmitResult::Accepted);

    const std::array<std::uint32_t, 3> failures{CAN_TSR_TERR0, CAN_TSR_ALST0, 0u};
    const std::array<CanTxOutcome, 3> outcomes{
        CanTxOutcome::TransmitError, CanTxOutcome::ArbitrationLost, CanTxOutcome::NotTransmitted,
    };
    auto active = shared;
    for (std::size_t i = 0; i < failures.size(); ++i) {
        const auto finishAt = 12u + i * 2u;
        // 失敗の種類にかかわらず、空きメールボックスは返すが通常送信は保留する。
        can1.complete(0, failures[i], finishAt);
        pollCanEvents(&can1.port, &can2.port, router);
        assert(sink.count == i + 2 && sink.entries[i + 1].bus == CanBus::Can1);
        assert(sink.entries[i + 1].token == active && sink.entries[i + 1].outcome == outcomes[i]);
        assert(tx1.service(finishAt).result == ScheduledTxResult::Busy);
        assert(tx1.enqueueStop(0, {}, active) == StopEnqueueResult::Invalid);
        assert(tx1.enqueueStop(1, {}, active) == StopEnqueueResult::Invalid);
        const TxToken retry{active.serial + 1u, active.origin};
        assert(tx1.enqueueStop(0, {}, retry) == StopEnqueueResult::Queued);
        assert(tx1.service(finishAt + 1u).token == retry);
        can1.occupy(0);

        CanEvent stale{CanEventKind::Tx, finishAt + 1u};
        stale.token = active;
        stale.outcome = CanTxOutcome::Sent;
        router.dispatch(CanBus::Can1, stale);
        assert(sink.count == i + 2);
        assert(tx1.trySubmit({}, {95, 31}, finishAt + 1u) == CanTxSubmitResult::Busy);
        active = retry;
    }
    can1.complete(0, CAN_TSR_TXOK0, 18);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(sink.count == 5u && sink.entries[4].token == active && sink.entries[4].outcome == CanTxOutcome::Sent);
    assert(tx1.trySubmit({}, {95, 31}, 18) == CanTxSubmitResult::Accepted);
}

void testStopWaitSurvivesNotReadyAndBusOff()
{
    Fixture can1;
    Recorder sink;
    CanTxScheduler tx{can1.port};
    CanRouter router{sink, &tx};
    const TxToken first{1, 31}, second{2, 31};
    const CanFrame invalid{0x800u, {}, 8, false};
    assert(tx.enqueueStop(0, invalid, first) == StopEnqueueResult::Invalid);
    assert(tx.enqueueStop(0, {}, {}) == StopEnqueueResult::Invalid);
    assert(tx.enqueueStop(0, {}, first) == StopEnqueueResult::Queued);
    assert(tx.enqueueStop(1, {}, second) == StopEnqueueResult::Queued);
    can1.handle.State = HAL_CAN_STATE_READY;
    assert(tx.service(1).result == ScheduledTxResult::NotReady);
    can1.handle.State = HAL_CAN_STATE_LISTENING;
    can1.registers.ESR = CAN_ESR_BOFF;
    can1.port.captureInterrupt(2);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(2).result == ScheduledTxResult::Busy);
    can1.registers.ESR = 0;
    can1.port.captureInterrupt(50000);
    pollCanEvents(&can1.port, nullptr, router);
    assert(sink.entries[0].kind == CanEventKind::BusOff);
    assert(sink.entries[1].kind == CanEventKind::BusRecovered);

    // 停止待ちは経過時間で捨てない。旧トークンの送信失敗があっても別対象は進む。
    assert(tx.service(50000).token == first);
    can1.occupy(0);
    can1.complete(0, CAN_TSR_TERR0, 50001);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.service(50001).token == second);
    can1.occupy(0);
    can1.complete(0, CAN_TSR_TXOK0, 50002);
    pollCanEvents(&can1.port, nullptr, router);
    assert(tx.trySubmit({}, {3, 31}, 50002) == CanTxSubmitResult::Busy);
}

struct MotorLinkFixture {
    // 番号・時刻ともホスト試験専用。対象番号とモータIDが別であることも確認する。
    static constexpr std::uint8_t hostId = 0xa6;
    Fixture can1, can2;
    MotorService motors{{{11, 12, 13, 14}, hostId}};
    RobotSequence sequence{{1000, true}};
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    MotorCanLink link{motors, tx1};
    CanRouter router{link, &tx1, &tx2};

    static constexpr std::uint32_t reportId(std::uint8_t motorId, std::uint32_t mode,
                                             std::uint32_t type = 2u) noexcept
    {
        return (type << 24u) | (mode << 22u) | (std::uint32_t{motorId} << 8u) | hostId;
    }

    void poll()
    {
        pollCanEvents(&can1.port, &can2.port, router);
    }

    ScheduledTxProgress service(std::uint64_t nowUs)
    {
        const auto progress = tx1.service(nowUs);
        link.onScheduledTxProgress(progress, nowUs);
        return progress;
    }

    RobotState beginCycle(std::uint64_t nowUs, OperatorEvents operations = {})
    {
        poll();
        RobotSequenceInput input{};
        input.operations = operations;
        input.interlocks = {true, true, true, true, true, true, MechanismSafety::Safe};
        input.rs06 = motors.rs06ArmFeedback(nowUs);
        input.observedFaults = motors.takeObservedFaults(nowUs);
        const auto result = sequence.update(input, nowUs);
        motors.applySequence(result, nowUs);
        link.submitPending(nowUs);
        return result.state;
    }

    void expectFrame(std::size_t mailbox, std::uint32_t type, std::uint8_t motorId)
    {
        const auto& tx = can1.registers.sTxMailBox[mailbox];
        assert((tx.TIR & CAN_TI0R_TXRQ) != 0u);
        assert((tx.TIR & CAN_TI0R_IDE) != 0u);
        assert((tx.TIR >> 3u) == ((type << 24u) | (std::uint32_t{hostId} << 8u) | motorId));
        assert(tx.TDTR == 8u && tx.TDLR == 0u && tx.TDHR == 0u);
        can1.occupy(mailbox);
    }

    void completeBootStops()
    {
        assert(beginCycle(1) == RobotState::Disarmed);
        // 専用枠への登録はメールボックス受付前なので、候補はまだ軸側にある。
        assert(motors.pendingRs06Requests()[0].has_value());
        link.submitPending(2);
        for (unsigned axis = 0; axis < 3u; ++axis) {
            assert(service(2).result == ScheduledTxResult::Accepted);
            expectFrame(axis, 4, static_cast<std::uint8_t>(11u + axis));
        }
        assert(service(2).result == ScheduledTxResult::Busy);
        assert(motors.pendingRs06Requests()[3].has_value());
        can1.complete(1, CAN_TSR_TXOK0, 3);
        assert(service(3).result == ScheduledTxResult::Busy);
        poll();
        assert(!motors.rs06Status(3)[1].stopPending);
        assert(service(3).result == ScheduledTxResult::Accepted);
        expectFrame(1, 4, 14);
        for (unsigned box = 0; box < 3u; ++box) {
            can1.complete(box, CAN_TSR_TXOK0, 4);
        }
        poll();
        assert(!motors.rs06ArmFeedback(4).stopPending);
        assert(motors.takeObservedFaults(4) == FaultFlags::None);
    }
};

void testMotorCanLinkCollectsSixAxesForOneCycle()
{
    MotorLinkFixture f;
    const auto empty = f.motors.feedback(0);
    for (const auto& axis : empty.rs06) {
        assert(!axis.latest && !axis.fresh);
    }
    for (const auto& axis : empty.c620) {
        assert(!axis.latest && !axis.fresh);
    }

    // 受信時刻をずらして、回収時刻で全軸を上書きしていないことも確認する。
    constexpr std::uint64_t receivedAt = 1000;
    constexpr std::uint64_t cycleAt = 2000;
    for (std::size_t i = 0; i < empty.rs06.size(); ++i) {
        const auto motorId = static_cast<std::uint8_t>(11u + i);
        const auto id = f.reportId(motorId, 2, i == 0 ? 2u : 24u);
        // 角度・速度・トルクは量子化範囲の端、温度は25 °C。
        f.can1.pushRx(id, receivedAt + i, true, 8, 0xffff0000u, 0xfa000000u);
    }
    for (std::size_t i = 0; i < empty.c620.size(); ++i) {
        // ロータ角4096/4097、-100 rpm、電流123、42 °C。送信レジスタ上は下位バイトが先頭。
        const auto low = 0x9cff0010u | (static_cast<std::uint32_t>(i) << 8u);
        f.can2.pushRx(0x201u + static_cast<std::uint32_t>(i), receivedAt + i, false, 8, low, 0x002a7b00u);
    }
    f.poll();
    const auto snapshot = f.motors.feedback(cycleAt);
    assert(snapshot.sampledAtUs == cycleAt);
    for (std::size_t i = 0; i < snapshot.rs06.size(); ++i) {
        const auto& axis = snapshot.rs06[i];
        assert(axis.fresh && axis.latest && axis.latest->receivedAtUs == receivedAt + i);
        const auto& report = axis.latest->report;
        assert(report.motorId == 11u + i && report.hostId == MotorLinkFixture::hostId);
        assert(report.report == (i == 0 ? Rs06Report::Status : Rs06Report::Periodic));
        assert(report.angleRad == -12.566370614359172f && report.velocityRadPerSec == 50.0f);
        assert(report.torqueNm == -36.0f && report.temperatureC == 25.0f);
    }
    for (std::size_t i = 0; i < snapshot.c620.size(); ++i) {
        const auto& axis = snapshot.c620[i];
        assert(axis.fresh && axis.latest && axis.latest->receivedAtUs == receivedAt + i);
        const auto& report = axis.latest->report;
        assert(report.motor_id == 1u + i && report.encoder_raw == 4096u + i);
        assert(report.rpm == -100 && report.current_raw == 123 && report.temperature_c == 42);
    }

    // バス違い・宛先違い・復号不能な報告は、過去の値や鮮度を更新しない。
    f.can1.pushRx(0x201, 19000, false, 8);
    f.can2.pushRx(f.reportId(11, 2), 19000, true, 8);
    f.can1.pushRx(f.reportId(11, 2) ^ 1u, 19001, true, 8);
    f.can1.pushRx(f.reportId(11, 3), 19002, true, 8);
    f.can2.pushRx(0x201, 19002, false, 8, 0x00000020u);  // ロータ角8192は範囲外。
    f.can2.pushRx(0x203, 19003, false, 8);
    f.poll();
    assert(f.motors.feedback(receivedAt + 19999).c620[0].fresh);
    const auto wheelDeadline = f.motors.feedback(receivedAt + 20000);
    assert(!wheelDeadline.c620[0].fresh && wheelDeadline.c620[1].fresh);
    assert(wheelDeadline.c620[0].latest->receivedAtUs == receivedAt);
    assert(f.motors.feedback(receivedAt + 29999).rs06[0].fresh);
    const auto legDeadline = f.motors.feedback(receivedAt + 30000);
    assert(!legDeadline.rs06[0].fresh && legDeadline.rs06[1].fresh);
    assert(legDeadline.rs06[0].latest->receivedAtUs == receivedAt);
    assert(!f.motors.rs06Status(legDeadline.sampledAtUs)[0].feedbackFresh);
    assert(snapshot.c620[0].fresh && snapshot.rs06[0].fresh);
}

void testCan2ProtocolDoesNotChangeWheelObservations()
{
    MotorLinkFixture f;
    f.can2.pushRx(0x201,1000,false,8,0x00000010u,0);
    f.can2.pushRx(0x202,1001,false,8,0x00000010u,0);
    f.poll();
    const auto before=f.motors.feedback(2000);
    for (const auto& w:before.c620) assert(w.latest && w.fresh);
    // 自作IDを実際のポート模擬FIFOから流す。車輪観測・鮮度を更新しない。
    const CanFrame frames[]{
        {0x0004bc01u,{},0,true},{0x0004fc01u,{},0,true},
        {0x0840bc03u,{},8,true},{0x08408404u,{},8,true},
        {0x0880c401u,{},8,true},{0x0880c402u,{},2,true},
        {0x0480c403u,{},2,true},{0x0880c404u,{},2,true},
        {0x0880c405u,{},3,true},{0x0cc0c480u,{},8,true},
        {0x0cc0c481u,{},4,true},{0x0d00c401u,{},4,true}
    };
    for (const auto& frame:frames) {
        f.can2.pushRx(frame.id,2100,true,frame.dlc);
        f.poll();
    }
    const auto after=f.motors.feedback(2200);
    for (unsigned i=0;i<2;++i) {
        assert(after.c620[i].latest && after.c620[i].fresh);
        assert(after.c620[i].latest->receivedAtUs==before.c620[i].latest->receivedAtUs);
        assert(after.c620[i].latest->sequence==before.c620[i].latest->sequence);
        assert(after.c620[i].continuity==before.c620[i].continuity);
    }
    // 既知車輪の短いDLC通知は既存の連続性試験で別途検証する。
}

void testWheelContinuityThroughCanFifo()
{
    // 人工間隔・半径・減速比。実機の校正値ではない。
    Fixture can1, can2;
    MotorService motors{{{11,12,13,14},0xa6}, {{{4000,true},{4000,true}}}};
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    MotorCanLink link{motors,tx1};
    CanRouter router{link,&tx1,&tx2};
    WheelOdometry odom{{{{0.1f,10.0f,1.0f,true},{0.1f,10.0f,-1.0f,true}}}};
    auto poll = [&]() { pollCanEvents(&can1.port,&can2.port,router); };
    auto word = [](unsigned raw) { return (raw >> 8) | ((raw & 0xff) << 8); };
    can2.pushRx(0x201,1000,false,8,word(0));
    can2.pushRx(0x202,1000,false,8,word(0));
    poll();
    odom.update(motors.feedback(1000));

    // 一回のIRQでFIFOの複数フレームに同じ時刻が付く。
    can2.registers.sFIFOMailBox[0] = {0x201u << 21,8,word(3000),0};
    can2.registers.sFIFOMailBox[1] = {0x201u << 21,8,word(6000),0};
    can2.registers.RF0R.count = 2;
    can2.port.captureInterrupt(2000);
    can2.pushRx(0x202,2000,false,8,word(1));
    poll();
    const auto valid = motors.feedback(2000);
    assert(valid.c620[0].position->counts == 6000);
    assert(valid.c620[0].latest->sequence == 3);
    assert(odom.update(valid)[0] == WheelUpdateResult::Updated);

    // 既知IDの短いDLCは片側だけの連続性を無効にする。
    can2.pushRx(0x201,2100,false,7,word(6001));
    poll();
    assert(!motors.feedback(2100).c620[0].position);
    assert(motors.feedback(2100).c620[1].position);
    assert(odom.update(motors.feedback(2100))[0] == WheelUpdateResult::ContinuityUnavailable);

    // RxLostは古い通常RXより先に配送される。残存フレームで再基準化しない。
    can2.pushRx(0x202,2200,false,8,word(2));
    can2.registers.RF0R.flags = CAN_RF0R_FOVR0;
    can2.port.captureInterrupt(2300);
    poll();
    const auto lost = motors.feedback(2300);
    assert(lost.c620[1].latest->report.encoder_raw == 2);
    assert(lost.c620[1].continuity == C620ContinuityStatus::TransportLost);
    assert(!lost.c620[1].position);
    assert(odom.update(lost)[1] == WheelUpdateResult::ContinuityUnavailable);
    assert(odom.update(valid)[1] == WheelUpdateResult::ContinuityUnavailable);
    can2.registers.ESR = CAN_ESR_BOFF;
    can2.port.captureInterrupt(2400);
    poll();
    assert(!motors.resetWheelReference());
    can2.registers.ESR = 0;
    can2.port.captureInterrupt(2500);
    poll();
    assert(!motors.feedback(2500).c620[1].position);
    assert(motors.resetWheelReference());
    odom.reset();
    can2.pushRx(0x201,3000,false,8,word(6010));
    can2.pushRx(0x202,3000,false,8,word(3));
    poll();
    const auto reset = motors.feedback(3000);
    assert(reset.c620[0].position->counts == 0 && reset.c620[1].position->counts == 0);
    assert(odom.update(reset)[0] == WheelUpdateResult::Updated);
    assert(odom.snapshot().wheel[0].fresh && odom.snapshot().wheel[1].fresh);
}

void testMotorCanLinkArmRequiresNewCan1ReportsAfterTransmission()
{
    MotorLinkFixture f;
    f.completeBootStops();
    assert(f.beginCycle(10, OperatorEvents{true}) == RobotState::Arming);
    for (unsigned box = 0; box < 3u; ++box) {
        f.expectFrame(box, 3, static_cast<std::uint8_t>(11u + box));
        f.can1.complete(box, CAN_TSR_TXOK0, 11);
    }
    f.can1.pushRx(f.reportId(11, 2), 11, true, 8);
    assert(f.beginCycle(12) == RobotState::Arming);
    assert(f.motors.rs06ArmFeedback(12).axes[0] == Rs06ArmStatus::Pending);
    f.expectFrame(0, 3, 14);
    f.can1.complete(0, CAN_TSR_TXOK0, 13);
    for (std::uint8_t motorId : {11, 12, 13}) {
        f.can1.pushRx(f.reportId(motorId, 2), 14, true, 8);
    }
    f.can2.pushRx(f.reportId(14, 2), 14, true, 8);
    // 3軸が確認済みでも、最後の軸の報告がCAN2ならARMを完了できない。
    assert(f.beginCycle(15) == RobotState::Arming);
    assert(f.motors.rs06ArmFeedback(15).axes[2] == Rs06ArmStatus::Confirmed);
    assert(f.motors.rs06ArmFeedback(15).axes[3] == Rs06ArmStatus::Pending);
    f.can1.pushRx(f.reportId(14, 2), 16, true, 8);
    assert(f.beginCycle(16) == RobotState::Balancing);
}

void testMotorCanLinkDisarmDropsUnacceptedEnableAndWaitsForSameAxis()
{
    MotorLinkFixture f;
    f.completeBootStops();
    // Type 3を1軸だけ受付し、残る候補がFIFOに残らない条件を作る。
    f.can1.registers.TSR.bits = CAN_TSR_TME0;
    assert(f.beginCycle(10, OperatorEvents{true}) == RobotState::Arming);
    f.expectFrame(0, 3, 11);
    const auto unaccepted = f.motors.pendingRs06Requests()[1];
    assert(unaccepted && unaccepted->command == Rs06Command::Enable);
    f.can1.registers.TSR.bits |= (CAN_TSR_TME0 << 1u) | (CAN_TSR_TME0 << 2u);
    assert(f.beginCycle(11, OperatorEvents{false, true}) == RobotState::Disarmed);
    assert(!f.motors.pendingRs06Requests()[0]);
    assert(f.service(11).result == ScheduledTxResult::Accepted);
    f.expectFrame(1, 4, 12);
    assert(f.service(11).result == ScheduledTxResult::Accepted);
    f.expectFrame(2, 4, 13);
    f.can1.complete(1, CAN_TSR_TXOK0, 12);
    assert(f.beginCycle(12) == RobotState::Disarmed);
    assert(f.service(12).result == ScheduledTxResult::Accepted);
    f.expectFrame(1, 4, 14);
    assert(f.service(12).result == ScheduledTxResult::Busy);

    // メールボックスに入った軸11の有効化の終端を回収してから、その軸を停止する。
    f.can1.complete(0, CAN_TSR_TXOK0, 13);
    assert(f.beginCycle(13) == RobotState::Disarmed);
    assert(f.service(13).result == ScheduledTxResult::Accepted);
    f.expectFrame(0, 4, 11);
    for (unsigned box = 0; box < 3u; ++box) {
        f.can1.complete(box, CAN_TSR_TXOK0, 14);
    }
    assert(f.beginCycle(15) == RobotState::Disarmed);
    assert(!f.motors.rs06ArmFeedback(15).stopPending);
    assert(f.service(15).result == ScheduledTxResult::Idle);
}

void testMotorCanLinkRetryAndFeedbackIgnoreForeignEvents()
{
    MotorLinkFixture f;
    // 発行元の種別が違えば、CAN1で同じ軸番号・シリアルを持つ送信も別物になる。
    const TxToken foreign{1, 11};
    assert(f.tx1.trySubmit({}, foreign, 0) == CanTxSubmitResult::Accepted);
    f.can1.occupy(0);
    assert(f.beginCycle(1) == RobotState::Disarmed);
    const auto first = f.service(1);
    assert(first.result == ScheduledTxResult::Accepted && first.token.serial == foreign.serial);
    f.expectFrame(1, 4, 11);

    // 別バスに同値トークンを送信させ、Routerが正当に配送してもCAN1の軸に届かないことを確認する。
    assert(f.tx2.trySubmit({}, first.token, 2) == CanTxSubmitResult::Accepted);
    f.can2.occupy(0);
    f.can2.complete(0, CAN_TSR_TXOK0, 2);
    f.can1.complete(0, CAN_TSR_TXOK0, 2);
    f.poll();
    assert(f.motors.rs06Status(2)[0].stopPending);
    f.can1.complete(1, CAN_TSR_TERR0, 3);
    assert(f.beginCycle(3) == RobotState::Fault);
    const auto retry = f.service(3);
    assert(retry.result == ScheduledTxResult::Accepted);
    assert(retry.token.origin == first.token.origin && retry.token.serial != first.token.serial);
    f.expectFrame(0, 4, 11);
    CanEvent stale{CanEventKind::Tx, 4};
    stale.token = first.token;
    stale.outcome = CanTxOutcome::Sent;
    f.router.dispatch(CanBus::Can1, stale);
    assert(f.motors.rs06Status(4)[0].stopPending);
    f.can1.complete(0, CAN_TSR_TXOK0, 5);
    f.poll();
    assert(!f.motors.rs06Status(5)[0].stopPending && f.motors.rs06Status(5)[1].stopPending);

    // 無効化は停止送信後に同じバスから届いたType 2でだけ確認できる。
    const auto resetReport = f.reportId(11, 0);
    f.can1.pushRx(resetReport, 5, true, 8);
    f.can2.pushRx(resetReport, 6, true, 8);
    f.poll();
    assert(!f.motors.rs06Status(6)[0].disableConfirmed);
    f.can1.pushRx(resetReport, 7, true, 8);
    f.poll();
    assert(f.motors.rs06Status(7)[0].disableConfirmed);
}

void testMotorCanLinkCan1BusOffRecoveryAndRxLoss()
{
    MotorLinkFixture f;
    assert(f.beginCycle(1) == RobotState::Disarmed);
    f.can2.registers.ESR = CAN_ESR_BOFF;
    f.can2.registers.RF0R.flags = CAN_RF0R_FOVR0;
    f.can2.port.captureInterrupt(2);
    f.poll();
    assert(f.motors.takeObservedFaults(2) == FaultFlags::BusOff);
    f.can2.registers.ESR = 0u;
    f.can2.port.captureInterrupt(2);
    f.poll();
    f.can1.registers.ESR = CAN_ESR_BOFF;
    f.can1.port.captureInterrupt(3);
    f.poll();
    assert(f.motors.takeObservedFaults(3) == FaultFlags::BusOff);
    assert(!f.motors.pendingRs06Requests()[0]);
    assert(f.service(3).result == ScheduledTxResult::Busy);
    f.can1.registers.ESR = 0u;
    f.can1.port.captureInterrupt(4);
    f.poll();
    assert(f.motors.takeObservedFaults(4) == FaultFlags::None);
    assert(f.service(4).result == ScheduledTxResult::Accepted);
    f.expectFrame(0, 4, 11); // バスオフの間も停止枠を保持していた。

    f.can1.registers.RF0R.flags = CAN_RF0R_FOVR0;
    f.can1.port.captureInterrupt(5);
    f.poll();
    assert(f.motors.takeObservedFaults(6) == FaultFlags::ExternalFault);
    assert(f.motors.takeObservedFaults(6) == FaultFlags::None);
}


void testMotorFramePlanCan1Can2StopPriority()
{
    // 6軸の数値fixtureを既存コーデックで実フレーム形式に変換し、
    // 同じスケジューラへ投入する。HALはスタブで実CAN送信は行わない。
    MotorCommandConfig calibration{};
    for (auto& axis : calibration.rs06) {
        axis = {2.0f, 1.0f, 3.0f, true};
    }
    for (auto& axis : calibration.c620) {
        axis = {0.4f, 1.0f, 3.0f, true};
    }
    MotorCommand motor{calibration};
    BalanceControlOutput torques{};
    torques.jointTorqueNm = {2.0f, 2.0f, 2.0f, 2.0f};
    torques.wheelTorqueNm = {0.4f, -0.4f};
    MotorFramePlan planner{{11, 12, 13, 14}};
    const auto converted = motor.evaluate(torques);
    const RobotSequenceResult driving{RobotState::Balancing, DesiredMotorState::DriveAllowed, 3};
    const auto planned = planner.compose(driving, converted);
    assert(planned.status == MotorFramePreview::Status::Ready);

    auto frame = [](const BMotorCanFrame& c) {
        CanFrame copy{c.id, {}, c.dlc, c.extended};
        std::copy_n(c.data, copy.data.size(), copy.data.begin());
        return copy;
    };

    Fixture can1, can2;
    Recorder sink;
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    CanRouter router{sink, &tx1, &tx2};
    PeriodicTxBatch rsRequests{};
    rsRequests.generation = 1;
    rsRequests.expiresAtUs = 2000;
    for (std::size_t axis = 0; axis < 4; ++axis) {
        rsRequests.requests[axis] =
            PeriodicTxRequest{frame(planned.can1Type1[axis]), {axis + 1u, 77}};
    }
    PeriodicTxBatch c620Requests{};
    c620Requests.generation = 1;
    c620Requests.expiresAtUs = 2000;
    c620Requests.requests[0] = PeriodicTxRequest{frame(planned.can2Current), {1, 88}};
    assert(tx1.replacePeriodic(rsRequests, 1000) == PeriodicBatchResult::Queued);
    assert(tx2.replacePeriodic(c620Requests, 1000) == PeriodicBatchResult::Queued);
    assert(tx1.service(1000).result == ScheduledTxResult::Accepted);
    assert((can1.registers.sTxMailBox[0].TIR & CAN_TI0R_IDE) != 0);
    can1.occupy(0);
    assert(tx2.service(1000).result == ScheduledTxResult::Accepted);
    assert((can2.registers.sTxMailBox[0].TIR & CAN_TI0R_IDE) == 0);
    assert(can2.registers.sTxMailBox[0].TDHR == 0); // 未搭載ID3/4=ゼロ
    can2.occupy(0);

    // 今周期の停止要求が入ったら未受付の残り3軸は消え、
    // Type4が次の空き枠で通常Type1より先に受付される。
    BMotorCanFrame stop{};
    assert(b_rs06_control_frame(11, 0x5a, false, &stop));
    assert(tx1.enqueueStop(0, frame(stop), {20, 99}) == StopEnqueueResult::Queued);
    const auto acceptedStop = tx1.service(1001);
    assert(acceptedStop.result == ScheduledTxResult::Accepted);
    assert(((can1.registers.sTxMailBox[1].TIR >> 27u) & 0x1fu) == 4u);
    can1.occupy(1);

    can1.complete(0, CAN_TSR_TXOK0, 1002);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(tx1.service(1002).result == ScheduledTxResult::Busy);
    can1.complete(1, CAN_TSR_TXOK0, 1003);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(tx1.service(1004).result == ScheduledTxResult::Idle);
    // 停止枠が解放されても旧世代の残りType1は復活しない。
    assert((can1.registers.sTxMailBox[2].TIR & CAN_TI0R_TXRQ) == 0);

    // CAN2も次世代のゼロ電流へ切り替え、前世代の非ゼロを再投入しない。
    const auto disarmed = planner.compose(
        {RobotState::Fault, DesiredMotorState::StopAll, 3}, converted);
    assert(disarmed.status == MotorFramePreview::Status::ZeroCurrentOnly);
    c620Requests.generation = 2;
    c620Requests.expiresAtUs = 3000;
    c620Requests.requests[0] = PeriodicTxRequest{frame(disarmed.can2Current), {2, 88}};
    assert(tx2.replacePeriodic(c620Requests, 1004) == PeriodicBatchResult::Queued);
    can2.complete(0, CAN_TSR_TXOK0, 1005);
    pollCanEvents(&can1.port, &can2.port, router);
    assert(tx2.service(1006).result == ScheduledTxResult::Accepted);
    assert(can2.registers.sTxMailBox[0].TDLR == 0u);
    assert(can2.registers.sTxMailBox[0].TDHR == 0u);
}

// 実機値ではない。既存SILと同じ閉リンクfixtureをCAN・IMUの実装と結合する。
// 2/4msの生成周期はこの試験の上位時計が所有し、Schedulerには絶対期限を渡す。
void testIntegratedObservationAndFrameScheduling()
{
    using namespace std::chrono_literals;
    Fixture can1, can2;
    constexpr std::uint8_t host=0xa6;
    MotorService motors{{{11,12,13,14},host}, {{{4000,true},{4000,true}}}};
    CanTxScheduler tx1{can1.port}, tx2{can2.port};
    MotorCanLink link{motors,tx1};
    CanRouter router{link,&tx1,&tx2};
    auto poll=[&]{pollCanEvents(&can1.port,&can2.port,router);};
    auto frame=[](const BMotorCanFrame& b) {
        CanFrame c{b.id,{},b.dlc,b.extended};
        std::copy_n(b.data,c.data.size(),c.data.begin());return c;
    };
    StateEstimatorConfig estConfig{};
    for(auto& leg:estConfig.legs) {
        leg.motor={{0,0},{0,0},{1,1},1,1,true};
        leg.fixture={0.2f,0.15f,0.15f,0.25f,0.25f,0.02f,0.01f,-1};
    }
    estConfig.wheels={{{0.1f,10.0f,1.0f,true},{0.1f,10.0f,-1.0f,true}}};
    BalanceControlConfig controlConfig{};
    controlConfig.lqr.verified=true;
    controlConfig.lqr.max_position_error_m=0.2f;
    controlConfig.lqr.gain[B_OUTPUT_WHEEL_TOTAL][B_STATE_PITCH]=2;
    controlConfig.lqr.gain[B_OUTPUT_HIP_TOTAL][B_STATE_PITCH]=3;
    controlConfig.legPd={100,10,2,0.5f,4,true};
    controlConfig.yawPi={3,4,2,true};
    controlConfig.wheelTorqueLimitNm={5,5};
    MotorCommandConfig motorConfig{};
    for(auto& a:motorConfig.rs06) a={1.5f,1,4,true};
    for(auto& a:motorConfig.c620) a={0.4f,1,4,true};
    StateEstimator estimator{estConfig};
    RobotSequence sequence{{100000,true}};
    BalanceControl control{controlConfig};
    MotorCommand converter{motorConfig};
    ControlCycle cycle{estimator,sequence,control,converter};
    MotorFramePlan planner{{11,12,13,14}};
    BAttitudeFilterConfig imuConfig{};
    imuConfig.mount={{0,1,2},{1,1,1},true};
    imuConfig.bias_samples_required=2;
    imuConfig.gravity_min_m_s2=9;
    imuConfig.gravity_max_m_s2=11;
    imuConfig.gyro_alpha=0.9f;
    imuConfig.max_sample_dt_s=0.05f;
    imuConfig.verified=true;
    ImuAttitudeSource imu{imuConfig};
    TimedImuSample sensor{{{-0.98f,0,9.75f},{0,0,0}},0,0,true};
    assert(imu.accept(sensor,true,0));
    sensor.earliestUs=sensor.completedAtUs=1000;
    assert(imu.accept(sensor,true,1000));

    auto feed=[&](std::uint64_t now,unsigned mode) {
        // 角度は約-π/2、速度・トルクは量子化中央。実デコーダで復号する。
        for(unsigned axis=0;axis<4;++axis)
            can1.pushRx((2u<<24)|(mode<<22)|((11u+axis)<<8)|host,
                        now,true,8,0x0080ff6fu,0xfa000080u);
        can2.pushRx(0x201,now,false,8,0x0000e803u);
        can2.pushRx(0x202,now,false,8,0x0000e803u);
        poll();
        sensor.earliestUs=sensor.completedAtUs=now;
        assert(imu.accept(sensor,false,now));
    };
    auto step=[&](std::uint64_t now,bool arm,FaultFlags faults=FaultFlags::None) {
        ControlTickInput in{};
        in.firstElapsed=2000us;
        in.observations={imu.attitude(now),motors.feedback(now)};
        in.interlocks={true,true,true,true,true,true,MechanismSafety::Safe};
        in.operations.arm=arm;
        in.armFeedback=motors.rs06ArmFeedback(now);
        in.observedFaults=motors.takeObservedFaults(now)|faults;
        auto result=cycle.step(in);
        motors.applySequence(result.sequence,now);
        return result;
    };
    // 模擬メールボックスの受付を物理送信完了と区別し、終端をRouterへ戻す。
    auto completeAccepted=[&](Fixture& bus,std::uint64_t now) {
        unsigned n=0;
        for(unsigned box=0;box<3;++box) {
            if ((bus.registers.sTxMailBox[box].TIR&CAN_TI0R_TXRQ)==0) continue;
            bus.occupy(box);
            bus.complete(box,CAN_TSR_TXOK0,now);
            ++n;
        }
        poll();return n;
    };
    auto drain1=[&](std::uint64_t now) {
        unsigned count=0;
        for(unsigned i=0;i<12;++i) {
            auto progress=tx1.service(now);
            link.onScheduledTxProgress(progress,now);
            count+=completeAccepted(can1,now+1);
            if(progress.result==ScheduledTxResult::Idle) break;
        }
        return count;
    };
    auto drain2=[&](std::uint64_t now) {
        const auto p=tx2.service(now);
        assert(p.result==ScheduledTxResult::Accepted);
        assert(completeAccepted(can2,now+1)==1);
    };
    feed(2000,0);
    assert(step(2000,false).sequence.state==RobotState::Disarmed);
    link.submitPending(2000);
    assert(drain1(2000)==4);
    feed(4000,0);
    auto arming=step(4000,true);
    assert(arming.sequence.state==RobotState::Arming);
    assert(arming.estimate && !arming.motorCommandPreview);
    for(unsigned i=0;i<4;++i) {
        link.submitPending(4000+i*2);
        completeAccepted(can1,4001+i*2);
    }
    feed(6000,2);
    assert(step(6000,false).sequence.state==RobotState::Balancing);

    unsigned rsFrames=0,wheelFrames=0;
    std::uint64_t generation1=0,generation2=0,serial=1000;
    for(unsigned tick=0;tick<100;++tick) {
        const auto now=8000u+2000u*tick;
        feed(now,2);
        const auto result=step(now,false);
        assert(result.sequence.state==RobotState::Balancing && result.motorCommandPreview);
        const auto plan=planner.compose(result.sequence,*result.motorCommandPreview);
        assert(plan.status==MotorFramePreview::Status::Ready && plan.can1Count==4);
        PeriodicTxBatch wheel{};
        wheel.generation=++generation2;wheel.expiresAtUs=now+2000;
        wheel.requests[0]=PeriodicTxRequest{frame(plan.can2Current),{++serial,88}};
        assert(tx2.replacePeriodic(wheel,now)==PeriodicBatchResult::Queued);
        if(tick!=99) {drain2(now);++wheelFrames;}
        // 4msごと。最後の世代は未受付のまま残し、故障時の破棄を検証する。
        if(tick%2==0) {
            PeriodicTxBatch legs{};
            legs.generation=++generation1;legs.expiresAtUs=now+4000;
            for(unsigned axis=0;axis<4;++axis)
                legs.requests[axis]=PeriodicTxRequest{frame(plan.can1Type1[axis]),{++serial,77}};
            assert(tx1.replacePeriodic(legs,now)==PeriodicBatchResult::Queued);
            if(tick!=98) rsFrames+=drain1(now);
        }
    }
    assert(generation1==50 && generation2==100 && rsFrames==196 && wheelFrames==99);
    // 次周期のE-STOPで旧非ゼロ待ちを破棄。送信済みのフレームの取り消しは主張しない。
    constexpr std::uint64_t faultAt=208000;
    feed(faultAt,2);
    const auto failed=step(faultAt,false,FaultFlags::EmergencyStop);
    assert(failed.sequence.state==RobotState::Fault && !failed.motorCommandPreview);
    link.submitPending(faultAt);
    const auto stop=planner.compose(failed.sequence,MotorCommandPreview{});
    assert(stop.status==MotorFramePreview::Status::ZeroCurrentOnly);
    PeriodicTxBatch zero{};
    zero.generation=++generation2;zero.expiresAtUs=faultAt+2000;
    zero.requests[0]=PeriodicTxRequest{frame(stop.can2Current),{++serial,88}};
    assert(tx2.replacePeriodic(zero,faultAt)==PeriodicBatchResult::Queued);
    for(unsigned axis=0;axis<4;++axis) {
        const auto progress=tx1.service(faultAt);
        assert(progress.result==ScheduledTxResult::Accepted);
        link.onScheduledTxProgress(progress,faultAt);
        assert(((can1.registers.sTxMailBox[0].TIR>>27)&0x1f)==4);
        assert(completeAccepted(can1,faultAt+1)==1);
    }
    assert(tx1.service(faultAt+2).result==ScheduledTxResult::Idle);
    const auto accepted=tx2.service(faultAt);
    assert(accepted.result==ScheduledTxResult::Accepted);
    assert(can2.registers.sTxMailBox[0].TDLR==0 && can2.registers.sTxMailBox[0].TDHR==0);
    assert(completeAccepted(can2,faultAt+1)==1);
    assert(tx2.service(faultAt+2).result==ScheduledTxResult::Idle);
}

void testDiagnosticPartsWithScheduler()
{
    // ここでのCAN ID/DLCは模擬転送の容器にすぎず、PC向け通信仕様ではない。
    Fixture can2;
    Recorder sink;
    CanTxScheduler scheduler{can2.port};
    CanRouter router{sink,nullptr,&scheduler};
    std::array<DiagnosticSlot,2> storage{};
    DiagnosticsService diag{storage.data(),storage.size(),nullptr,0,90};
    assert(diag.pushFault({FaultFlags::BusOff,FaultFlags::BusOff,FaultFlags::None,1000},1000));
    std::uint64_t serial=0;
    for (unsigned attempt=0;attempt<4;++attempt) {
        auto p=diag.nextPending();
        assert(p && p->frameIndex==(attempt==0 ? 0 : attempt-1));
        const TxToken token{++serial,90};
        const CanFrame opaque{0x7ff,{},0,false};
        const auto now=1000u+attempt*10u;
        assert(scheduler.trySubmit(opaque,token,now)==CanTxSubmitResult::Accepted);
        assert(diag.onAccepted(*p,token));
        assert(diag.summary().importantPending==1 && !diag.nextPending());
        can2.occupy(0);
        can2.complete(0,attempt==0 ? CAN_TSR_TERR0 : CAN_TSR_TXOK0,now+1);
        pollCanEvents(nullptr,&can2.port,router);
        const auto& terminal=sink.entries[sink.count-1];
        assert(terminal.bus==CanBus::Can2 && terminal.token==token);
        assert(diag.onTerminal(terminal.token,terminal.outcome));
    }
    assert(diag.summary().importantPending==0 && !diag.nextPending());
}

} // namespace

int main()
{
    testBusAndEventTypeRemainPaired();
    testBothPortsReturnAtLimitAndResumeLater();
    testRoutingUsesBusIdentifierFormatAndDlc();
    testCan2ProtocolReceiveDirections();
    testRoutingFromBothPortQueues();
    testPowerStatusThroughCanQueue();
    testTerminalOnlyReleasesTheOriginatingBus();
    testSchedulerIgnoresUnacceptedAndStaleEvents();
    testNonSendTerminalEventsReleaseTheEntry();
    testFullCan1ReplacesTheFourthPeriodicFrame();
    testFourthPeriodicFrameCanUseTheFirstReleasedMailbox();
    testPeriodicDeadlineAndAtomicValidation();
    testPeriodicGenerationIsIndependentOnCan2();
    testNormalEventsFinishInOrderBeforePeriodicTx();
    testNormalEventFifoFullExpiryAndWrap();
    testFourStopsHavePriorityWhileTheNormalFifoIsFull();
    testFailedStopsRequireNewTokensAndAreIndependentBetweenBuses();
    testStopWaitSurvivesNotReadyAndBusOff();
    testMotorCanLinkCollectsSixAxesForOneCycle();
    testCan2ProtocolDoesNotChangeWheelObservations();
    testWheelContinuityThroughCanFifo();
    testMotorCanLinkArmRequiresNewCan1ReportsAfterTransmission();
    testMotorCanLinkDisarmDropsUnacceptedEnableAndWaitsForSameAxis();
    testMotorCanLinkRetryAndFeedbackIgnoreForeignEvents();
    testMotorCanLinkCan1BusOffRecoveryAndRxLoss();
    testMotorFramePlanCan1Can2StopPriority();
    testIntegratedObservationAndFrameScheduling();
    testDiagnosticPartsWithScheduler();
    std::puts("PASS: F446 CAN dispatch, TX scheduling, six-axis feedback and RS06 arm/stop");
}
