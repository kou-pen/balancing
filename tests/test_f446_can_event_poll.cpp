#include "balancing/app/CanEventPoll.hpp"
#include "balancing/communication/can/CanPort.hpp"
#include "balancing/communication/can/CanRouter.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
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

    void pushRx(std::uint32_t id, std::uint64_t atUs, bool extended = false, std::uint32_t dlc = 1)
    {
        auto& fifo = registers.RF0R;
        assert(fifo.count < 3u);
        const auto rawId = extended ? (id << 3u) | CAN_RI0R_IDE : id << 21u;
        registers.sFIFOMailBox[fifo.count++] = {rawId, dlc, 0x45u, 0};
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

} // namespace

int main()
{
    testBusAndEventTypeRemainPaired();
    testBothPortsReturnAtLimitAndResumeLater();
    testRoutingUsesBusIdentifierFormatAndDlc();
    testRoutingFromBothPortQueues();
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
    std::puts("PASS: F446 CAN dispatch, periodic replacement, normal FIFO and stop priority");
}
