#include "balancing/app/CanEventPoll.hpp"
#include "balancing/communication/can/CanPort.hpp"
#include "balancing/communication/can/CanRouter.hpp"

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

    void finish(TxToken token, std::uint64_t atUs)
    {
        assert(port.trySend({}, token, atUs) == CanTxSubmitResult::Accepted);
        registers.TSR.bits |= CAN_TSR_TME0 | CAN_TSR_RQCP0 | CAN_TSR_TXOK0;
        port.captureInterrupt(atUs);
    }
};

struct Entry {
    CanBus bus{};
    CanEventKind kind{};
    std::uint64_t atUs{};
    TxToken token{};
    std::uint32_t id{};
    CanRxTarget target{CanRxTarget::Unmatched};
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
        assert(outcome == CanTxOutcome::Sent);
        record({bus, CanEventKind::Tx, atUs, token});
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

} // namespace

int main()
{
    testBusAndEventTypeRemainPaired();
    testBothPortsReturnAtLimitAndResumeLater();
    testRoutingUsesBusIdentifierFormatAndDlc();
    testRoutingFromBothPortQueues();
    std::puts("PASS: F446 bounded CAN dispatch and RX routing");
}
