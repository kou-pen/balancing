#include "balancing_tx_scheduler.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>

/* 実機の値ではなく、決定論的なホスト故障注入の閾値。 */
static const BSchedConfig TEST_CONFIG = {9000u, 6000u};

static BTxScheduler fresh_scheduler(BSchedConfig config)
{
    BTxScheduler scheduler;
    assert(b_sched_init(&scheduler, &config));
    for (uint8_t i = 0; i < 4u; ++i) {
        assert(b_sched_feedback(&scheduler, B_SCHED_CAN1_RS06, i, 0u));
    }
    for (uint8_t i = 0; i < 2u; ++i) {
        assert(b_sched_feedback(&scheduler, B_SCHED_CAN2_C620, i, 0u));
    }
    return scheduler;
}

static void complete_one(BTxScheduler *s, BSchedBus bus, uint8_t mailbox,
                         uint64_t t)
{
    assert(b_sched_completed(s, bus, mailbox, t));
}

static void wheel_send(BTxScheduler *s, uint64_t t, bool expect_zero)
{
    BSchedIntent send;
    assert(b_sched_next(s, B_SCHED_CAN2_C620, &send));
    assert(send.axis == 0xffu && send.zero_current == expect_zero);
    assert(b_sched_accepted(s, &send, 0u, t));
    assert(!b_sched_next(s, B_SCHED_CAN2_C620, &send));
    complete_one(s, B_SCHED_CAN2_C620, 0u, t + 10u);
}

static void rs_send_four(BTxScheduler *s, uint64_t t, uint32_t expected_generation)
{
    BSchedIntent send;
    for (unsigned i = 0u; i < 3u; ++i) {
        assert(b_sched_next(s, B_SCHED_CAN1_RS06, &send));
        assert(send.axis == i && send.generation == expected_generation);
        assert(b_sched_accepted(s, &send, (uint8_t)i, t + 20u));
    }
    assert(b_sched_next(s, B_SCHED_CAN1_RS06, &send));
    assert(send.axis == 3u);
    assert(!b_sched_accepted(s, &send, 0u, t + 20u)); /* mailbox占有 */
    complete_one(s, B_SCHED_CAN1_RS06, 0u, t + 40u);
    assert(b_sched_accepted(s, &send, 0u, t + 41u)); /* 完了IRQで4件目を受付 */
    assert(!b_sched_next(s, B_SCHED_CAN1_RS06, &send));
    complete_one(s, B_SCHED_CAN1_RS06, 1u, t + 60u);
    complete_one(s, B_SCHED_CAN1_RS06, 2u, t + 61u);
    complete_one(s, B_SCHED_CAN1_RS06, 0u, t + 62u);
}

static void test_periods_and_three_mailboxes(void)
{
    BTxScheduler s = fresh_scheduler(TEST_CONFIG);
    for (uint64_t t = 0u; t <= 8000u; t += 2000u) {
        b_sched_tick(&s, t, true);
        assert(!s.fault_requested);
        wheel_send(&s, t, false);
        if (t % 4000u == 0u) {
            rs_send_four(&s, t + 10u, (uint32_t)(t / 4000u + 1u));
        } else {
            BSchedIntent unused;
            assert(!b_sched_next(&s, B_SCHED_CAN1_RS06, &unused));
        }
    }
    for (unsigned i = 0; i < 4u; ++i) {
        assert(s.rs_tx_valid[i]);
    }
    assert(s.wheel_tx_valid);
    assert(s.rs_dropped_pending == 0u);
    assert(s.wheel_dropped_pending == 0u);
    assert(s.rs_late_completions == 0u);
    puts("PASS cadence: 2ms CAN2 (5); 4ms CAN1 (3x4), 3 mailboxes");
}

static void test_old_pending_dropped_at_next_period(void)
{
    BTxScheduler s = fresh_scheduler((BSchedConfig){20000u, 20000u});
    BSchedIntent send;
    b_sched_tick(&s, 0u, true);
    for (unsigned i = 0u; i < 3u; ++i) {
        assert(b_sched_next(&s, B_SCHED_CAN1_RS06, &send));
        assert(b_sched_accepted(&s, &send, (uint8_t)i, 0u));
    }
    assert(b_sched_next(&s, B_SCHED_CAN1_RS06, &send) && send.axis == 3u);
    const BSchedIntent obsolete = send;
    b_sched_tick(&s, 2000u, true);
    b_sched_tick(&s, 4000u, true);
    assert(s.rs_dropped_pending == 1u);
    assert(!b_sched_accepted(&s, &obsolete, 0u, 4000u));
    /* 前世代が実ハードに残る場合、新世代を重ねない。 */
    assert(!b_sched_next(&s, B_SCHED_CAN1_RS06, &send));
    complete_one(&s, B_SCHED_CAN1_RS06, 0u, 4100u);
    complete_one(&s, B_SCHED_CAN1_RS06, 1u, 4101u);
    complete_one(&s, B_SCHED_CAN1_RS06, 2u, 4102u);
    assert(s.rs_late_completions == 3u);
    assert(!s.rs_tx_valid[0]);
    assert(b_sched_next(&s, B_SCHED_CAN1_RS06, &send));
    assert(send.axis == 0u && send.generation == 2u);
    puts("PASS drop: obsolete pending rejected, old in-flight completion not fresh");
}

static void test_hal_acceptance_is_not_tx_completion(void)
{
    BTxScheduler s = fresh_scheduler(TEST_CONFIG);
    b_sched_tick(&s, 0u, true);
    BSchedIntent send;
    assert(b_sched_next(&s, B_SCHED_CAN1_RS06, &send));
    assert(send.axis == 0u);
    assert(b_sched_accepted(&s, &send, 0u, 0u));
    assert(!s.rs_tx_valid[0]); /* HAL_OKでは実際のCAN送信はまだ完了しない。 */
    assert(!b_sched_completed(&s, B_SCHED_CAN2_C620, 0u, 10u));
    assert(!s.rs_tx_valid[0]);
    complete_one(&s, B_SCHED_CAN1_RS06, 0u, 20u);
    assert(s.rs_tx_valid[0] && s.rs_tx_done_us[0] == 20u);
    puts("PASS TX: HAL acceptance and mailbox completion are distinct");
}

static void test_phase_does_not_catch_up_burst(void)
{
    BTxScheduler s;
    assert(b_sched_init(&s, &TEST_CONFIG));
    b_sched_tick(&s, 0u, false);
    assert(s.rs_generation == 1u);
    b_sched_tick(&s, 5000u, false); /* 2 msティック欠落、ただしDISARMED。 */
    assert(s.rs_generation == 2u); /* 4 ms枠を一度だけ進める。 */
    b_sched_tick(&s, 8000u, false);
    assert(s.rs_generation == 3u); /* 0/4/8 msの位相を維持。 */
    assert(!s.fault_requested);
    puts("PASS timing: late tick does not replay missed RS command bursts");
}

static void test_no_rx_does_not_hide_tx_fault(void)
{
    BTxScheduler s = fresh_scheduler(TEST_CONFIG);
    b_sched_tick(&s, 0u, true);
    BSchedIntent c620;
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &c620));
    assert(b_sched_accepted(&s, &c620, 0u, 0u));
    /* 毎回新鮮な受信が来ても、送信完了IRQは来ない。 */
    for (uint64_t t = 2000u; t <= 6000u; t += 2000u) {
        for (uint8_t i = 0u; i < 4u; ++i) {
            assert(b_sched_feedback(&s, B_SCHED_CAN1_RS06, i, t));
        }
        for (uint8_t i = 0u; i < 2u; ++i) {
            assert(b_sched_feedback(&s, B_SCHED_CAN2_C620, i, t));
        }
        b_sched_tick(&s, t, true);
    }
    assert(s.fault_requested);
    assert(s.fault_flags & B_SCHED_FAULT_C620_TX);
    assert(!(s.fault_flags & (B_SCHED_FAULT_RS_RX | B_SCHED_FAULT_WHEEL_RX)));
    BSchedIntent send;
    assert(!b_sched_next(&s, B_SCHED_CAN1_RS06, &send));
    /* 既に受理されたCAN2指令はモデルから物理的には取り消せない。 */
    assert(!b_sched_next(&s, B_SCHED_CAN2_C620, &send));
    complete_one(&s, B_SCHED_CAN2_C620, 0u, 6010u);
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &send) && send.zero_current);
    puts("PASS tx watchdog: fresh RX cannot mask missing TX complete");
}

static void test_rs_tx_watchdog_independent_per_axis(void)
{
    BTxScheduler s = fresh_scheduler((BSchedConfig){9000u, 30000u});
    b_sched_tick(&s, 0u, true);
    /* 一台の送信を永遠に完了させない。全ての受信は新鮮。 */
    for (uint64_t t = 2000u; t <= 10000u; t += 2000u) {
        for (uint8_t i = 0u; i < 4u; ++i) {
            assert(b_sched_feedback(&s, B_SCHED_CAN1_RS06, i, t));
        }
        for (uint8_t i = 0u; i < 2u; ++i) {
            assert(b_sched_feedback(&s, B_SCHED_CAN2_C620, i, t));
        }
        b_sched_tick(&s, t, true);
    }
    assert(s.fault_requested);
    assert(s.fault_flags & B_SCHED_FAULT_RS_TX);
    assert(!(s.fault_flags & B_SCHED_FAULT_RS_RX));
    puts("PASS RS per-axis TX watchdog");
}

static void test_rx_timeout_separate_from_tx(void)
{
    BTxScheduler s = fresh_scheduler((BSchedConfig){100000u, 100000u});
    b_sched_tick(&s, 0u, true);
    /* 車輪の一軸のみ更新欠落、他軸は正常のまま。 */
    for (uint64_t t = 2000u; t <= 20000u; t += 2000u) {
        for (uint8_t i = 0u; i < 4u; ++i) {
            assert(b_sched_feedback(&s, B_SCHED_CAN1_RS06, i, t));
        }
        assert(b_sched_feedback(&s, B_SCHED_CAN2_C620, 0u, t));
        b_sched_tick(&s, t, true);
    }
    assert(s.fault_requested);
    assert(s.fault_flags & B_SCHED_FAULT_WHEEL_RX);
    assert(!(s.fault_flags & B_SCHED_FAULT_C620_TX));

    s = fresh_scheduler((BSchedConfig){100000u, 100000u});
    b_sched_tick(&s, 0u, true);
    for (uint64_t t = 2000u; t <= 30000u; t += 2000u) {
        for (uint8_t i = 0u; i < 2u; ++i) {
            assert(b_sched_feedback(&s, B_SCHED_CAN2_C620, i, t));
        }
        b_sched_tick(&s, t, true);
    }
    assert(s.fault_flags & B_SCHED_FAULT_RS_RX);
    puts("PASS RX timeouts: wheel 20ms, RS06 30ms");
}

static void test_control_gap_and_clock_regression(void)
{
    BTxScheduler s = fresh_scheduler((BSchedConfig){20000u, 20000u});
    b_sched_tick(&s, 0u, true);
    b_sched_tick(&s, 6000u, true);
    assert(s.fault_requested);
    assert(s.fault_flags & B_SCHED_FAULT_CONTROL_GAP);
    assert(!s.drive_active);

    s = fresh_scheduler((BSchedConfig){20000u, 20000u});
    b_sched_tick(&s, 0u, true);
    b_sched_tick(&s, 2000u, true);
    b_sched_tick(&s, 1999u, true);
    assert(s.fault_flags & B_SCHED_FAULT_CLOCK);
    assert(s.fault_requested);
    puts("PASS control faults: exact 6ms gap and clock regression");
}

static void test_disarmed_zero_and_busoff(void)
{
    BTxScheduler s;
    assert(!b_sched_init(&s, &(BSchedConfig){0u, 9000u}));
    assert(b_sched_init(&s, &TEST_CONFIG));
    for (uint64_t t = 0u; t <= 6000u; t += 2000u) {
        b_sched_tick(&s, t, false);
        wheel_send(&s, t, true);
        BSchedIntent send;
        assert(!b_sched_next(&s, B_SCHED_CAN1_RS06, &send));
    }
    assert(!s.fault_requested);
    b_sched_bus_off(&s, 6500u);
    assert(s.fault_requested && (s.fault_flags & B_SCHED_FAULT_BUS_OFF));
    BSchedIntent stop;
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &stop) && stop.zero_current);
    puts("PASS disarmed C620=0, bus-off fault");
}

static void test_c620_backpressure_and_generation(void)
{
    BTxScheduler s;
    assert(b_sched_init(&s, &TEST_CONFIG));
    BSchedIntent old, current;
    b_sched_tick(&s, 0u, false);
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &old));
    b_sched_tick(&s, 2000u, false);
    assert(s.wheel_dropped_pending == 1u);
    assert(!b_sched_accepted(&s, &old, 0u, 2000u));
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &current));
    assert(current.generation != old.generation);
    assert(b_sched_accepted(&s, &current, 0u, 2000u));
    b_sched_tick(&s, 4000u, false);
    /* 旧フレームがハードで完了するまで、次の電流フレームを重ねない。 */
    assert(!b_sched_next(&s, B_SCHED_CAN2_C620, &current));
    complete_one(&s, B_SCHED_CAN2_C620, 0u, 4100u);
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &current));
    assert(current.zero_current && current.generation == 3u);
    puts("PASS C620: one in flight and no stale software queue");
}

static void test_manual_disarm_and_fault_latch(void)
{
    BTxScheduler s = fresh_scheduler(TEST_CONFIG);
    BSchedIntent current;
    b_sched_tick(&s, 0u, true);
    assert(b_sched_next(&s, B_SCHED_CAN1_RS06, &current));
    b_sched_tick(&s, 2000u, false);
    assert(!s.fault_requested); /* 通常のDISARMではFAULTにしない。 */
    assert(!b_sched_next(&s, B_SCHED_CAN1_RS06, &current));
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &current));
    assert(current.zero_current);

    b_sched_bus_off(&s, 2100u);
    b_sched_tick(&s, 4000u, true);
    assert(s.fault_requested && !s.drive_active);
    assert(!b_sched_next(&s, B_SCHED_CAN1_RS06, &current));
    assert(b_sched_next(&s, B_SCHED_CAN2_C620, &current));
    assert(current.zero_current);
    puts("PASS disarm: RS06 stopped, fault stays latched without auto-clear");
}

int main(void)
{
    test_periods_and_three_mailboxes();
    test_old_pending_dropped_at_next_period();
    test_hal_acceptance_is_not_tx_completion();
    test_phase_does_not_catch_up_burst();
    test_no_rx_does_not_hide_tx_fault();
    test_rs_tx_watchdog_independent_per_axis();
    test_rx_timeout_separate_from_tx();
    test_control_gap_and_clock_regression();
    test_disarmed_zero_and_busoff();
    test_c620_backpressure_and_generation();
    test_manual_disarm_and_fault_latch();
    puts("PASS: issue #5 host CAN scheduler model");
    return 0;
}
