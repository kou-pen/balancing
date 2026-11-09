#include "balancing_tx_scheduler.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* 全て同一の単調増加マイクロ秒時計。実機のタイミング保証ではない。 */
#define RS_PERIOD_US 4000u
#define CONTROL_GAP_FAULT_US 6000u
#define RS_RX_TIMEOUT_US 30000u
#define WHEEL_RX_TIMEOUT_US 20000u

static int bus_slot(BSchedBus bus)
{
    if (bus == B_SCHED_CAN1_RS06) {
        return 0;
    }
    return bus == B_SCHED_CAN2_C620 ? 1 : -1;
}

static void request_fault(BTxScheduler *s, uint32_t flags)
{
    s->fault_requested = true;
    s->fault_flags |= flags;
    s->drive_active = false;
    for (unsigned i = 0; i < 4u; ++i) {
        if (s->rs_pending[i]) {
            ++s->rs_dropped_pending;
        }
        s->rs_pending[i] = false;
    }
    /* 既にHALへ渡した古いフレームはここでは取り消せない。 */
    if (s->wheel_pending) {
        ++s->wheel_dropped_pending;
    }
    ++s->wheel_generation;
    s->wheel_pending = true;
    s->wheel_zero = true;
}

static bool note_time(BTxScheduler *s, uint64_t now_us)
{
    if (s->event_valid && now_us < s->last_event_us) {
        if (s->drive_active) {
            request_fault(s, B_SCHED_FAULT_CLOCK);
        }
        return false;
    }
    s->event_valid = true;
    s->last_event_us = now_us;
    return true;
}

static bool expired(uint64_t now_us, uint64_t since_us, uint64_t limit_us)
{
    return now_us >= since_us && now_us - since_us >= limit_us;
}

bool b_sched_init(BTxScheduler *s, const BSchedConfig *config)
{
    if (!s) {
        return false;
    }
    /* 以前の有効設定が残っても、再初期化失敗時には必ず無効状態にする。 */
    memset(s, 0, sizeof(*s));
    if (!config || config->rs_tx_timeout_us == 0u ||
        config->c620_tx_timeout_us == 0u) {
        return false;
    }
    s->config = *config;
    s->wheel_zero = true;
    s->initialized = true;
    return true;
}

static void check_freshness(BTxScheduler *s, uint64_t now_us)
{
    if (!s->drive_active) {
        return;
    }
    for (unsigned i = 0; i < 4u; ++i) {
        if (!s->rs_rx_valid[i] ||
            expired(now_us, s->rs_rx_us[i], RS_RX_TIMEOUT_US)) {
            request_fault(s, B_SCHED_FAULT_RS_RX);
            return;
        }
    }
    for (unsigned i = 0; i < 2u; ++i) {
        if (!s->wheel_rx_valid[i] ||
            expired(now_us, s->wheel_rx_us[i], WHEEL_RX_TIMEOUT_US)) {
            request_fault(s, B_SCHED_FAULT_WHEEL_RX);
            return;
        }
    }
    for (unsigned i = 0; i < 4u; ++i) {
        const uint64_t since = s->rs_tx_valid[i] ?
            s->rs_tx_done_us[i] : s->active_since_us;
        if (expired(now_us, since, s->config.rs_tx_timeout_us)) {
            request_fault(s, B_SCHED_FAULT_RS_TX);
            return;
        }
    }
    const uint64_t since = s->wheel_tx_valid ?
        s->wheel_tx_done_us : s->active_since_us;
    if (expired(now_us, since, s->config.c620_tx_timeout_us)) {
        request_fault(s, B_SCHED_FAULT_C620_TX);
    }
}

void b_sched_tick(BTxScheduler *s, uint64_t now_us, bool drive_allowed)
{
    if (!s || !s->initialized || !note_time(s, now_us)) {
        return;
    }
    if (s->tick_valid && s->drive_active &&
        expired(now_us, s->last_tick_us, CONTROL_GAP_FAULT_US)) {
        request_fault(s, B_SCHED_FAULT_CONTROL_GAP);
    }
    s->last_tick_us = now_us;
    if (!s->tick_valid) {
        s->next_rs_due_us = now_us;
        s->tick_valid = true;
    }

    const bool activate = drive_allowed && !s->fault_requested;
    if (!activate) {
        s->drive_active = false;
        for (unsigned i = 0; i < 4u; ++i) {
            if (s->rs_pending[i]) {
                ++s->rs_dropped_pending;
            }
            s->rs_pending[i] = false;
        }
    } else if (!s->drive_active) {
        s->drive_active = true;
        s->active_since_us = now_us;
        for (unsigned i = 0; i < 4u; ++i) {
            s->rs_tx_valid[i] = false;
        }
        s->wheel_tx_valid = false;
    }

    /* CAN2は2 msごとに必ず最新の要求へ置換する。FAULT中は0 A。 */
    if (s->wheel_pending) {
        ++s->wheel_dropped_pending;
    }
    ++s->wheel_generation;
    s->wheel_pending = true;
    s->wheel_zero = !s->drive_active;

    if (now_us >= s->next_rs_due_us) {
        /* 遅れた周期をまとめて送信しない。時間基準の位相を維持する。 */
        const uint64_t phases = (now_us - s->next_rs_due_us) / RS_PERIOD_US + 1u;
        const uint64_t remaining = UINT64_MAX - s->next_rs_due_us;
        const uint64_t advance = phases <= remaining / RS_PERIOD_US ?
            phases * RS_PERIOD_US : remaining;
        s->next_rs_due_us += advance;
        ++s->rs_generation;
        for (unsigned i = 0; i < 4u; ++i) {
            if (s->rs_pending[i]) {
                ++s->rs_dropped_pending;
            }
            s->rs_pending[i] = s->drive_active;
        }
    }

    check_freshness(s, now_us);
}

bool b_sched_next(const BTxScheduler *s, BSchedBus bus, BSchedIntent *intent)
{
    if (!s || !s->initialized || !intent) {
        return false;
    }
    if (bus == B_SCHED_CAN1_RS06 && s->drive_active && !s->fault_requested) {
        /* 前世代が実メールボックスに残る間は新しい世代を重ねない。 */
        for (unsigned m = 0; m < 3u; ++m) {
            if (s->mail[0][m].active &&
                s->mail[0][m].intent.generation != s->rs_generation) {
                return false;
            }
        }
        for (unsigned i = 0; i < 4u; ++i) {
            if (s->rs_pending[i]) {
                *intent = (BSchedIntent){B_SCHED_CAN1_RS06, (uint8_t)i,
                                         s->rs_generation, false};
                return true;
            }
        }
    } else if (bus == B_SCHED_CAN2_C620 && s->wheel_pending) {
        /* CAN2の旧共通電流指令との同時滞留も避ける。 */
        for (unsigned m = 0; m < 3u; ++m) {
            if (s->mail[1][m].active) {
                return false;
            }
        }
        *intent = (BSchedIntent){B_SCHED_CAN2_C620, 0xffu,
                                 s->wheel_generation, s->wheel_zero};
        return true;
    }
    return false;
}

bool b_sched_accepted(BTxScheduler *s, const BSchedIntent *intent,
                      uint8_t mailbox, uint64_t now_us)
{
    if (!s || !s->initialized || !intent || mailbox >= 3u) {
        return false;
    }
    const int bus = bus_slot(intent->bus);
    if (bus < 0 || s->mail[bus][mailbox].active) {
        return false;
    }
    if (bus == 0) {
        if (!s->drive_active || s->fault_requested || intent->axis >= 4u ||
            intent->generation != s->rs_generation ||
            !s->rs_pending[intent->axis] || intent->zero_current) {
            return false;
        }
    } else if (intent->axis != 0xffu ||
               intent->generation != s->wheel_generation ||
               !s->wheel_pending || intent->zero_current != s->wheel_zero) {
        return false;
    }
    if (!note_time(s, now_us)) {
        return false;
    }
    if (bus == 0) {
        s->rs_pending[intent->axis] = false;
    } else {
        s->wheel_pending = false;
    }
    s->mail[bus][mailbox].active = true;
    s->mail[bus][mailbox].intent = *intent;
    s->mail[bus][mailbox].accepted_us = now_us;
    return true;
}

bool b_sched_completed(BTxScheduler *s, BSchedBus bus,
                       uint8_t mailbox, uint64_t now_us)
{
    const int slot = bus_slot(bus);
    if (!s || !s->initialized || slot < 0 || mailbox >= 3u || !s->mail[slot][mailbox].active ||
        !note_time(s, now_us)) {
        return false;
    }
    const BSchedIntent intent = s->mail[slot][mailbox].intent;
    s->mail[slot][mailbox].active = false;
    if (slot == 0) {
        if (intent.generation != s->rs_generation ||
            !s->drive_active || s->fault_requested) {
            ++s->rs_late_completions;
        } else {
            s->rs_tx_valid[intent.axis] = true;
            s->rs_tx_done_us[intent.axis] = now_us;
        }
    } else if (s->drive_active && !s->fault_requested &&
               !intent.zero_current && intent.generation == s->wheel_generation) {
        s->wheel_tx_valid = true;
        s->wheel_tx_done_us = now_us;
    }
    return true;
}

bool b_sched_feedback(BTxScheduler *s, BSchedBus bus,
                      uint8_t axis, uint64_t now_us)
{
    const int slot = bus_slot(bus);
    if (!s || !s->initialized || slot < 0 || (slot == 0 && axis >= 4u) ||
        (slot == 1 && axis >= 2u) || !note_time(s, now_us)) {
        return false;
    }
    if (slot == 0) {
        if (s->rs_rx_valid[axis] && now_us <= s->rs_rx_us[axis]) {
            return false;
        }
        s->rs_rx_valid[axis] = true;
        s->rs_rx_us[axis] = now_us;
    } else {
        if (s->wheel_rx_valid[axis] && now_us <= s->wheel_rx_us[axis]) {
            return false;
        }
        s->wheel_rx_valid[axis] = true;
        s->wheel_rx_us[axis] = now_us;
    }
    return true;
}

void b_sched_bus_off(BTxScheduler *s, uint64_t now_us)
{
    if (s && s->initialized && note_time(s, now_us)) {
        request_fault(s, B_SCHED_FAULT_BUS_OFF);
    }
}
