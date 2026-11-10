#include "balancing/communication/balancing_tx_scheduler.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* 全て同一の単調増加マイクロ秒時計。実機のタイミング保証ではない。 */
#define RS_PERIOD_US 4000u
#define CONTROL_GAP_FAULT_US 6000u
#define RS_RX_TIMEOUT_US 30000u
#define WHEEL_RX_TIMEOUT_US 20000u

/* 論理バスを固定2スロットへ写像する。CAN1/2は別のメールボックス群。
 * 不明なバスをCAN2として黙認しない。 */
static int bus_slot(BSchedBus bus)
{
    if (bus == B_SCHED_CAN1_RS06) {
        return 0;
    }
    return bus == B_SCHED_CAN2_C620 ? 1 : -1;
}

/*
 * FAULTは累積ビットとしてラッチし、以後の駆動許可を落とす。
 * ソフトウェア上の未受付要求を破棄し、CAN2の次要求は0 Aへ置換する。
 * HAL受付済みフレームや物理モータを停止できたことは保証しない。
 */
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

/*
 * すべてのイベントを同じ単調増加us時計で記録する。
 * 時刻が逆行した場合は、駆動中のみFAULTを要求して以後の更新を拒否。
 * 先行イベントの時刻で期限を再計算するとTX欠落を見逃すため重要。
 */
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

/* 期限は差分で計算してuint64_t加算オーバーフローを避ける。
 * しきい値ちょうどを「期限到達」と判定する。 */
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

/*
 * 受信フィードバック(RX)と実送信完了(TX)は独立に監視する。
 * HALの受付成功はTX完了でも受信成功でもない。
 * 有効化直後に完了履歴がない軸はactive_sinceを起点に期限を測る。
 * 異常は最初に検出した系統をFAULTとして要求し、上位Safetyが扱う。
 */
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

/*
 * 500 Hz周期を仮想時刻で表現し、CAN2を毎ティック、CAN1を4ms位相で更新。
 * 遅延したティックの過去世代を後追い送信しない。これにより
 * 古いトルク要求が現在の制御結果より後から出ることを防ぐ。
 * 本関数はデータなしの送信要求モデルであり、実フレームを生成しない。
 */
void b_sched_tick(BTxScheduler *s, uint64_t now_us, bool drive_allowed)
{
    if (!s || !s->initialized || !note_time(s, now_us)) {
        return;
    }
    /* 重複タイマ通知は冪等。ただし駆動許可の立下がりは同時刻でも優先し、
     * 旧トルク要求を即座に無効化する。立上がりは次の正式ティックまで待つ。 */
    if (s->tick_valid && now_us == s->last_tick_us &&
        !(s->drive_active && !drive_allowed)) {
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

    /* 上位Safetyからの許可が落ちると、受付待ちのRS06要求を破棄する。
     * 実CANを使ってType4を送る制御は別コンポーネントの責務。 */
    const bool activate = drive_allowed && !s->fault_requested;
    if (!activate) {
        if (s->drive_active) {
            /* HAL受付済みの旧ARM世代が後で完了しても鮮度を回復させない。 */
            ++s->rs_generation;
        }
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

    /* RS06の位相は4ms境界で進める。遅延した周期をバースト送信すると
     * 3メールボックスに旧コマンドが溜まるので、最新世代のみ保持する。 */
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

/* next()とaccepted()に同じ占有条件を適用する。
 * accepted()は呼出し側が古いIntentを保持している場合でも再検証する。 */
/*
 * 前世代のフレームが実HALメールボックスに残る間は新世代を重ねない。
 * 受付済み要求をソフトから取り消せないため、遅いCANが旧トルクを
 * 後送する危険をモデル側で検知・抑制する。
 */
static bool rs_previous_generation_inflight(const BTxScheduler *s)
{
    for (unsigned m = 0; m < 3u; ++m) {
        if (s->mail[0][m].active &&
            s->mail[0][m].intent.generation != s->rs_generation) {
            return true;
        }
    }
    return false;
}

/* C620の共通0x200は1フレームに4軸分が入る。
 * 旧0x200のHAL受付が残る間は別世代の共通指令を重ねない。 */
static bool wheel_inflight(const BTxScheduler *s)
{
    for (unsigned m = 0; m < 3u; ++m) {
        if (s->mail[1][m].active) {
            return true;
        }
    }
    return false;
}

/*
 * 未受付の最新世代Intentを提示するだけでキューには積まない。
 * 同じIntentを複数回参照し得るため、実HALが送信を受けた場合のみ
 * accepted()で占有を確定する。
 */
bool b_sched_next(const BTxScheduler *s, BSchedBus bus, BSchedIntent *intent)
{
    if (!s || !s->initialized || !intent) {
        return false;
    }
    if (bus == B_SCHED_CAN1_RS06 && s->drive_active && !s->fault_requested) {
        /* 前世代が実メールボックスに残る間は新しい世代を重ねない。 */
        if (rs_previous_generation_inflight(s)) {
            return false;
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
        if (wheel_inflight(s)) {
            return false;
        }
        *intent = (BSchedIntent){B_SCHED_CAN2_C620, 0xffu,
                                 s->wheel_generation, s->wheel_zero};
        return true;
    }
    return false;
}

/*
 * HALがメールボックスへの受付に成功した時刻を記録する。
 * next()の呼出し後に別の制御ティックやFAULTが発生した可能性があるので、
 * 世代・軸・駆動許可・占有状態を改めて検証する。
 * 実HAL受付と本関数の間の割込み競合の扱いは統合層の責務。
 */
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
            !s->rs_pending[intent->axis] || intent->zero_current ||
            rs_previous_generation_inflight(s)) {
            return false;
        }
    } else if (intent->axis != 0xffu ||
               intent->generation != s->wheel_generation ||
               !s->wheel_pending || intent->zero_current != s->wheel_zero ||
               wheel_inflight(s)) {
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
    /* 後の送信完了通知で、そのメールボックスの元の世代を特定する。
     * 受付済みでも実バスへ出たとは限らず、完了時刻は別途記録する。 */
    s->mail[bus][mailbox].active = true;
    s->mail[bus][mailbox].intent = *intent;
    s->mail[bus][mailbox].accepted_us = now_us;
    return true;
}

/*
 * HALの送信完了と受付を区別し、古い世代の完了では現在のTX鮮度を回復しない。
 * 遅延完了でもメールボックス占有は解放して次要求に進める。
 * 結果は物理モータの停止やCANの信頼性を保証しない。
 */
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

/*
 * 通信デコーダが新規妥当と判定したフィードバックの受信時刻だけ受け取る。
 * TX完了とは別の時刻を保持し、同一時刻の再投入では鮮度を延長しない。
 */
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

/* CAN Bus-offを上位から通知された場合、FAULT要求をラッチする。
 * 実CANの復旧手順やFAULT解除は本送信モデルの責務外である。 */
void b_sched_bus_off(BTxScheduler *s, uint64_t now_us)
{
    if (s && s->initialized && note_time(s, now_us)) {
        request_fault(s, B_SCHED_FAULT_BUS_OFF);
    }
}
