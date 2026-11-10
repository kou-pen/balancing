#ifndef BALANCING_TX_SCHEDULER_H
#define BALANCING_TX_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ホスト試験用の抽象CANバス。フレーム・HALハンドルは持たない。 */
typedef enum {
    B_SCHED_CAN1_RS06 = 1,
    B_SCHED_CAN2_C620 = 2
} BSchedBus;

/** @brief スケジューラが検出した異常。上位のSafetyがFAULT要求として扱う。 */
typedef enum {
    B_SCHED_FAULT_NONE = 0,
    B_SCHED_FAULT_CLOCK = 1u << 0,
    B_SCHED_FAULT_CONTROL_GAP = 1u << 1,
    B_SCHED_FAULT_RS_RX = 1u << 2,
    B_SCHED_FAULT_WHEEL_RX = 1u << 3,
    B_SCHED_FAULT_RS_TX = 1u << 4,
    B_SCHED_FAULT_C620_TX = 1u << 5,
    B_SCHED_FAULT_BUS_OFF = 1u << 6
} BSchedFault;

/**
 * @brief CAN送信完了の実機判定閾値は未確定。テスト値を明示的に注入する。
 * @spec{BOARD-F446-001}
 * @warning ここへ渡す値は実機の安全閾値を決めるものではない。0を拒否する。
 */
typedef struct {
    uint64_t rs_tx_timeout_us;
    uint64_t c620_tx_timeout_us;
} BSchedConfig;

/**
 * @brief データなしの送信要求。実トルク・CAN ID・HAL送信は含まない。
 * @spec{BOARD-F446-001}
 * @note 受付と完了は別API。RS06 axis=0..3、CAN2 axis=0xff。
 */
typedef struct {
    BSchedBus bus;
    uint8_t axis;
    uint32_t generation;
    bool zero_current; /**< CAN2は駆動不許可時にゼロを要求する。 */
} BSchedIntent;

typedef struct {
    bool active;
    BSchedIntent intent;
    uint64_t accepted_us;
} BSchedMail;

/** @brief 静的に配置する送信要求モデル。1バス3メールボックス。 */
typedef struct {
    BSchedConfig config;
    bool initialized; /**< 設定が有効な初期化完了までは送信要求を拒否する。 */
    bool tick_valid;
    bool event_valid;
    bool drive_active;
    bool fault_requested;
    uint32_t fault_flags;
    uint64_t last_tick_us;
    uint64_t last_event_us;
    uint64_t active_since_us;
    uint64_t next_rs_due_us;
    uint32_t rs_generation;
    uint32_t wheel_generation;
    bool rs_pending[4];
    bool wheel_pending;
    bool wheel_zero;
    BSchedMail mail[2][3];
    bool rs_rx_valid[4];
    uint64_t rs_rx_us[4];
    bool wheel_rx_valid[2];
    uint64_t wheel_rx_us[2];
    bool rs_tx_valid[4];
    uint64_t rs_tx_done_us[4];
    bool wheel_tx_valid;
    uint64_t wheel_tx_done_us;
    uint32_t rs_dropped_pending;
    uint32_t wheel_dropped_pending;
    uint32_t rs_late_completions;
} BTxScheduler;

/**
 * @brief HALに触れず送信スケジューラを初期化する。
 * @spec{BOARD-F446-001}
 * @return 有効な設定ならtrue。実機駆動は一切有効化しない。
 * @warning 現場のCANメールボックスをキャンセルするAPIではない。初期化は
 *          CAN送信開始前か、外側で完全に送信停止・回復した後に限る。
 *          configはscheduler内のconfig領域を指してはならない。
 */
bool b_sched_init(BTxScheduler *scheduler, const BSchedConfig *config);

/**
 * @brief 500 Hz制御ティック。CAN2毎回・CAN1を4 msごとに更新する。
 * @spec{BOARD-F446-001}
 * @param[in] drive_allowed 上位Safetyの駆動許可。FAULT時には無視する。
 * @param[in] now_us 共通単調時刻[us]。時刻逆行と6 ms以上の欠落を監視。
 * @warning 機構のトルク・電流値の計算や送信は行わない。
 */
void b_sched_tick(BTxScheduler *scheduler, uint64_t now_us, bool drive_allowed);

/**
 * @brief 未受付の最新送信要求を1件だけ参照する（HALを呼ばない）。
 * @spec{PROTO-RS06-001}
 * @note false/失敗時にはHAL受付の通知を行わないこと。要求をキューに追加しない。
 */
bool b_sched_next(const BTxScheduler *scheduler, BSchedBus bus, BSchedIntent *intent);

/**
 * @brief HALが送信を受理した場合のみ、メールボックスを占有する。
 * @spec{PROTO-RS06-001}
 * @param[in] mailbox 0..2。失敗／BUSYの場合は呼ばない。
 * @return 要求が現行世代でメールボックスが空ならtrue。
 * @pre b_sched_nextで取得した要求を使用する。古い世代や占有競合は再検査して拒否する。
 * @warning 将来HALへ接続する場合、HAL受付から本APIまでにIRQが割り込まない設計とし、
 *          falseでも実HALが受付済みなら無追跡フレームを残さずFAULT/中止処理へ移す。
 */
bool b_sched_accepted(BTxScheduler *scheduler, const BSchedIntent *intent,
                      uint8_t mailbox, uint64_t now_us);

/**
 * @brief CAN送信完了IRQを模擬する。HAL受付では完了扱いにしない。
 * @spec{PROTO-RS06-001}
 * @return 追跡中のメールボックスを完了した場合true。
 */
bool b_sched_completed(BTxScheduler *scheduler, BSchedBus bus,
                       uint8_t mailbox, uint64_t now_us);

/**
 * @brief 新規で妥当な受信フィードバックの時刻を通知する。
 * @spec{PROTO-RS06-001}
 * @param[in] axis CAN1は0..3、CAN2は0..1。再利用した旧測定には呼ばない。
 */
bool b_sched_feedback(BTxScheduler *scheduler, BSchedBus bus,
                      uint8_t axis, uint64_t now_us);

/**
 * @brief バスオフを報告し、制御へのFAULT要求をラッチする。
 * @spec{BOARD-F446-001}
 * @warning FAULTを自動解除しない。バスオフの物理回復処理は別責務。
 */
void b_sched_bus_off(BTxScheduler *scheduler, uint64_t now_us);

#ifdef __cplusplus
}
#endif
#endif /* BALANCING_TX_SCHEDULER_H */
