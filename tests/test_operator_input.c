#include "balancing/app/balancing_operator_input.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * ここで使う速度・Yaw・デッドゾーン・期限は数学fixture専用。
 * 実PS5のスティック換算、HIDボタン番号、CAN_REMOTE IDを仮決定しない。
 */
static BOperatorInputConfig fixture_config(void)
{
    const BOperatorInputConfig cfg = {
        .input_timeout_us = 10000u,
        .neutral_hold_us = 4000u,
        .max_speed_m_s = 3.0f,
        .max_yaw_rate_rad_s = 2.0f,
        .neutral_speed_m_s = 0.05f,
        .neutral_yaw_rad_s = 0.03f,
        .speed_slew_m_s2 = 2.0f,
        .yaw_slew_rad_s2 = 1.0f,
        .verified = true
    };
    return cfg;
}

static void close_to(float got, float expected, float tolerance)
{
    if (!(isfinite(got) && isfinite(expected) &&
          fabsf(got - expected) <= tolerance)) {
        fprintf(stderr, "got=%g expected=%g tolerance=%g\n",
                (double)got, (double)expected, (double)tolerance);
        assert(false);
    }
}

/* ニュートラルの証拠は「時間が経った」だけでは不足する。
 * 実際に2件以上の別時刻の新規サンプルが必要。 */
static void test_startup_neutral_gate_and_no_auto_arm(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    BOperatorTarget out = {0};
    assert(b_operator_init(&input, &cfg));
    assert(input.selected == B_OPERATOR_USB_DUALSENSE);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             1.0f, 0.3f, true, 0u, 0u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 0u, &out));
    assert(out.waiting_neutral && !out.operator_ready);
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);

    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 1000u, 1000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 5000u, &out));
    assert(out.waiting_neutral); /* 新規ニュートラルは1件だけ */
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 5000u, 5000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 5000u, &out));
    assert(!out.waiting_neutral && out.operator_ready);
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);
    /* この出力構造体にARM要求フィールドは存在しない。 */
    assert(out.source == B_OPERATOR_USB_DUALSENSE);

    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             1.0f, -0.5f, true, 6000u, 6000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 7000u, &out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    close_to(out.yaw_rate_rad_s, -0.002f, 1e-6f);
}

/* DISARMED以外の操作元変更と未選択ソース入力は受け付けない。
 * 通信断時にUSB→CANを暗黙切替しないのが最重要の不変条件。 */
static void test_source_switch_requires_disarmed(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    BOperatorTarget out = {0};
    assert(b_operator_init(&input, &cfg));
    assert(!b_operator_select(&input, B_OPERATOR_CAN_REMOTE, B_MODE_BALANCING));
    assert(!b_operator_select(&input, B_OPERATOR_CAN_REMOTE, B_MODE_FAULT));
    assert(!b_operator_select(&input, (BOperatorSource)99, B_MODE_DISARMED));
    assert(!b_operator_accept(&input, B_OPERATOR_CAN_REMOTE,
                              0.0f, 0.0f, true, 1000u, 1000u));

    assert(b_operator_select(&input, B_OPERATOR_CAN_REMOTE, B_MODE_DISARMED));
    assert(input.selected == B_OPERATOR_CAN_REMOTE);
    assert(input.waiting_neutral && !input.last_sample.valid);
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              0.0f, 0.0f, true, 1000u, 1000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 1000u, &out));
    assert(out.source == B_OPERATOR_CAN_REMOTE);
    assert(!out.source_fresh && out.waiting_neutral);

    assert(b_operator_accept(&input, B_OPERATOR_CAN_REMOTE,
                             0.0f, 0.0f, true, 2000u, 2000u));
    assert(b_operator_accept(&input, B_OPERATOR_CAN_REMOTE,
                             0.0f, 0.0f, true, 6000u, 6000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 6000u, &out));
    assert(out.operator_ready);
    assert(b_operator_select(&input, B_OPERATOR_USB_DUALSENSE, B_MODE_DISARMED));
    assert(!input.last_sample.valid && input.waiting_neutral);
    assert(b_operator_tick(&input, B_MODE_BALANCING, 7000u, &out));
    assert(!out.operator_ready && !out.source_fresh);
}

/* 正常リンクで指令を徐々に追いかけ、通信断では両目標をゼロに戻す。
 * 復旧時にスティックが傾いたままなら操作を自動再開しない。 */
static void test_loss_and_recovery_neutral_with_slew(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    BOperatorTarget out = {0};
    assert(b_operator_init(&input, &cfg));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 0u, 0u));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 4000u, 4000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 4000u, &out));
    assert(out.operator_ready);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             1.0f, 0.5f, true, 6000u, 6000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 6000u, &out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.002f, 1e-6f);
    assert(b_operator_tick(&input, B_MODE_BALANCING, 8000u, &out));
    close_to(out.speed_m_s, 0.008f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.004f, 1e-6f);

    /* 新規データのない状態で500 Hzが動いても受信鮮度は更新しない。
     * 最終新規入力6msからちょうど10msの16msまでは有効。 */
    assert(b_operator_tick(&input, B_MODE_BALANCING, 16000u, &out));
    assert(out.operator_ready && out.source_fresh);
    const float speed_before_loss = out.speed_m_s;
    const float yaw_before_loss = out.yaw_rate_rad_s;
    assert(speed_before_loss > 0.0f && yaw_before_loss > 0.0f);

    assert(b_operator_tick(&input, B_MODE_BALANCING, 18000u, &out));
    assert(!out.source_fresh && out.waiting_neutral && !out.operator_ready);
    assert(out.speed_m_s < speed_before_loss && out.speed_m_s >= 0.0f);
    assert(out.yaw_rate_rad_s < yaw_before_loss && out.yaw_rate_rad_s >= 0.0f);
    assert(input.selected == B_OPERATOR_USB_DUALSENSE);

    /* 非ニュートラルで戻ってもバランシング操作は復旧しない。 */
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             1.0f, 0.5f, true, 20000u, 20000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 20000u, &out));
    assert(out.waiting_neutral && !out.operator_ready);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 22000u, 22000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 24000u, &out));
    assert(!out.operator_ready);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 26000u, 26000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 26000u, &out));
    assert(out.operator_ready && !out.waiting_neutral);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             -1.0f, -0.5f, true, 28000u, 28000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 28000u, &out));
    assert(out.speed_m_s <= 0.0f);
    assert(out.yaw_rate_rad_s <= 0.0f);
}

/* 重複時刻・未来時刻・NaN・未選択ソースを拒否しても前回の値と
 * その時刻が残り、通信断を隠さないことを検査する。 */
static void test_invalid_samples_never_refresh(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    assert(b_operator_init(&input, &cfg));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 1000u, 1000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              0.0f, 0.0f, false, 2000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              0.0f, 0.0f, true, 1000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              0.0f, 0.0f, true, 3000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              NAN, 0.0f, true, 2000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              0.0f, INFINITY, true, 2000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              3.1f, 0.0f, true, 2000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                              0.0f, 2.1f, true, 2000u, 2000u));
    assert(!b_operator_accept(&input, B_OPERATOR_CAN_REMOTE,
                              0.0f, 0.0f, true, 2000u, 2000u));
    assert(input.last_sample.valid && input.last_sample.stamp_us == 1000u);
}

/* 通信断をティックで観測しなかった場合も、受信時刻のギャップで
 * 復帰前のニュートラル確認状態を初期化することを検証する。 */
static void test_missing_ticks_cannot_confirm_old_neutral(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    assert(b_operator_init(&input, &cfg));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 0u, 0u));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 100000u, 100000u));
    assert(input.waiting_neutral);
    assert(input.neutral_sample_count == 1u);
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 104000u, 104000u));
    assert(!input.waiting_neutral);
}

/* 正常な操作入力があっても、実状態機械がDISARMED/FAULTなら目標は即0。
 * 同一ティックの再呼出しは二重加速させず、時刻逆行は拒否する。 */
static void test_mode_zero_priority_and_clock(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    BOperatorTarget out = {0};
    assert(b_operator_init(&input, &cfg));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 0u, 0u));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.0f, 0.0f, true, 4000u, 4000u));
    assert(b_operator_tick(&input, B_MODE_DISARMED, 4000u, &out));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.8f, -0.4f, true, 6000u, 6000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 6000u, &out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    assert(b_operator_tick(&input, B_MODE_BALANCING, 6000u, &out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    assert(!b_operator_tick(&input, B_MODE_BALANCING, 5000u, &out));
    close_to(out.speed_m_s, 0.0f, 0.0f); /* 失敗時の出力はゼロ */
    assert(b_operator_tick(&input, B_MODE_DISARMED, 8000u, &out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);
    assert(b_operator_tick(&input, B_MODE_FAULT, 10000u, &out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
    assert(!b_operator_tick(&input, (BMode)99, 12000u, &out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
}

/* ニュートラル境界内の微小な目標は0にし、実HIDが未確定の
 * デッドゾーンを暗黙のハードコード値で判定しない。 */
static void test_deadband_and_config_rejection(void)
{
    BOperatorInput input;
    BOperatorInputConfig cfg = fixture_config();
    BOperatorTarget out = {0};
    cfg.verified = false;
    assert(!b_operator_init(&input, &cfg));
    assert(!input.initialized);
    cfg = fixture_config();
    cfg.neutral_speed_m_s = cfg.max_speed_m_s;
    assert(!b_operator_init(&input, &cfg));
    cfg = fixture_config();
    cfg.speed_slew_m_s2 = NAN;
    assert(!b_operator_init(&input, &cfg));
    cfg = fixture_config();
    assert(b_operator_init(&input, &cfg));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.03f, -0.02f, true, 1000u, 1000u));
    assert(b_operator_accept(&input, B_OPERATOR_USB_DUALSENSE,
                             0.02f, 0.0f, true, 5000u, 5000u));
    assert(b_operator_tick(&input, B_MODE_BALANCING, 5000u, &out));
    assert(out.operator_ready);
    assert(b_operator_tick(&input, B_MODE_BALANCING, 7000u, &out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);
}

int main(void)
{
    test_startup_neutral_gate_and_no_auto_arm();
    test_source_switch_requires_disarmed();
    test_loss_and_recovery_neutral_with_slew();
    test_invalid_samples_never_refresh();
    test_missing_ticks_cannot_confirm_old_neutral();
    test_mode_zero_priority_and_clock();
    test_deadband_and_config_rejection();
    puts("PASS: operator mock source isolation, loss/recovery, neutral gating, slew and no ARM");
    return 0;
}
