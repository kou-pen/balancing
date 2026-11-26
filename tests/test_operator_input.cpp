#include "balancing/app/RemoteControlService.hpp"
using namespace balancing;

#include <assert.h>
#include <cmath>
#include <limits>
#include <stdio.h>
#include <string.h>

/*
 * ここで使う速度・Yaw・デッドゾーン・期限は数学fixture専用。
 * 実PS5のスティック換算、HIDボタン番号、CAN_REMOTE IDを仮決定しない。
 */
static OperatorInputConfig fixture_config(void)
{
    const OperatorInputConfig cfg{10000,4000,3.0f,2.0f,0.05f,0.03f,2.0f,1.0f,true};
    return cfg;
}

static bool tick(RemoteControlService& service,RobotState state,std::uint64_t now,OperatorTarget& out)
{
    const auto target=service.update(state,now);out=target.value_or(OperatorTarget{});return target.has_value();
}

static void close_to(float got, float expected, float tolerance)
{
    if (!(std::isfinite(got) && std::isfinite(expected) &&
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
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    OperatorTarget out{};
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(input.selectedSource() == OperatorSource::UsbDualSense);
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,0u,1.0f,0.3f},true,0u));
    assert(tick(input, RobotState::Balancing, 0u, out));
    assert(out.waiting_neutral && !out.operator_ready);
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);

    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,1000u,0.0f,0.0f},true,1000u));
    assert(tick(input, RobotState::Balancing, 5000u, out));
    assert(out.waiting_neutral); /* 新規ニュートラルは1件だけ */
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,5000u,0.0f,0.0f},true,5000u));
    assert(tick(input, RobotState::Balancing, 5000u, out));
    assert(!out.waiting_neutral && out.operator_ready);
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);
    /* この出力構造体にARM要求フィールドは存在しない。 */
    assert(out.source == OperatorSource::UsbDualSense);

    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,6000u,1.0f,-0.5f},true,6000u));
    assert(tick(input, RobotState::Balancing, 7000u, out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    close_to(out.yaw_rate_rad_s, -0.002f, 1e-6f);
}

/* DISARMED以外の操作元変更と未選択ソース入力は受け付けない。
 * 通信断時にUSB→CANを暗黙切替しないのが最重要の不変条件。 */
static void test_source_switch_requires_disarmed(void)
{
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    OperatorTarget out{};
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(!input.selectSource(OperatorSource::CanRemote, RobotState::Balancing));
    assert(!input.selectSource(OperatorSource::CanRemote, RobotState::Fault));
    assert(!input.selectSource(static_cast<OperatorSource>(99), RobotState::Disarmed));
    assert(!input.acceptDecoded(OperatorSource::CanRemote,{true,1000u,0.0f,0.0f},true,1000u));

    assert(input.selectSource(OperatorSource::CanRemote, RobotState::Disarmed));
    assert(input.selectedSource() == OperatorSource::CanRemote);
    assert(input.update(RobotState::Disarmed,1000)->waiting_neutral);
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,1000u,0.0f,0.0f},true,1000u));
    assert(tick(input, RobotState::Balancing, 1000u, out));
    assert(out.source == OperatorSource::CanRemote);
    assert(!out.source_fresh && out.waiting_neutral);

    assert(input.acceptDecoded(OperatorSource::CanRemote,{true,2000u,0.0f,0.0f},true,2000u));
    assert(input.acceptDecoded(OperatorSource::CanRemote,{true,6000u,0.0f,0.0f},true,6000u));
    assert(tick(input, RobotState::Balancing, 6000u, out));
    assert(out.operator_ready);
    assert(input.selectSource(OperatorSource::UsbDualSense, RobotState::Disarmed));
    assert(!input.update(RobotState::Disarmed,7000)->source_fresh);
    assert(tick(input, RobotState::Balancing, 7000u, out));
    assert(!out.operator_ready && !out.source_fresh);
}

/* 正常リンクで指令を徐々に追いかけ、通信断では両目標をゼロに戻す。
 * 復旧時にスティックが傾いたままなら操作を自動再開しない。 */
static void test_loss_and_recovery_neutral_with_slew(void)
{
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    OperatorTarget out{};
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,0u,0.0f,0.0f},true,0u));
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,4000u,0.0f,0.0f},true,4000u));
    assert(tick(input, RobotState::Balancing, 4000u, out));
    assert(out.operator_ready);
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,6000u,1.0f,0.5f},true,6000u));
    assert(tick(input, RobotState::Balancing, 6000u, out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.002f, 1e-6f);
    assert(tick(input, RobotState::Balancing, 8000u, out));
    close_to(out.speed_m_s, 0.008f, 1e-6f);
    close_to(out.yaw_rate_rad_s, 0.004f, 1e-6f);

    /* 新規データのない状態で500 Hzが動いても受信鮮度は更新しない。
     * 最終新規入力6msからちょうど10msの16msまでは有効。 */
    assert(tick(input, RobotState::Balancing, 16000u, out));
    assert(out.operator_ready && out.source_fresh);
    const float speed_before_loss = out.speed_m_s;
    const float yaw_before_loss = out.yaw_rate_rad_s;
    assert(speed_before_loss > 0.0f && yaw_before_loss > 0.0f);

    assert(tick(input, RobotState::Balancing, 18000u, out));
    assert(!out.source_fresh && out.waiting_neutral && !out.operator_ready);
    assert(out.speed_m_s < speed_before_loss && out.speed_m_s >= 0.0f);
    assert(out.yaw_rate_rad_s < yaw_before_loss && out.yaw_rate_rad_s >= 0.0f);
    assert(input.selectedSource() == OperatorSource::UsbDualSense);

    /* 非ニュートラルで戻ってもバランシング操作は復旧しない。 */
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,20000u,1.0f,0.5f},true,20000u));
    assert(tick(input, RobotState::Balancing, 20000u, out));
    assert(out.waiting_neutral && !out.operator_ready);
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,22000u,0.0f,0.0f},true,22000u));
    assert(tick(input, RobotState::Balancing, 24000u, out));
    assert(!out.operator_ready);
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,26000u,0.0f,0.0f},true,26000u));
    assert(tick(input, RobotState::Balancing, 26000u, out));
    assert(out.operator_ready && !out.waiting_neutral);
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,28000u,-1.0f,-0.5f},true,28000u));
    assert(tick(input, RobotState::Balancing, 28000u, out));
    assert(out.speed_m_s <= 0.0f);
    assert(out.yaw_rate_rad_s <= 0.0f);
}

/* 重複時刻・未来時刻・NaN・未選択ソースを拒否しても前回の値と
 * その時刻が残り、通信断を隠さないことを検査する。 */
static void test_invalid_samples_never_refresh(void)
{
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,1000u,0.0f,0.0f},true,1000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,2000u,0.0f,0.0f},false,2000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,1000u,0.0f,0.0f},true,2000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,3000u,0.0f,0.0f},true,2000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,2000u,NAN,0.0f},true,2000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,2000u,0.0f,INFINITY},true,2000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,2000u,3.1f,0.0f},true,2000u));
    assert(!input.acceptDecoded(OperatorSource::UsbDualSense,{true,2000u,0.0f,2.1f},true,2000u));
    assert(!input.acceptDecoded(OperatorSource::CanRemote,{true,2000u,0.0f,0.0f},true,2000u));
    assert(input.update(RobotState::Disarmed,11000)->source_fresh);
    assert(!input.update(RobotState::Disarmed,11001)->source_fresh);
}

/* 通信断をティックで観測しなかった場合も、受信時刻のギャップで
 * 復帰前のニュートラル確認状態を初期化することを検証する。 */
static void test_missing_ticks_cannot_confirm_old_neutral(void)
{
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,0u,0.0f,0.0f},true,0u));
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,100000u,0.0f,0.0f},true,100000u));
    assert(input.update(RobotState::Disarmed,100000)->waiting_neutral);
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,104000u,0.0f,0.0f},true,104000u));
    assert(input.update(RobotState::Disarmed,104000)->operator_ready);
}

/* 正常な操作入力があっても、実状態機械がDISARMED/FAULTなら目標は即0。
 * 同一ティックの再呼出しは二重加速させず、時刻逆行は拒否する。 */
static void test_mode_zero_priority_and_clock(void)
{
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    OperatorTarget out{};
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,0u,0.0f,0.0f},true,0u));
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,4000u,0.0f,0.0f},true,4000u));
    assert(tick(input, RobotState::Disarmed, 4000u, out));
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,6000u,0.8f,-0.4f},true,6000u));
    assert(tick(input, RobotState::Balancing, 6000u, out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    assert(tick(input, RobotState::Balancing, 6000u, out));
    close_to(out.speed_m_s, 0.004f, 1e-6f);
    assert(!tick(input, RobotState::Balancing, 5000u, out));
    close_to(out.speed_m_s, 0.0f, 0.0f); /* 失敗時の出力はゼロ */
    assert(tick(input, RobotState::Disarmed, 8000u, out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);
    assert(tick(input, RobotState::Fault, 10000u, out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
    assert(!tick(input, static_cast<RobotState>(99), 12000u, out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
}

/* ニュートラル境界内の微小な目標は0にし、実HIDが未確定の
 * デッドゾーンを暗黙のハードコード値で判定しない。 */
static void test_deadband_and_config_rejection(void)
{
    RemoteControlService input;
    OperatorInputConfig cfg = fixture_config();
    OperatorTarget out{};
    cfg.verified = false;
    input=RemoteControlService{cfg};
    assert(!input.selectedSource());
    assert(!input.update(RobotState::Disarmed,0));
    cfg = fixture_config();
    cfg.neutral_speed_m_s = cfg.max_speed_m_s;
    input=RemoteControlService{cfg};
    assert(!input.selectedSource());
    cfg = fixture_config();
    cfg.speed_slew_m_s2 = NAN;
    input=RemoteControlService{cfg};
    assert(!input.selectedSource());
    cfg = fixture_config();
    input=RemoteControlService{cfg};
    assert(input.selectedSource());
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,1000u,0.03f,-0.02f},true,1000u));
    assert(input.acceptDecoded(OperatorSource::UsbDualSense,{true,5000u,0.02f,0.0f},true,5000u));
    assert(tick(input, RobotState::Balancing, 5000u, out));
    assert(out.operator_ready);
    assert(tick(input, RobotState::Balancing, 7000u, out));
    close_to(out.speed_m_s, 0.0f, 0.0f);
    close_to(out.yaw_rate_rad_s, 0.0f, 0.0f);
}

int main()
{
    test_startup_neutral_gate_and_no_auto_arm();
    test_source_switch_requires_disarmed();
    test_loss_and_recovery_neutral_with_slew();
    test_invalid_samples_never_refresh();
    test_missing_ticks_cannot_confirm_old_neutral();
    test_mode_zero_priority_and_clock();
    test_deadband_and_config_rejection();
    // 巨大な変化率と周期で積が非有限になった場合は目標を発行しない。
    auto cfg=fixture_config();cfg.speed_slew_m_s2=std::numeric_limits<float>::max();
    RemoteControlService overflow{cfg};
    assert(overflow.update(RobotState::Disarmed,0));
    assert(!overflow.update(RobotState::Balancing,2000000));
    const auto zero=overflow.update(RobotState::Arming,2000000);
    assert(zero && zero->speed_m_s==0 && zero->yaw_rate_rad_s==0);
    puts("PASS: operator mock source isolation, loss/recovery, neutral gating, slew and no ARM");
    return 0;
}
