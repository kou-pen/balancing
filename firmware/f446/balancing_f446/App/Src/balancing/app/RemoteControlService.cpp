#include "balancing/app/RemoteControlService.hpp"
#include "balancing/communication/can/RemoteControlCodec.hpp"
#include <cmath>

namespace balancing {
namespace {
bool knownSource(OperatorSource source)
{
    return source == OperatorSource::UsbDualSense ||
           source == OperatorSource::CanRemote;
}

bool validConfig(const OperatorInputConfig& config)
{
    /*
     * 未校正の上限・デッドゾーンで意図せぬ指令を生成しないよう、
     * 人工fixtureを含めて設定を明示的に承認した場合だけ初期化できる。
     * 通信期限もニュートラル継続期間も0を許さない。
     */
    return config.verified &&
           config.input_timeout_us > 0u &&
           config.neutral_hold_us > 0u &&
           std::isfinite(config.max_speed_m_s) && config.max_speed_m_s > 0.0f &&
           std::isfinite(config.max_yaw_rate_rad_s) && config.max_yaw_rate_rad_s > 0.0f &&
           std::isfinite(config.neutral_speed_m_s) && config.neutral_speed_m_s >= 0.0f &&
           config.neutral_speed_m_s < config.max_speed_m_s &&
           std::isfinite(config.neutral_yaw_rad_s) && config.neutral_yaw_rad_s >= 0.0f &&
           config.neutral_yaw_rad_s < config.max_yaw_rate_rad_s &&
           std::isfinite(config.speed_slew_m_s2) && config.speed_slew_m_s2 > 0.0f &&
           std::isfinite(config.yaw_slew_rad_s2) && config.yaw_slew_rad_s2 > 0.0f;
}

/*
 * 前回の目標値から1周期に進める量を max_rate × dt に制限する。
 * スティックがニュートラルへ戻った際にも瞬間的な指令反転を避けるため、
 * 出力範囲そのものではなく「前回からの変化量」を飽和する。
 * NaN・Infと不正周期は0へフェイルクローズする。
 */
bool rateLimit(float previous, float request, float max_rate,
                  float dt_s, float *result)
{
    float max_step;
    float delta;

    if (result == nullptr) {
        return false;
    }
    *result = 0.0f;
    if (!std::isfinite(previous) || !std::isfinite(request) || !std::isfinite(max_rate) ||
        !std::isfinite(dt_s) || max_rate <= 0.0f || dt_s <= 0.0f) {
        return false;
    }
    max_step = max_rate * dt_s;
    if (!std::isfinite(max_step)) {
        return false;
    }

    delta = request - previous;
    if (!std::isfinite(delta)) {
        return false;
    }
    delta = std::fmax(-max_step, std::fmin(max_step, delta));
    *result = previous + delta;
    if (!std::isfinite(*result)) {
        *result = 0.0f;
        return false;
    }
    return true;
}

}

RemoteControlService::RemoteControlService(const OperatorInputConfig& config) noexcept
{
    // 未検証設定で入力・目標を有効化しない。状態の所有者はこのクラスだけ。
    if (!validConfig(config)) return;
    input_.config=config;
    input_.selected=OperatorSource::UsbDualSense;
    input_.initialized=true;
    requireNeutral();
}

void RemoteControlService::requireNeutral() noexcept
{
    // 中断前の中立入力は復帰の証拠にならない。
    input_.waiting_neutral=true;
    input_.neutral_start_valid=false;
    input_.neutral_sample_count=0;
}

bool RemoteControlService::sampleRecent(std::uint64_t nowUs) const noexcept
{
    // 期限ちょうどは有効。未来の時刻は鮮度を成立させない。
    return input_.last_sample.valid && nowUs>=input_.last_sample.stamp_us &&
           nowUs-input_.last_sample.stamp_us<=input_.config.input_timeout_us;
}

bool RemoteControlService::selectSource(OperatorSource source,RobotState state) noexcept
{
    // ARMINGも含めDISARMED以外の切替は禁止。通信断で暗黙切替しない。
    if (!input_.initialized || !knownSource(source) || state!=RobotState::Disarmed) return false;
    if (input_.selected==source) return true;
    input_.selected=source;
    input_.last_sample={};
    input_.target_speed_m_s=0;
    input_.target_yaw_rate_rad_s=0;
    input_.tick_valid=false;
    requireNeutral();
    return true;
}

bool RemoteControlService::acceptDecoded(OperatorSource source,const OperatorSample& sample,
    bool new_sample_proven,std::uint64_t now_us) noexcept
{
    const auto stamp_us=sample.stamp_us;
    const auto speed_m_s=sample.speed_m_s;
    const auto yaw_rate_rad_s=sample.yaw_rate_rad_s;
    /*
     * 新しいデコード済み入力だけが受信鮮度を更新できる。
     * 選択されていない操作元、重複/過去/未来タイムスタンプ、
     * NaN/Infや最大範囲超過は既存状態を変えず拒否する。
     */
    if (!input_.initialized || !knownSource(source) || !sample.valid ||
        source != input_.selected || !new_sample_proven ||
        stamp_us > now_us ||
        (input_.last_sample.valid && stamp_us <= input_.last_sample.stamp_us) ||
        !std::isfinite(speed_m_s) || !std::isfinite(yaw_rate_rad_s) ||
        std::fabs(speed_m_s) > input_.config.max_speed_m_s ||
        std::fabs(yaw_rate_rad_s) > input_.config.max_yaw_rate_rad_s) {
        return false;
    }

    /*
     * tick()が通信断をまだ観測していなくても、前回の新規入力から
     * 規定期限以上の空白がある場合、復帰前のニュートラル確認を要求する。
     * 単一の長時間据置きニュートラル報告だけでは復帰させない。
     */
    if (input_.last_sample.valid &&
        stamp_us - input_.last_sample.stamp_us > input_.config.input_timeout_us) {
        requireNeutral();
    }

    if (input_.waiting_neutral) {
        if (std::fabs(speed_m_s) > input_.config.neutral_speed_m_s || std::fabs(yaw_rate_rad_s) > input_.config.neutral_yaw_rad_s) {
            /* 操作されている間はニュートラル確認タイマを進めない。 */
            input_.neutral_start_valid = false;
            input_.neutral_sample_count = 0u;
        } else if (!input_.neutral_start_valid) {
            input_.neutral_start_valid = true;
            input_.neutral_start_us = stamp_us;
            input_.neutral_sample_count = 1u;
        } else {
            /*
             * 2つ以上の独立した新規ニュートラルサンプルと継続時間の両方を要求。
             * 途中に非ニュートラルが入ったら上記分岐で最初からやり直す。
             */
            input_.neutral_sample_count = 2u;
            if (stamp_us - input_.neutral_start_us >=
                input_.config.neutral_hold_us) {
                input_.waiting_neutral = false;
            }
        }
    }
    input_.last_sample = OperatorSample{
        true, stamp_us, speed_m_s, yaw_rate_rad_s
    };
    return true;
}


bool RemoteControlService::acceptCanMotion(const CanFrame& frame,std::uint64_t receivedAtUs,
                                           std::uint64_t nowUs) noexcept
{
    // 受信キューで期限を超えた入力を中立確認や鮮度更新に利用しない。
    if (!input_.initialized || receivedAtUs>nowUs ||
        nowUs-receivedAtUs>input_.config.input_timeout_us) return false;
    const auto motion=protocol::decodeMotionCommand(frame);
    return motion && acceptDecoded(OperatorSource::CanRemote,
        {true,receivedAtUs,motion->forwardVelocityMps,motion->yawRateRadS},true,nowUs);
}

std::optional<OperatorSource> RemoteControlService::selectedSource() const noexcept
{
    return input_.initialized ? std::optional<OperatorSource>{input_.selected} : std::nullopt;
}


void RemoteControlService::invalidate(OperatorSource source) noexcept
{
    if (!input_.initialized || input_.selected!=source) return;
    // 旧サンプルを破棄する一方、変化率制限の状態は保ち安全に減速する。
    input_.last_sample={};
    requireNeutral();
}
void RemoteControlService::invalidateCanInput() noexcept { invalidate(OperatorSource::CanRemote); }
void RemoteControlService::invalidateUsbInput() noexcept { invalidate(OperatorSource::UsbDualSense); }

std::optional<OperatorTarget> RemoteControlService::update(RobotState state,std::uint64_t now_us) noexcept
{
    if (!input_.initialized || (state!=RobotState::Disarmed &&
        state!=RobotState::Arming && state!=RobotState::Balancing &&
        state!=RobotState::Fault)) return std::nullopt;
    if (input_.tick_valid && now_us < input_.last_tick_us) {
        /* 時計逆行時は失敗・ゼロ出力にし、前回のタイムスタンプを
         * 書き換えて見かけ上の鮮度を回復することを防ぐ。 */
        return std::nullopt;
    }

    bool fresh = sampleRecent(now_us);
    if (!fresh && (!input_.waiting_neutral ||
                   input_.neutral_start_valid ||
                   input_.neutral_sample_count != 0u)) {
        /* 有効な前回指令が切れた後は、復帰しても自動再開を許可しない。 */
        requireNeutral();
    }

    const bool ready = fresh && !input_.waiting_neutral;
    const bool drive_target = ready && state == RobotState::Balancing;
    /*
     * 実機のデッドゾーン値は外部設定。確認済みの中立域内は目標を0へ落とし、
     * ニュートラル成立直後に微小な動作要求が残らないようにする。
     */
    const float requested_speed =
        drive_target &&
        std::fabs(input_.last_sample.speed_m_s) > input_.config.neutral_speed_m_s ?
        input_.last_sample.speed_m_s : 0.0f;
    const float requested_yaw =
        drive_target &&
        std::fabs(input_.last_sample.yaw_rate_rad_s) > input_.config.neutral_yaw_rad_s ?
        input_.last_sample.yaw_rate_rad_s : 0.0f;

    /*
     * DISARMED/FAULTはモータ出力の許可ではないので、操作目標を即時ゼロへ。
     * BALANCING中のリンク喪失では、姿勢保持を妨げる急な目標変化を避け、
     * 速度とYawの両方をそれぞれ設定された変化率でゼロへ戻す。
     */
    if (state != RobotState::Balancing) {
        input_.target_speed_m_s = 0.0f;
        input_.target_yaw_rate_rad_s = 0.0f;
    } else if (input_.tick_valid && now_us > input_.last_tick_us) {
        const float dt_s = static_cast<float>(now_us - input_.last_tick_us) * 0.000001f;
        float speed;
        float yaw;
        if (!rateLimit(input_.target_speed_m_s, requested_speed,
                          input_.config.speed_slew_m_s2, dt_s, &speed) ||
            !rateLimit(input_.target_yaw_rate_rad_s, requested_yaw,
                          input_.config.yaw_slew_rad_s2, dt_s, &yaw)) {
            /* 不正な計算結果を実指令として残さない。
             * FALSEはこのティック全体の処理失敗を上位に通知する。 */
            input_.target_speed_m_s = 0.0f;
            input_.target_yaw_rate_rad_s = 0.0f;
            return std::nullopt;
        }
        input_.target_speed_m_s = speed;
        input_.target_yaw_rate_rad_s = yaw;
    }

    input_.tick_valid = true;
    input_.last_tick_us = now_us;

    /* ARM/CLEAR_FAULTの出力項目を意図的に持たせない。
     * USB/CANの操作を復帰させても上位の状態機械は自動ARMできない。 */
    return OperatorTarget{input_.selected,fresh,ready,input_.waiting_neutral,
                          input_.target_speed_m_s,input_.target_yaw_rate_rad_s};
}
}
