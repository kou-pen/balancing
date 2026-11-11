#include "balancing/app/balancing_operator_input.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * このモジュールはUSB HIDとCANの「デコード済み」操作量を扱う。
 * USBレポート番号・CAN ID・送信元認証の解釈は一切行わず、
 * 実機で未確定な数値は全てBOperatorInputConfigで明示的に受け取る。
 * ARMやモータ駆動には接続せず、状態機械のモードを入力として受け取るだけ。
 */
static bool known_source(BOperatorSource source)
{
    return source == B_OPERATOR_USB_DUALSENSE ||
           source == B_OPERATOR_CAN_REMOTE;
}

static bool known_mode(BMode mode)
{
    return mode == B_MODE_DISARMED ||
           mode == B_MODE_BALANCING ||
           mode == B_MODE_FAULT;
}

static bool valid_config(const BOperatorInputConfig *config)
{
    /*
     * 未校正の上限・デッドゾーンで意図せぬ指令を生成しないよう、
     * 人工fixtureを含めて設定を明示的に承認した場合だけ初期化できる。
     * 通信期限もニュートラル継続期間も0を許さない。
     */
    return config != NULL && config->verified &&
           config->input_timeout_us > 0u &&
           config->neutral_hold_us > 0u &&
           isfinite(config->max_speed_m_s) && config->max_speed_m_s > 0.0f &&
           isfinite(config->max_yaw_rate_rad_s) && config->max_yaw_rate_rad_s > 0.0f &&
           isfinite(config->neutral_speed_m_s) && config->neutral_speed_m_s >= 0.0f &&
           config->neutral_speed_m_s < config->max_speed_m_s &&
           isfinite(config->neutral_yaw_rad_s) && config->neutral_yaw_rad_s >= 0.0f &&
           config->neutral_yaw_rad_s < config->max_yaw_rate_rad_s &&
           isfinite(config->speed_slew_m_s2) && config->speed_slew_m_s2 > 0.0f &&
           isfinite(config->yaw_slew_rad_s2) && config->yaw_slew_rad_s2 > 0.0f;
}

static bool neutral_sample(const BOperatorInput *input, float speed, float yaw)
{
    /* 「ニュートラル」の判定値は実HIDの生スティック値ではなく、
     * 外部で換算したSI単位の指令値に対する二軸同時条件。 */
    return fabsf(speed) <= input->config.neutral_speed_m_s &&
           fabsf(yaw) <= input->config.neutral_yaw_rad_s;
}

static void require_neutral(BOperatorInput *input)
{
    /*
     * 受信が一度欠けたり操作元を切替えたら、旧入力の継続を認めない。
     * 次の復帰には複数の独立した新規ニュートラル入力を再度要求する。
     */
    input->waiting_neutral = true;
    input->neutral_start_valid = false;
    input->neutral_sample_count = 0u;
}

static bool sample_recent(const BOperatorInput *input, uint64_t now_us)
{
    /* 同じ単調時計で測定した時刻だけを使う。未来時刻は無効。
     * 期限ちょうどは有効とし、1us超過したときに通信断とみなす。 */
    return input->last_sample.valid &&
           now_us >= input->last_sample.stamp_us &&
           now_us - input->last_sample.stamp_us <= input->config.input_timeout_us;
}

bool b_operator_init(BOperatorInput *input, const BOperatorInputConfig *config)
{
    if (input == NULL) {
        return false;
    }
    /* 途中で不正な設定が渡されても、以前の有効状態が残らないようゼロ化する。 */
    memset(input, 0, sizeof(*input));
    if (!valid_config(config)) {
        return false;
    }
    input->config = *config;
    input->selected = B_OPERATOR_USB_DUALSENSE;
    input->initialized = true;
    require_neutral(input);
    return true;
}

bool b_operator_select(BOperatorInput *input, BOperatorSource source, BMode mode)
{
    /*
     * 物理リンクの断続だけでは操作元を変更しない。
     * 明示的な操作元切替はDISARMED中のみ受理し、切替後の旧サンプル、
     * 目標値、ニュートラル確認状態を持ち越さない。
     */
    if (input == NULL || !input->initialized || !known_source(source) ||
        mode != B_MODE_DISARMED) {
        return false;
    }
    if (input->selected == source) {
        return true;
    }
    input->selected = source;
    input->last_sample = (BOperatorSample){0};
    input->target_speed_m_s = 0.0f;
    input->target_yaw_rate_rad_s = 0.0f;
    input->tick_valid = false;
    require_neutral(input);
    return true;
}

bool b_operator_accept(BOperatorInput *input, BOperatorSource source,
                       float speed_m_s, float yaw_rate_rad_s,
                       bool new_sample_proven, uint64_t stamp_us, uint64_t now_us)
{
    /*
     * 新しいデコード済み入力だけが受信鮮度を更新できる。
     * 選択されていない操作元、重複/過去/未来タイムスタンプ、
     * NaN/Infや最大範囲超過は既存状態を変えず拒否する。
     */
    if (input == NULL || !input->initialized || !known_source(source) ||
        source != input->selected || !new_sample_proven ||
        stamp_us > now_us ||
        (input->last_sample.valid && stamp_us <= input->last_sample.stamp_us) ||
        !isfinite(speed_m_s) || !isfinite(yaw_rate_rad_s) ||
        fabsf(speed_m_s) > input->config.max_speed_m_s ||
        fabsf(yaw_rate_rad_s) > input->config.max_yaw_rate_rad_s) {
        return false;
    }

    /*
     * tick()が通信断をまだ観測していなくても、前回の新規入力から
     * 規定期限以上の空白がある場合、復帰前のニュートラル確認を要求する。
     * 単一の長時間据置きニュートラル報告だけでは復帰させない。
     */
    if (input->last_sample.valid &&
        stamp_us - input->last_sample.stamp_us > input->config.input_timeout_us) {
        require_neutral(input);
    }

    if (input->waiting_neutral) {
        if (!neutral_sample(input, speed_m_s, yaw_rate_rad_s)) {
            /* 操作されている間はニュートラル確認タイマを進めない。 */
            input->neutral_start_valid = false;
            input->neutral_sample_count = 0u;
        } else if (!input->neutral_start_valid) {
            input->neutral_start_valid = true;
            input->neutral_start_us = stamp_us;
            input->neutral_sample_count = 1u;
        } else {
            /*
             * 2つ以上の独立した新規ニュートラルサンプルと継続時間の両方を要求。
             * 途中に非ニュートラルが入ったら上記分岐で最初からやり直す。
             */
            input->neutral_sample_count = 2u;
            if (stamp_us - input->neutral_start_us >=
                input->config.neutral_hold_us) {
                input->waiting_neutral = false;
            }
        }
    }
    input->last_sample = (BOperatorSample){
        true, stamp_us, speed_m_s, yaw_rate_rad_s
    };
    return true;
}

bool b_operator_tick(BOperatorInput *input, BMode mode,
                     uint64_t now_us, BOperatorTarget *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (input == NULL || !input->initialized || !known_mode(mode)) {
        return false;
    }
    if (input->tick_valid && now_us < input->last_tick_us) {
        /* 時計逆行時は失敗・ゼロ出力にし、前回のタイムスタンプを
         * 書き換えて見かけ上の鮮度を回復することを防ぐ。 */
        return false;
    }

    bool fresh = sample_recent(input, now_us);
    if (!fresh && (!input->waiting_neutral ||
                   input->neutral_start_valid ||
                   input->neutral_sample_count != 0u)) {
        /* 有効な前回指令が切れた後は、復帰しても自動再開を許可しない。 */
        require_neutral(input);
    }

    const bool ready = fresh && !input->waiting_neutral;
    const bool drive_target = ready && mode == B_MODE_BALANCING;
    /*
     * 実機のデッドゾーン値は外部設定。確認済みの中立域内は目標を0へ落とし、
     * ニュートラル成立直後に微小な動作要求が残らないようにする。
     */
    const float requested_speed =
        drive_target &&
        fabsf(input->last_sample.speed_m_s) > input->config.neutral_speed_m_s ?
        input->last_sample.speed_m_s : 0.0f;
    const float requested_yaw =
        drive_target &&
        fabsf(input->last_sample.yaw_rate_rad_s) > input->config.neutral_yaw_rad_s ?
        input->last_sample.yaw_rate_rad_s : 0.0f;

    /*
     * DISARMED/FAULTはモータ出力の許可ではないので、操作目標を即時ゼロへ。
     * BALANCING中のリンク喪失では、姿勢保持を妨げる急な目標変化を避け、
     * 速度とYawの両方をそれぞれ設定された変化率でゼロへ戻す。
     */
    if (mode != B_MODE_BALANCING) {
        input->target_speed_m_s = 0.0f;
        input->target_yaw_rate_rad_s = 0.0f;
    } else if (input->tick_valid && now_us > input->last_tick_us) {
        const float dt_s = (float)(now_us - input->last_tick_us) * 0.000001f;
        float speed;
        float yaw;
        if (!b_rate_limit(input->target_speed_m_s, requested_speed,
                          input->config.speed_slew_m_s2, dt_s, &speed) ||
            !b_rate_limit(input->target_yaw_rate_rad_s, requested_yaw,
                          input->config.yaw_slew_rad_s2, dt_s, &yaw)) {
            /* 不正な計算結果を実指令として残さない。
             * FALSEはこのティック全体の処理失敗を上位に通知する。 */
            input->target_speed_m_s = 0.0f;
            input->target_yaw_rate_rad_s = 0.0f;
            return false;
        }
        input->target_speed_m_s = speed;
        input->target_yaw_rate_rad_s = yaw;
    }

    input->tick_valid = true;
    input->last_tick_us = now_us;

    /* ARM/CLEAR_FAULTの出力項目を意図的に持たせない。
     * USB/CANの操作を復帰させても上位の状態機械は自動ARMできない。 */
    *out = (BOperatorTarget){
        .source = input->selected,
        .source_fresh = fresh,
        .operator_ready = ready,
        .waiting_neutral = input->waiting_neutral,
        .speed_m_s = input->target_speed_m_s,
        .yaw_rate_rad_s = input->target_yaw_rate_rad_s,
    };
    return true;
}
