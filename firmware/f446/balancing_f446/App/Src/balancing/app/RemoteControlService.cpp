#include "balancing/app/RemoteControlService.hpp"
#include "balancing/communication/can/RemoteControlCodec.hpp"

namespace balancing {
RemoteControlService::RemoteControlService(const BOperatorInputConfig& config) noexcept
{
    // C11の回帰基準を一つだけ使う。未設定の場合は入力も目標も無効。
    (void)b_operator_init(&input_, &config);
}

bool RemoteControlService::selectSource(BOperatorSource source, RobotState state) noexcept
{
    // ARMINGをDISARMEDへ変換して切替を許可してはならない。
    return state == RobotState::Disarmed && b_operator_select(&input_, source, B_MODE_DISARMED);
}

bool RemoteControlService::acceptDecoded(BOperatorSource source, const BOperatorSample& sample,
                                         bool newSampleProven, std::uint64_t nowUs) noexcept
{
    return sample.valid && b_operator_accept(&input_, source, sample.speed_m_s,
        sample.yaw_rate_rad_s, newSampleProven, sample.stamp_us, nowUs);
}

bool RemoteControlService::acceptCanMotion(const CanFrame& frame,std::uint64_t receivedAtUs,
                                           std::uint64_t nowUs) noexcept
{
    // 受信キューで期限を超えた入力を中立確認や鮮度更新に利用しない。
    if (!input_.initialized || receivedAtUs>nowUs ||
        nowUs-receivedAtUs>input_.config.input_timeout_us) return false;
    const auto motion=protocol::decodeMotionCommand(frame);
    return motion && acceptDecoded(B_OPERATOR_CAN_REMOTE,
        {true,receivedAtUs,motion->forwardVelocityMps,motion->yawRateRadS},true,nowUs);
}

std::optional<BOperatorSource> RemoteControlService::selectedSource() const noexcept
{
    return input_.initialized ? std::optional<BOperatorSource>{input_.selected} : std::nullopt;
}

void RemoteControlService::invalidateCanInput() noexcept
{
    if (!input_.initialized || input_.selected!=B_OPERATOR_CAN_REMOTE) return;
    // 欠落区間の中立入力は証明できない。目標値を瞬時に書き換えず、既存tickで減速する。
    input_.last_sample={};
    input_.waiting_neutral=true;
    input_.neutral_start_valid=false;
    input_.neutral_sample_count=0;
}

std::optional<BOperatorTarget> RemoteControlService::update(RobotState state, std::uint64_t nowUs) noexcept
{
    BMode mode{};
    switch (state) {
    case RobotState::Disarmed: mode = B_MODE_DISARMED; break;
    case RobotState::Arming: mode = B_MODE_DISARMED; break; // 目標ゼロ、切替判定とは別
    case RobotState::Balancing: mode = B_MODE_BALANCING; break;
    case RobotState::Fault: mode = B_MODE_FAULT; break;
    default: return std::nullopt;
    }
    BOperatorTarget out{};
    if (!b_operator_tick(&input_, mode, nowUs, &out)) return std::nullopt;
    return out;
}
}
