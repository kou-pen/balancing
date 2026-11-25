#include "balancing/app/RemoteControlService.hpp"

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
