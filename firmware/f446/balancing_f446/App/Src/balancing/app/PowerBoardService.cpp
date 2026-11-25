#include "balancing/app/PowerBoardService.hpp"

namespace balancing {
namespace { constexpr std::uint64_t timeoutUs = 500000; }

bool PowerBoardService::acceptDecoded(PowerStatusReport report, std::uint64_t nowUs) noexcept
{
    // 受信前・未来・重複・逆行・期限切れで鮮度を更新しない。
    if (report.receivedAtUs > nowUs || nowUs - report.receivedAtUs >= timeoutUs ||
        (latest_ && report.receivedAtUs <= latest_->receivedAtUs)) return false;
    switch (report.state) {
    case PowerState::Off: case PowerState::Precharge: case PowerState::On: break;
    case PowerState::Fault: faultPending_ = true; break;
    default: return false;
    }
    latest_ = report;
    return true;
}

PowerBoardSnapshot PowerBoardService::snapshot(std::uint64_t nowUs) const noexcept
{
    const bool fresh = latest_ && latest_->receivedAtUs <= nowUs &&
                       nowUs - latest_->receivedAtUs < timeoutUs;
    // 次の周期までにONへ戻っても、途中で受理したFAULT通知を失わない。
    const bool fault = faultPending_ || (latest_ && latest_->state == PowerState::Fault);
    return {latest_, fresh, fresh && latest_->state == PowerState::On && !fault, fault};
}

FaultFlags PowerBoardService::takeObservedFaults(std::uint64_t nowUs) noexcept
{
    const bool fault = snapshot(nowUs).faultObserved;
    faultPending_ = false;
    return fault ? FaultFlags::ExternalFault : FaultFlags::None;
}
}
