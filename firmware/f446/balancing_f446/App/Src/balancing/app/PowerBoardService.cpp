#include "balancing/app/PowerBoardService.hpp"
#include "balancing/communication/can/PowerBoardCodec.hpp"

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

bool PowerBoardService::acceptCanStatus(CanBus bus, CanRxTarget target, const CanFrame& frame,
                                        std::uint64_t receivedAtUs, std::uint64_t nowUs) noexcept
{
    // Routerの分類は外形だけ。別バスや操作結果を電源状態の更新に使わず、
    // Codecで送信元・宛先・DLC・列挙値・予約領域を改めて照合する。
    if (bus != CanBus::Can2 || target != CanRxTarget::PowerStatus) return false;
    const auto payload = protocol::decodePowerStatus(frame);
    if (!payload) return false;

    // ワイヤの値をサービスの状態へ明示変換する。未定義のビットは
    // 診断用に保存し、意味を推測してFAULTやreadyの条件に追加しない。
    PowerState state;
    switch (payload->state) {
    case protocol::WirePowerState::Off: state = PowerState::Off; break;
    case protocol::WirePowerState::Precharge: state = PowerState::Precharge; break;
    case protocol::WirePowerState::On: state = PowerState::On; break;
    case protocol::WirePowerState::Fault: state = PowerState::Fault; break;
    default: return false;
    }

    // 受信時刻の新規性・期限は既存の受理入口だけで判定する。
    // 連番の再起動・周回規則は未確定なので生値を保存し、時刻と混同しない。
    return acceptDecoded({state, receivedAtUs,
                          PowerWireMetadata{payload->flags, payload->faultBits, payload->sequence}}, nowUs);
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
