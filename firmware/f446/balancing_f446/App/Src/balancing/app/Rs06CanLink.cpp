#include "balancing/app/Rs06CanLink.hpp"

#include "balancing/app/MotorService.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

#include <algorithm>

namespace balancing {
namespace {

// 転送トークンの発行元キーは上位を区別する。下位8 bitだけがRS06のモータID。
// CANフレームに載るCAN IDとは独立で、各軸のシリアルは元の値を保持する。
constexpr std::uint32_t kRs06Origin = 0x100u;
constexpr std::uint32_t kMotorIdMask = 0xffu;

TxToken encodeToken(Rs06TxToken token) noexcept
{
    return {token.serial, kRs06Origin | token.motorId};
}

Rs06TxToken decodeToken(TxToken token) noexcept
{
    return {static_cast<std::uint8_t>(token.origin & kMotorIdMask), token.serial};
}

CanFrame toCanFrame(const BMotorCanFrame& source) noexcept
{
    CanFrame frame{source.id, {}, source.dlc, source.extended};
    std::copy_n(source.data, frame.data.size(), frame.data.begin());
    return frame;
}

Rs06TxResult toRs06Outcome(CanTxOutcome outcome) noexcept
{
    switch (outcome) {
    case CanTxOutcome::Sent:
        return Rs06TxResult::Completed;
    case CanTxOutcome::TransmitError:
        return Rs06TxResult::Failed;
    case CanTxOutcome::ArbitrationLost:
    case CanTxOutcome::NotTransmitted:
        return Rs06TxResult::RetryableFailure;
    }
    return Rs06TxResult::Failed;
}

} // namespace

Rs06CanLink::Rs06CanLink(MotorService& motors, CanTxScheduler& can1Tx) noexcept
    : motors_{motors}, can1Tx_{can1Tx}
{
}

void Rs06CanLink::submitPending(std::uint64_t nowUs) noexcept
{
    const auto requests = motors_.pendingRs06Requests();
    for (std::size_t axis = 0; axis < requests.size(); ++axis) {
        if (!requests[axis]) {
            continue;
        }
        const auto& request = *requests[axis];
        const auto token = encodeToken(request.token);
        const auto frame = toCanFrame(request.frame);

        if (request.command == Rs06Command::Stop) {
            // 専用枠に入れてからハード受付まで、MotorServiceは同じ候補を返す。
            // この間の再登録は重複で拒否されるため、保留トークンを1軸1件だけ持つ。
            if (queuedStops_[axis].serial != 0u) {
                continue;
            }
            const auto result = can1Tx_.enqueueStop(axis, frame, token);
            if (result == StopEnqueueResult::Queued) {
                queuedStops_[axis] = token;
            } else if (result == StopEnqueueResult::Invalid) {
                motors_.onRs06TxResult(request.token, Rs06TxResult::Failed, nowUs);
            }
            continue;
        }

        // Type 3を通常FIFOに積むと、停止後に旧セッションの有効化を送るおそれがある。
        // 未受付の候補はスケジューラに残さず、次周期の指令から取り直す。
        const auto result = can1Tx_.trySubmit(frame, token, nowUs);
        if (result == CanTxSubmitResult::Accepted) {
            motors_.onRs06TxResult(request.token, Rs06TxResult::Accepted, nowUs);
        } else if (result == CanTxSubmitResult::Invalid) {
            motors_.onRs06TxResult(request.token, Rs06TxResult::Failed, nowUs);
        }
    }
}

void Rs06CanLink::onScheduledTxProgress(const ScheduledTxProgress& progress, std::uint64_t nowUs) noexcept
{
    const auto result = progress.result;
    if (progress.token.serial == 0u ||
        (result != ScheduledTxResult::Accepted && result != ScheduledTxResult::Invalid)) {
        return;
    }
    const auto queued = std::find(queuedStops_.begin(), queuedStops_.end(), progress.token);
    if (queued == queuedStops_.end()) {
        return;
    }

    // 受付後は軸側とスケジューラが同じトークンで終端を待つ。
    // Invalidでも専用枠は再試行待ちへ移るので、軸が新しいシリアルを発行できる。
    *queued = {};
    const auto rs06Result = result == ScheduledTxResult::Accepted
                                ? Rs06TxResult::Accepted : Rs06TxResult::Failed;
    motors_.onRs06TxResult(decodeToken(progress.token), rs06Result, nowUs);
}

void Rs06CanLink::onCanReceive(CanBus bus, CanRxTarget target, const CanFrame& frame,
                              std::uint64_t atUs) noexcept
{
    if (bus != CanBus::Can1 || target != CanRxTarget::Rs06Status) {
        return;
    }
    // Routerはバスとフレーム外形を絞る。軸、主機ID、故障ビットはCodecとServiceで扱う。
    BMotorCanFrame report{};
    report.id = frame.id;
    report.dlc = frame.dlc;
    report.extended = frame.extended;
    std::copy(frame.data.begin(), frame.data.end(), report.data);
    motors_.onRs06Frame(report, atUs);
}

void Rs06CanLink::onCanTransmit(CanBus bus, TxToken token, CanTxOutcome outcome,
                               std::uint64_t atUs) noexcept
{
    // 同じモータIDとシリアルを含んでいても、別バスや他の発行元は軸の状態を進めない。
    if (bus == CanBus::Can1 && (token.origin & ~kMotorIdMask) == kRs06Origin) {
        motors_.onRs06TxResult(decodeToken(token), toRs06Outcome(outcome), atUs);
    }
}

void Rs06CanLink::onCanPortStatus(CanBus bus, CanEventKind kind, std::uint64_t) noexcept
{
    if (bus != CanBus::Can1) {
        return;
    }
    if (kind == CanEventKind::BusOff || kind == CanEventKind::BusRecovered) {
        motors_.setCan1BusOff(kind == CanEventKind::BusOff);
    } else if (kind == CanEventKind::RxLost) {
        motors_.reportCan1RxLost();
    }
}

} // namespace balancing
