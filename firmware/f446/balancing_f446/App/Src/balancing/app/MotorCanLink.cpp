#include "balancing/app/MotorCanLink.hpp"

#include "balancing/app/MotorService.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

#include <algorithm>
#include <limits>
#include "balancing/communication/C620Codec.hpp"

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

MotorCanLink::MotorCanLink(MotorService& motors, CanTxScheduler& can1Tx) noexcept
    : motors_{motors}, can1Tx_{can1Tx}
{
}

void MotorCanLink::submitPending(std::uint64_t nowUs) noexcept
{
    const auto requests = motors_.pendingRs06Requests();
    for (std::size_t axis = 0; axis < requests.size(); ++axis) {
        if (!requests[axis]) {
            continue;
        }
        const auto& request = *requests[axis];
        const auto token = encodeToken(request.token);
        const auto& frame = request.frame;

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

void MotorCanLink::onScheduledTxProgress(const ScheduledTxProgress& progress, std::uint64_t nowUs) noexcept
{
    if (progress.result != ScheduledTxResult::Accepted) {
        return;
    }
    const auto queued = std::find(queuedStops_.begin(), queuedStops_.end(), progress.token);
    if (queued == queuedStops_.end()) {
        return;
    }

    // 受付後は軸側とスケジューラが同じトークンで終端を待つ。
    *queued = {};
    motors_.onRs06TxResult(decodeToken(progress.token), Rs06TxResult::Accepted, nowUs);
}

void MotorCanLink::onCanReceive(CanBus bus, CanRxTarget target, const CanFrame& frame,
                              std::uint64_t atUs) noexcept
{
    // RouterがバスとIDで分類済み。内容の復号と軸への割り当てはMotorServiceに任せる。
    // CAN2の既知軸IDはDLC不正でもMotorServiceへ知らせる。
    // RouterのUnmatchedを無条件に捨てると、その軸の角度欠落を見逃す。
    const bool knownWheel = bus == CanBus::Can2 && !frame.extended &&
                            (frame.id == 0x201u || frame.id == 0x202u);
    const bool rs06 = bus == CanBus::Can1 && target == CanRxTarget::Rs06Status;
    // 新規の自作CAN2分類を既存C620デコーダへ誤配送しない。
    // 既知車輪の短いDLCは欠落検出のため、引き続き機器サービスへ通知する。
    if (!rs06 && !knownWheel) {
        return;
    }
    if (rs06) {
        motors_.onRs06Frame(frame, atUs);
    } else if (knownWheel) {
        motors_.onC620Frame(frame, atUs);
    }
}

void MotorCanLink::onCanTransmit(CanBus bus, TxToken token, CanTxOutcome outcome,
                               std::uint64_t atUs) noexcept
{
    // 同じモータIDとシリアルを含んでいても、別バスや他の発行元は軸の状態を進めない。
    if (bus == CanBus::Can1 && (token.origin & ~kMotorIdMask) == kRs06Origin) {
        motors_.onRs06TxResult(decodeToken(token), toRs06Outcome(outcome), atUs);
    }
}

void MotorCanLink::onCanPortStatus(CanBus bus, CanEventKind kind, std::uint64_t) noexcept
{
    if (bus == CanBus::Can1) {
        if (kind == CanEventKind::BusOff || kind == CanEventKind::BusRecovered) {
            motors_.setCan1BusOff(kind == CanEventKind::BusOff);
        } else if (kind == CanEventKind::RxLost) {
            motors_.reportCan1RxLost();
        }
    } else if (bus == CanBus::Can2) {
        if (kind == CanEventKind::BusOff || kind == CanEventKind::BusRecovered) {
            motors_.setCan2BusOff(kind == CanEventKind::BusOff);
        } else if (kind == CanEventKind::RxLost) {
            motors_.reportCan2RxLost();
        }
    }
}

MotorPeriodicTx::MotorPeriodicTx(CanTxScheduler& a,CanTxScheduler& b,std::uint32_t origin) noexcept
    :can1_{a},can2_{b},origin_{origin} {}

bool MotorPeriodicTx::queue(CanBus bus,const std::array<CanFrame,4>& frames,
                            std::size_t count,std::uint64_t now) noexcept
{
    const bool rs=bus==CanBus::Can1;
    if ((!rs && bus!=CanBus::Can2) || count!=(rs?4u:1u) || !origin_) return false;
    const std::uint64_t period=rs?4000u:2000u;
    auto& generation=rs?generation1_:generation2_;
    constexpr auto max=std::numeric_limits<std::uint64_t>::max();
    if (generation==max || serial_>max-count || now>max-period) return false;
    PeriodicTxBatch batch{++generation,now+period,{}};
    const auto start=rs?0u:4u;
    for (std::size_t i=0;i<count;++i) {
        batch.requests[i]=PeriodicTxRequest{frames[i],{++serial_,origin_}};
        ++status_[start+i].queued;
    }
    const auto result=(rs?can1_:can2_).replacePeriodic(batch,now);
    // Invalid/StaleGenerationはSchedulerが旧予約を保持する。
    // 置換・期限切れ・停止による破棄の場合だけ旧予約を消す。
    const bool replaced=result==PeriodicBatchResult::Queued ||
        result==PeriodicBatchResult::Expired || result==PeriodicBatchResult::StopPending;
    for (std::size_t i=0;i<count;++i) {
        const auto axis=start+i;
        if (replaced && queued_[axis].serial) { ++status_[axis].dropped;queued_[axis]={}; }
        if (result==PeriodicBatchResult::Queued) {
            queued_[axis]=latest_[axis]=batch.requests[i]->token;
            queuedDriving_[axis]=driving_;
        } else ++status_[axis].dropped;
    }
    return result==PeriodicBatchResult::Queued;
}

bool MotorPeriodicTx::queueZero(std::uint64_t now) noexcept
{
    const auto zero=C620Codec::encodeCurrent({},20.0f);
    if (!zero || !queue(CanBus::Can2,{*zero},1,now)) return false;
    // 駆動中でも停止用ゼロは駆動指令の証跡にしない。
    queuedDriving_[4]=false;
    return true;
}

ScheduledTxProgress MotorPeriodicTx::service(CanBus bus,MotorCanLink& link,std::uint64_t now) noexcept
{
    const bool rs=bus==CanBus::Can1;
    if (!rs && bus!=CanBus::Can2) return {};
    const auto progress=(rs?can1_:can2_).service(now);
    if (rs) link.onScheduledTxProgress(progress,now);
    // 他の所有者の受付や終端は数えない。1対象につき同時受付は1件。
    if (progress.result==ScheduledTxResult::Accepted ||
        progress.result==ScheduledTxResult::Expired ||
        progress.result==ScheduledTxResult::Invalid) {
        for (unsigned i=rs?0u:4u;i<(rs?4u:5u);++i) {
            if (!queued_[i].serial || !(queued_[i]==progress.token)) continue;
            if (progress.result==ScheduledTxResult::Accepted) {
                accepted_[i]=progress.token;
                acceptedAt_[i]=now;
                acceptedDriving_[i]=queuedDriving_[i];
                ++status_[i].accepted;
            } else {
                ++status_[i].dropped;
                if (progress.result==ScheduledTxResult::Invalid) fault_=true;
            }
            queued_[i]={};
            queuedDriving_[i]=false;
        }
    }
    return progress;
}

void MotorPeriodicTx::onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,std::uint64_t at) noexcept
{
    if (!token.serial || token.origin!=origin_) return;
    const bool rs=bus==CanBus::Can1;
    if (!rs && bus!=CanBus::Can2) return;
    for (unsigned i=rs?0u:4u;i<(rs?4u:5u);++i) {
        if (!(accepted_[i]==token)) continue;
        // Routerと同じく逆行時刻で受付記録を解放しない。
        if (!acceptedAt_[i] || at<*acceptedAt_[i]) return;
        accepted_[i]={};
        acceptedAt_[i].reset();
        const bool monitored=acceptedDriving_[i] && driving_ && activeSince_;
        acceptedDriving_[i]=false;
        if (outcome==CanTxOutcome::Sent) {
            ++status_[i].sent;
            status_[i].lastSentUs=at;
            if (monitored) {
                const auto since=lastSent_[i].value_or(*activeSince_);
                const auto timeout=rs?rsTimeout_:wheelTimeout_;
                if (at<since || at-since>=timeout) fault_=true;
                else if (latest_[i]==token) lastSent_[i]=at;
            }
        } else {
            ++status_[i].failed;
            // 通常停止専用でも明示TX errorを報告。SILの非駆動中は
            // 呼出側が従来通り運転監視の対象外として扱える。
            if (outcome==CanTxOutcome::TransmitError && (!activeSince_ || monitored)) fault_=true;
        }
        return;
    }
}

void MotorPeriodicTx::beginActive(std::uint64_t now,std::uint64_t rs,std::uint64_t wheel) noexcept
{
    if (activeSince_) return;
    activeSince_=now;rsTimeout_=rs;wheelTimeout_=wheel;lastSent_={};
}
void MotorPeriodicTx::markDriving() noexcept
{
    driving_=true;
}
bool MotorPeriodicTx::deadlineFault(std::uint64_t now) const noexcept
{
    if (!activeSince_) return false;
    for (unsigned i=0;i<5;++i) {
        const auto since=lastSent_[i].value_or(*activeSince_);
        const auto timeout=i<4?rsTimeout_:wheelTimeout_;
        if (!timeout || now<since || now-since>=timeout) return true;
    }
    return false;
}
void MotorPeriodicTx::withdraw() noexcept
{
    can1_.invalidatePeriodic();can2_.invalidatePeriodic();
    for (unsigned i=0;i<5;++i) if (queued_[i].serial) ++status_[i].dropped;
    queued_={};latest_={};queuedDriving_={};acceptedDriving_={};
    // 中止済み受付の終端も診断累計へ残す。ただし再駆動の監視には使わない。
    lastSent_={};activeSince_.reset();driving_=false;
    // Schedulerは中止要求を出した受付済みトークンを終端まで保持する。
    // 上位の駆動鮮度はここで切り離し、旧終端による回復を禁止する。
}
bool MotorPeriodicTx::takeFault() noexcept
{
    const bool result=fault_;fault_=false;return result;
}

} // namespace balancing
