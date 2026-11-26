#include "balancing/app/Can2ServiceLink.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"
#include <limits>

namespace balancing {
namespace {
using namespace protocol;
WireRobotState wireState(RobotState s) noexcept
{
    switch(s) {
    case RobotState::Disarmed: return WireRobotState::Disarmed;
    case RobotState::Arming: return WireRobotState::Arming;
    case RobotState::Balancing: return WireRobotState::Balancing;
    case RobotState::Fault: return WireRobotState::Fault;
    }
    return WireRobotState::Fault;
}
}
Can2ServiceLink::Can2ServiceLink(PowerBoardService& power,RemoteControlService& remote,
                                 DiagnosticsService& diagnostics,Can2ServiceConfig config) noexcept
    : power_{power},remote_{remote},diagnostics_{diagnostics},config_{config}
{
    // 数値候補を実機値として固定しない。外部が全条件を与えた非駆動試験のみ有効。
    enabled_=config_.requestLifetimeUs && config_.basePeriodUs && config_.telemetryLifetimeUs &&
        config_.minimumDetailPeriodMs && config_.maximumLeaseMs &&
        config_.responseCapacity && config_.responseCapacity<=responses_.size() && config_.tokenOrigin;
}
bool Can2ServiceLink::recent(std::uint64_t atUs,std::uint64_t nowUs) const noexcept
{
    return nowUs>=atUs && nowUs-atUs<config_.requestLifetimeUs;
}
void Can2ServiceLink::queueResponse(std::optional<CanFrame> frame,std::uint64_t nowUs) noexcept
{
    // 応答が詰まっても操作を保存し直さない。満杯時は欠落を記録するだけ。
    if (!frame || responseCount_>=config_.responseCapacity) {
        countDroppedResponse();
        return;
    }
    responses_[responseCount_++]={*frame,nowUs};
}
void Can2ServiceLink::countDroppedResponse() noexcept
{
    if (droppedResponses_!=std::numeric_limits<std::uint32_t>::max()) ++droppedResponses_;
}
void Can2ServiceLink::removeResponse() noexcept
{
    for (std::size_t i=1;i<responseCount_;++i) responses_[i-1]=responses_[i];
    --responseCount_;
}
void Can2ServiceLink::remoteResult(const Request& request,protocol::ResultCode code,
                                   RobotState state,std::uint64_t nowUs) noexcept
{
    queueResponse(protocol::encodeRemoteResult({request.value.kind,request.value.requestId,
                                               code,wireState(state)}),nowUs);
}
void Can2ServiceLink::receive(CanBus bus,CanRxTarget target,const CanFrame& frame,
                              std::uint64_t atUs,std::uint64_t nowUs) noexcept
{
    using namespace protocol;
    if (!enabled_ || bus!=CanBus::Can2 || atUs>nowUs) return;
    // 非常停止は他の受信の期限・応答FIFO・操作元の制約に依存させない。
    if (target==CanRxTarget::EmergencyStop) {
        const auto source=decodeEmergencyStop(frame);
        if (source && (*source==NodeId::F303 || *source==NodeId::Pi)) emergencyPending_=true;
        return;
    }
    if (target==CanRxTarget::RemoteRequest) {
        const auto req=decodeRemoteRequest(frame);
        if (!req) return;
        // DISARMも通常の操作要求と別枠。古い非停止操作を再実行することはない。
        if (req->kind==RemoteRequestKind::Disarm) {
            if (!disarm_) disarm_=Request{*req,atUs};
            else remoteResult({*req,atUs},ResultCode::Busy,previousState_,nowUs);
            return;
        }
    }
    if (busOff_ || (rxBarrier_ && atUs<=*rxBarrier_)) return;
    switch(target) {
    case CanRxTarget::PowerStatus:
        (void)power_.acceptCanStatus(bus,target,frame,atUs,nowUs);
        break;
    case CanRxTarget::RemoteMotion:
        if (!motionBarrier_ || atUs>*motionBarrier_) (void)remote_.acceptCanMotion(frame,atUs,nowUs);
        break;
    case CanRxTarget::RemoteRequest: {
        const auto req=decodeRemoteRequest(frame);
        if (!req || !recent(atUs,nowUs)) return;
        if (pending_) remoteResult({*req,atUs},ResultCode::Busy,previousState_,nowUs);
        else pending_=Request{*req,atUs};
        break;
    }
    case CanRxTarget::DiagnosticRequest:
        if (recent(atUs,nowUs)) handleDiagnostic(frame,nowUs);
        break;
    case CanRxTarget::PowerResult: {
        const auto result=decodePowerResult(frame);
        // 応答は要求番号と種別の両方を照合する。ON報告と状態管理を混ぜない。
        if (result && result->destination==NodeId::F446 && powerExchange_ &&
            powerExchange_->sentAtUs && atUs>=*powerExchange_->sentAtUs &&
            recent(powerExchange_->atUs,nowUs) &&
            result->kind==powerExchange_->kind && result->requestId==powerExchange_->id) {
            powerResult_=*result;
            powerExchange_.reset();
        }
        break;
    }
    default: break;
    }
}
void Can2ServiceLink::onPortStatus(CanBus bus,CanEventKind kind,std::uint64_t atUs) noexcept
{
    if (!enabled_ || bus!=CanBus::Can2) return;
    if (kind==CanEventKind::BusRecovered) {
        busOff_=false;
        if (!rxBarrier_ || atUs>*rxBarrier_) rxBarrier_=atUs;
    } else if (kind==CanEventKind::RxLost || kind==CanEventKind::BusOff) {
        if (kind==CanEventKind::BusOff) busOff_=true;
        // 同時刻の欠落前後を判別できない。新規時刻からのみ状態を復帰させる。
        if (!rxBarrier_ || atUs>*rxBarrier_) rxBarrier_=atUs;
        remote_.invalidateCanInput();
        power_.invalidateCanStatus();
        pending_.reset();
        powerExchange_.reset();
        while (responseCount_) { removeResponse(); countDroppedResponse(); }
        leases_={};
        telemetry_={};
        telemetryAtUs_.reset();
        // 非常停止・DISARMは落とさない。
    }
}
Can2CycleInput Can2ServiceLink::takeCycleInput(std::uint64_t nowUs,RobotState state,
                                               bool clearFaultAuthorized) noexcept
{
    using namespace protocol;
    Can2CycleInput input{};
    if (!enabled_) return input;
    previousState_=state;
    if (emergencyPending_) input.faults=FaultFlags::EmergencyStop;
    emergencyPending_=false;
    executingDisarm_=disarm_;
    disarm_.reset();
    input.operations.disarm=executingDisarm_.has_value();
    const auto request=pending_;
    pending_.reset();
    if (!request || !recent(request->atUs,nowUs)) return input;
    if (input.operations.disarm || input.faults!=FaultFlags::None) {
        remoteResult(*request,ResultCode::SafetyRejected,state,nowUs);
        return input;
    }
    switch(request->value.kind) {
    case RemoteRequestKind::SelectSource: {
        const auto source=*request->value.source==WireInputSource::UsbDualSense
            ? OperatorSource::UsbDualSense : OperatorSource::CanRemote;
        const auto old=remote_.selectedSource();
        const bool accepted=remote_.selectSource(source,state);
        // 切替前に採取された未回収フレームを新しい操作元の入力として扱わない。
        if (accepted && old!=source) motionBarrier_=nowUs;
        remoteResult(*request,accepted?(old==source?ResultCode::AlreadyInState:ResultCode::Accepted)
                                     :ResultCode::SafetyRejected,state,nowUs);
        break;
    }
    case RemoteRequestKind::Arm:
        if (remote_.selectedSource()!=OperatorSource::CanRemote) {
            remoteResult(*request,ResultCode::Unauthorized,state,nowUs);
        } else {
            input.operations.arm=true;
            executing_=request;
        }
        break;
    case RemoteRequestKind::ClearFault:
        input.operations.clearFault=true;
        input.operations.clearFaultAuthorized=clearFaultAuthorized;
        executing_=request;
        break;
    default: break;
    }
    return input;
}
void Can2ServiceLink::completeCycle(const RobotSequenceResult& result,std::uint64_t nowUs) noexcept
{
    using namespace protocol;
    if (!enabled_) return;
    if (executing_) {
        auto code=ResultCode::SafetyRejected;
        if (executing_->value.kind==RemoteRequestKind::Arm) {
            if (previousState_==RobotState::Disarmed && result.state==RobotState::Arming)
                code=ResultCode::Accepted;
            else if (previousState_==result.state &&
                     (result.state==RobotState::Arming || result.state==RobotState::Balancing))
                code=ResultCode::AlreadyInState;
        } else if (previousState_==RobotState::Fault && result.state==RobotState::Disarmed) {
            code=ResultCode::Accepted;
        } else if (previousState_==RobotState::Disarmed && result.state==RobotState::Disarmed) {
            code=ResultCode::AlreadyInState;
        }
        remoteResult(*executing_,code,result.state,nowUs);
        executing_.reset();
    }
    if (executingDisarm_) {
        const auto code=result.desiredMotorState==DesiredMotorState::StopAll
            ? (previousState_==result.state?ResultCode::AlreadyInState:ResultCode::Accepted)
            : ResultCode::SafetyRejected;
        remoteResult(*executingDisarm_,code,result.state,nowUs);
        executingDisarm_.reset();
    }
    previousState_=result.state;
}
void Can2ServiceLink::handleDiagnostic(const CanFrame& frame,std::uint64_t nowUs) noexcept
{
    using namespace protocol;
    const auto decoded=decodeDiagnostic(frame);
    if (!decoded) return;
    // 詳細配信の対象は仕様の2〜9だけ。期間は試験設定内で明示する。
    if (const auto* req=std::get_if<StreamSubscribe>(&*decoded)) {
        const auto leaseUs=std::uint64_t{req->leaseMs}*1000u;
        const bool valid=req->registerId>=2 && req->registerId<=9 &&
            req->periodMs>=config_.minimumDetailPeriodMs &&
            std::uint64_t{req->periodMs}*1000u<config_.basePeriodUs &&
            req->leaseMs && req->leaseMs<=config_.maximumLeaseMs &&
            leaseUs<=std::numeric_limits<std::uint64_t>::max()-nowUs;
        if (valid) leases_[req->registerId-1]={req->periodMs,nowUs+leaseUs};
        queueResponse(encodeDiagnostic(StreamResult{req->requestId,0x80,
            valid?ResultCode::Accepted:ResultCode::InvalidRequest}),nowUs);
    } else if (const auto* req=std::get_if<StreamUnsubscribe>(&*decoded)) {
        const bool valid=req->registerId>=2 && req->registerId<=9;
        if (valid) leases_[req->registerId-1]={};
        queueResponse(encodeDiagnostic(StreamResult{req->requestId,0x81,
            valid?ResultCode::Accepted:ResultCode::InvalidRequest}),nowUs);
    } else if (const auto* req=std::get_if<ParamRead>(&*decoded)) {
        // 番号別の型・単位が未確定。値を捏造せず、明示エラーで応答。
        queueResponse(encodeDiagnostic(ParamError{req->paramId,req->requestId,
                                                  ResultCode::InvalidRequest}),nowUs);
    }
}
void Can2ServiceLink::publishTelemetry(const Can2Telemetry& frames,std::uint64_t nowUs) noexcept
{
    using namespace protocol;
    if (!enabled_ || (telemetryAtUs_ && nowUs<*telemetryAtUs_)) return;
    telemetry_={};
    telemetryAtUs_=nowUs;
    for (std::size_t i=0;i<frames.size();++i) {
        if (!frames[i]) continue;
        const auto id=decodeId(frames[i]->id);
        if (id && id->source==NodeId::F446 && id->destination==NodeId::Pi &&
            id->type==DataType::Diagnostics && id->reg==i+1 && decodeDiagnostic(*frames[i]))
            telemetry_[i]=frames[i];
    }
}
bool Can2ServiceLink::requestMockPower(protocol::PowerRequestKind kind,std::uint16_t id,
                                       std::uint64_t nowUs) noexcept
{
    if (!enabled_ || !config_.allowMockPowerRequests || busOff_ ||
        (powerExchange_ && (nowUs<powerExchange_->atUs || recent(powerExchange_->atUs,nowUs))) ||
        !protocol::encodePowerRequest({protocol::NodeId::F446,kind,id})) return false;
    powerExchange_=PowerExchange{kind,id,nowUs};
    powerResult_.reset();
    return true;
}
void Can2ServiceLink::onTxTerminal(CanBus bus,TxToken token,CanTxOutcome outcome,
                                  std::uint64_t atUs) noexcept
{
    if (!enabled_ || bus!=CanBus::Can2) return;
    if (powerExchange_ && powerExchange_->token && *powerExchange_->token==token) {
        if (outcome==CanTxOutcome::Sent) powerExchange_->sentAtUs=atUs;
        else powerExchange_.reset(); // 自動再送しない
        return;
    }
    if (responseInFlight_ && *responseInFlight_==token) {
        if (outcome!=CanTxOutcome::Sent) countDroppedResponse();
        responseInFlight_.reset();
        return;
    }
    (void)diagnostics_.onTerminal(token,outcome);
}

namespace {
std::optional<CanFrame> faultFrame(const DiagnosticPart& part) noexcept
{
    using namespace protocol;
    if (part.event.kind!=DiagnosticKind::Fault) return std::nullopt;
    const auto& e=part.event.fault;
    const auto id=static_cast<std::uint16_t>(e.eventSequence);
    const auto& f=e.record;
    // 内部イベントの下位16bitを3パート共通にし、元FAULT時刻と追加異常時刻を分離。
    switch(part.frameIndex) {
    case 0: return encodeDiagnostic(FaultPart1{id,e.occurredAtMs,static_cast<std::uint16_t>(f.primaryCause)});
    case 1: return encodeDiagnostic(FaultPart2{id,static_cast<std::uint16_t>(f.triggerFlags),
                                               static_cast<std::uint16_t>(f.subsequentFlags)});
    case 2: return encodeDiagnostic(FaultPart3{id,static_cast<std::uint32_t>(f.occurredAtUs/1000u)});
    default: return std::nullopt;
    }
}
}
void Can2ServiceLink::submitPending(CanTxScheduler& tx,std::uint64_t nowUs) noexcept
{
    using namespace protocol;
    if (!enabled_ || busOff_ || serial_==std::numeric_limits<std::uint64_t>::max()) return;
    // 期限切れは未受付の応答だけを捨てる。操作や電源要求は再実行しない。
    while(responseCount_ && !recent(responses_[0].atUs,nowUs)) {
        removeResponse();
        countDroppedResponse();
    }
    if (powerExchange_ && !recent(powerExchange_->atUs,nowUs)) powerExchange_.reset();
    const TxToken token{serial_+1,config_.tokenOrigin};
    const auto send=[&](const CanFrame& frame) {
        const auto result=tx.trySubmitBackground(frame,token,nowUs);
        if (result==CanTxSubmitResult::Accepted) ++serial_;
        return result==CanTxSubmitResult::Accepted;
    };
    // FAULTは最優先の背景データ。通常状態遷移のワイヤIDは未決定のため送らない。
    const auto part=diagnostics_.nextPending();
    if (part) if (const auto frame=faultFrame(*part)) {
        if (send(*frame)) (void)diagnostics_.onAccepted(*part,token);
        return;
    }
    if (responseCount_) {
        if (send(responses_[0].frame)) { responseInFlight_=token; removeResponse(); }
        return;
    }
    if (powerExchange_ && !powerExchange_->token) {
        const auto frame=encodePowerRequest({NodeId::F446,powerExchange_->kind,powerExchange_->id});
        if (frame && send(*frame)) powerExchange_->token=token;
        return;
    }
    // 最新の有効観測だけを周期配信。購読期限を過ぎたら基本周期へ戻る。
    if (!telemetryAtUs_ || nowUs<*telemetryAtUs_ ||
        nowUs-*telemetryAtUs_>=config_.telemetryLifetimeUs) return;
    for (std::size_t n=0;n<telemetry_.size();++n) {
        const auto i=(nextTelemetry_+n)%telemetry_.size();
        if (!telemetry_[i]) continue;
        const auto& lease=leases_[i];
        const auto period=nowUs<lease.untilUs?std::uint64_t{lease.periodMs}*1000u:config_.basePeriodUs;
        if (lastTelemetryTx_[i] && (nowUs<*lastTelemetryTx_[i] ||
            nowUs-*lastTelemetryTx_[i]<period)) continue;
        if (send(*telemetry_[i])) {
            lastTelemetryTx_[i]=nowUs;
            nextTelemetry_=(i+1)%telemetry_.size();
        }
        return;
    }
}
}
