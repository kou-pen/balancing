#include "balancing/pi/CanPeer.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace balancing;
using namespace balancing::protocol;
using namespace balancing::pi;
#define CHECK(x) do { if (!(x)) { std::cerr << "failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while(false)

template<class T>
T receive(const std::optional<CanFrame>& frame) {
    CHECK(frame);
    auto incoming=decodeIncoming(*frame,false,false);
    CHECK(incoming && std::holds_alternative<T>(*incoming));
    return std::get<T>(*incoming);
}
void reject(const std::optional<CanFrame>& frame) {
    CHECK(frame && !decodeIncoming(*frame,false,false));
}

int main() {
    // 独立したワイヤ既知値を照合。送信元は外部から選択できない。
    auto p=powerRequest(PowerRequestKind::On,0x1234);
    CHECK(p && p->id==0x0840C801 && p->extended && p->dlc==2);
    CHECK(p->data[0]==0x12 && p->data[1]==0x34);
    CHECK(powerRequest(PowerRequestKind::Off,1)->id==0x0440C802);
    CHECK(!powerRequest(static_cast<PowerRequestKind>(0),1));
    auto stop=emergencyStop();
    CHECK(stop && stop->id==0x0004FC01 && stop->dlc==0 && stop->extended);
    reject(stop);reject(p);
    CHECK(receive<NodeId>(encodeEmergencyStop(NodeId::F446))==NodeId::F446);
    CHECK(receive<NodeId>(encodeEmergencyStop(NodeId::F303))==NodeId::F303);

    // POWER_RESULTは自機宛てだけ。Acceptedと実際の状態は別フィールド。
    auto status=encodePowerStatus({WirePowerState::On,0x12,0x3456,0xffff});
    auto ps=receive<PowerStatusPayload>(status);
    CHECK(ps.state==WirePowerState::On && ps.flags==0x12 && ps.faultBits==0x3456 && ps.sequence==0xffff);
    auto response=receive<PowerResult>(encodePowerResult({NodeId::Pi,PowerRequestKind::On,0x1234,ResultCode::Accepted,WirePowerState::Precharge}));
    CHECK(response.requestId==0x1234 && response.result==ResultCode::Accepted && response.state==WirePowerState::Precharge);
    reject(encodePowerResult({NodeId::F446,PowerRequestKind::On,0x1234,ResultCode::Accepted,WirePowerState::On}));
    auto result=receive<RemoteResult>(encodeRemoteResult({RemoteRequestKind::Arm,9,ResultCode::Accepted,WireRobotState::Arming}));
    CHECK(result.requestId==9 && result.state==WireRobotState::Arming);
    reject(encodeRemoteRequest({RemoteRequestKind::Arm,9,{}}));
    reject(encodeMotionCommand({1.0f,-0.2f}));

    // 初期監視に必要な型を個別に通す。未定義ビットを意味付けしない。
    auto rs=receive<RobotStatusPayload>(encodeDiagnostic(RobotStatusPayload{WireRobotState::Fault,WireInputSource::CanRemote,0x80,8,0x40}));
    CHECK(rs.state==WireRobotState::Fault && rs.source==WireInputSource::CanRemote && rs.flags==0x80 && rs.faultBits==8 && rs.validity==0x40);
    CHECK(receive<AttitudeAngle>(encodeDiagnostic(AttitudeAngle{1.0f,-2.0f})).rollRad==-2.0f);
    CHECK(receive<AttitudeRate>(encodeDiagnostic(AttitudeRate{1.0f,-2.0f})).pitchRadS==1.0f);
    CHECK(receive<YawRate>(encodeDiagnostic(YawRate{-3.0f})).yawRadS==-3.0f);
    CHECK(receive<ChassisState>(encodeDiagnostic(ChassisState{2.0f,0.25f})).velocityMps==0.25f);
    CHECK(receive<WheelState>(encodeDiagnostic(WheelState{2.0f,-1.0f})).rightRadS==-1.0f);
    CHECK(receive<LeftLegState>(encodeDiagnostic(LeftLegState{0.25f,1.0f})).lengthM==0.25f);
    CHECK(receive<RightLegState>(encodeDiagnostic(RightLegState{0.5f,-1.0f})).angleRad==-1.0f);
    CHECK(receive<ControlTarget>(encodeDiagnostic(ControlTarget{0.5f,-0.25f})).yawRateRadS==-0.25f);
    CHECK(receive<FaultPart1>(encodeDiagnostic(FaultPart1{7,100,8})).primaryCause==8);
    CHECK(receive<FaultPart2>(encodeDiagnostic(FaultPart2{7,8,0})).eventId==7);
    CHECK(receive<FaultPart3>(encodeDiagnostic(FaultPart3{7,100})).originalAtMs==100);
    reject(encodeDiagnostic(ControlTiming{2000,0}));
    reject(encodeDiagnostic(StreamSubscribe{2,20,1000,3}));
    reject(encodeDiagnostic(StreamUnsubscribe{2,3}));
    reject(encodeDiagnostic(StreamResult{3,0x80,ResultCode::Accepted}));
    reject(encodeDiagnostic(ParamRead{1,2}));
    reject(encodeDiagnostic(ParamValue{1,2,42}));
    reject(encodeDiagnostic(ParamError{1,2,ResultCode::SafetyRejected}));

    // 同じペイロードでもRTR、エラー、標準ID、予約領域、DLCが違えば拒否。
    CHECK(!decodeIncoming(*status,true,false));
    CHECK(!decodeIncoming(*status,false,true));
    auto broken=*status;broken.extended=false;CHECK(!decodeIncoming(broken,false,false));
    broken=*status;broken.dlc=9;CHECK(!decodeIncoming(broken,false,false));
    broken=*status;broken.dlc=7;CHECK(!decodeIncoming(broken,false,false));
    broken=*status;broken.id|=0x20000000;CHECK(!decodeIncoming(broken,false,false));
    broken=*status;broken.data[7]=1;CHECK(!decodeIncoming(broken,false,false));
    broken=*status;broken.data[0]=4;CHECK(!decodeIncoming(broken,false,false));
    CanFrame motor{};motor.id=0x201;motor.dlc=8;CHECK(!decodeIncoming(motor,false,false));
    auto angle=encodeDiagnostic(AttitudeAngle{0,0});CHECK(angle);
    angle->data[0]=0x7f;angle->data[1]=0xc0; // big-endian NaN
    CHECK(!decodeIncoming(*angle,false,false));
    std::cout<<"Pi CAN peer: supported types, directions, malformed frames PASS\n";
}
