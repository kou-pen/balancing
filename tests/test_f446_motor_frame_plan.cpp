#include "balancing/communication/can/MotorFramePlan.hpp"
#include "balancing/control/controller/MotorCommand.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

using namespace balancing;

namespace {
MotorCommandConfig fixture()
{
    MotorCommandConfig c{};
    for (auto& motor : c.rs06) {
        motor = {2.0f, 1.0f, 4.0f, true};
    }
    c.rs06[1].motorDirection = -1.0f;
    c.rs06[3].motorDirection = -1.0f;
    c.c620 = {{{0.4f, 1.0f, 3.0f, true}, {0.4f, -1.0f, 3.0f, true}}};
    return c;
}

BalanceControlOutput request()
{
    BalanceControlOutput c{};
    c.jointTorqueNm = {4.0f, 4.0f, -4.0f, -4.0f};
    c.wheelTorqueNm = {0.4f, 0.4f};
    return c;
}

RobotSequenceResult state(RobotState robot, DesiredMotorState desired, ArmSessionId session)
{
    return {robot, desired, session};
}

bool empty(const MotorFramePreview& preview)
{
    if (preview.can1Count != 0 || preview.can2Current.id != 0) {
        return false;
    }
    for (const auto& frame : preview.can1Type1) {
        if (frame.id != 0 || frame.dlc != 0) {
            return false;
        }
    }
    return true;
}

void testRegularFramePlanMatchesGoldenFrames()
{
    // 実機のモータ番号・伝達比ではない。ホスト上の4軸マッピングを明示する。
    MotorCommand motor{fixture()};
    MotorFramePlan planner{{11, 12, 13, 14}};
    const auto torque = motor.evaluate(request());
    assert(torque.status == CommandStatus::Ready);
    const auto preview = planner.compose(
        state(RobotState::Balancing, DesiredMotorState::DriveAllowed, 4), torque);
    assert(preview.status == MotorFramePreview::Status::Ready);
    assert(preview.fixtureOnly && preview.can1Count == 4);

    for (std::size_t i = 0; i < 4; ++i) {
        // 変換結果は+2,-2,-2,+2 Nm。旧Cの量子化と同じ独立期待値。
        constexpr std::array<std::uint32_t,4> ids{0x01871c0bu,0x0178e30cu,0x0178e30du,0x01871c0eu};
        const auto& actual = preview.can1Type1[i];
        assert(actual.id == ids[i]);
        assert(actual.extended && actual.dlc == 8);
        assert((actual.data==std::array<std::uint8_t,8>{0x80,0,0x80,0,0,0,0,0}));
        // Type1だけ。stop/enableのType3/4をフレームプランへ混ぜない。
        assert(((actual.id >> 24u) & 0x1fu) == 1u);
        assert((actual.id & 0xffu) == i + 11u);
        assert(actual.data[4] == 0 && actual.data[5] == 0 &&
               actual.data[6] == 0 && actual.data[7] == 0); // Kp=Kd=0
    }
    assert(preview.can2Current.id == 0x200u && !preview.can2Current.extended);
    assert(preview.can2Current.dlc == 8);
    assert((preview.can2Current.data==std::array<std::uint8_t,8>{0x03,0x33,0xfc,0xcd,0,0,0,0}));
    for (std::size_t i = 4; i < 8; ++i) {
        assert(preview.can2Current.data[i] == 0); // 未搭載C620 ID3/4は常に0
    }
}

void testDisarmAndArmingNeverReissueTorque()
{
    MotorCommand motor{fixture()};
    MotorFramePlan planner{{11, 12, 13, 14}};
    const auto torque = motor.evaluate(request());
    const auto stop = planner.compose(
        state(RobotState::Fault, DesiredMotorState::StopAll, 4), torque);
    const auto arming = planner.compose(
        state(RobotState::Arming, DesiredMotorState::EnableRs06, 5), torque);
    for (const auto& plan : {stop, arming}) {
        assert(plan.status == MotorFramePreview::Status::ZeroCurrentOnly);
        assert(plan.can1Count == 0 && plan.can2Current.id == 0x200u);
        for (const auto byte : plan.can2Current.data) {
            assert(byte == 0);
        }
    }
    // 古いセッションのDriveAllowedまたは未成立の数学入力は
    // CAN1の非ゼロType1を絶対に復活させない。
    for (const auto& wrong : {
        state(RobotState::Disarmed, DesiredMotorState::DriveAllowed, 4),
        state(RobotState::Balancing, DesiredMotorState::DriveAllowed, 0),
    }) {
        const auto result = planner.compose(wrong, torque);
        assert(result.status == MotorFramePreview::Status::NotDriveAllowed);
        assert(empty(result));
    }
    auto invalid = torque;
    invalid.status = CommandStatus::InvalidInput;
    assert(empty(planner.compose(state(RobotState::Balancing, DesiredMotorState::DriveAllowed, 4),
                                 invalid)));
    invalid = torque;
    invalid.fixtureOnly = false;
    assert(empty(planner.compose(state(RobotState::Balancing, DesiredMotorState::DriveAllowed, 4),
                                 invalid)));
}

void testMisconfigurationOrCodecErrorGivesNoPartialFrames()
{
    MotorCommand motor{fixture()};
    const auto valid = motor.evaluate(request());
    MotorFramePlan duplicate{{11, 12, 12, 14}};
    MotorFramePlan zero{{11, 0, 13, 14}};
    const auto driving = state(RobotState::Balancing, DesiredMotorState::DriveAllowed, 1);
    assert(duplicate.compose(driving, valid).status ==
           MotorFramePreview::Status::InvalidConfiguration);
    assert(empty(zero.compose(driving, valid)));

    MotorFramePlan planner{{11, 12, 13, 14}};
    auto overflow = valid;
    overflow.rs06[3].limitedMotorTorqueNm = 99.0f;
    const auto rejected = planner.compose(driving, overflow);
    assert(rejected.status == MotorFramePreview::Status::EncodingFailed);
    assert(empty(rejected)); // CAN1先頭3軸も見えない

    overflow = valid;
    overflow.c620[1].limitedCurrentA = std::numeric_limits<float>::quiet_NaN();
    const auto rejectedCurrent = planner.compose(driving, overflow);
    assert(rejectedCurrent.status == MotorFramePreview::Status::EncodingFailed);
    assert(empty(rejectedCurrent));
}

} // namespace

int main()
{
    testRegularFramePlanMatchesGoldenFrames();
    testDisarmAndArmingNeverReissueTorque();
    testMisconfigurationOrCodecErrorGivesNoPartialFrames();
    std::puts("PASS: six-motor CAN frame plan and stop/arming zero output");
}
