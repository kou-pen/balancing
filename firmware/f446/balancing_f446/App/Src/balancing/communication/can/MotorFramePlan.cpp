#include "balancing/communication/can/MotorFramePlan.hpp"

#include <algorithm>

namespace balancing {
namespace {
// メーカー指定の電流表現上限。MotorCommandは別途、実機用のより小さい限界で
// 制限済みであることが前提。ここで物理上限として流用しない。
constexpr float kC620WireMaxA = 20.0f;

bool validMotorIds(const std::array<std::uint8_t, 4>& ids) noexcept
{
    // 順序は左A/B、右A/B。失敗時にIDを推測して埋めない。
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == 0 ||
            std::find(ids.begin(), ids.begin() + i, ids[i]) != ids.begin() + i) {
            return false;
        }
    }
    return true;
}

} // namespace

MotorFramePlan::MotorFramePlan(std::array<std::uint8_t, 4> rs06MotorIds) noexcept
    : rs06MotorIds_{rs06MotorIds}, motorIdsValid_{validMotorIds(rs06MotorIds)}
{
}

MotorFramePreview MotorFramePlan::compose(
    const RobotSequenceResult& sequence, const MotorCommandPreview& command) const noexcept
{
    MotorFramePreview result{};
    if (sequence.desiredMotorState != DesiredMotorState::DriveAllowed) {
        // StopAll/EnableRs06中のC620はゼロ電流のみ。実際のType4停止と
        // RS06 EnableのTx終端管理はMotorCanLinkに一元化してあるため重複しない。
        const float zeroCurrent[4]{};
        if (!b_c620_current_frame(zeroCurrent, kC620WireMaxA, &result.can2Current)) {
            return {MotorFramePreview::Status::EncodingFailed};
        }
        result.status = MotorFramePreview::Status::ZeroCurrentOnly;
        return result;
    }

    // 状態機械のDriveAllowedフラグだけで非ゼロを許可しない。
    // この段階で作れるのはホストSILの数値プレビューだけ。
    if (!motorIdsValid_) {
        return {MotorFramePreview::Status::InvalidConfiguration};
    }
    if (sequence.state != RobotState::Balancing || sequence.armSession == 0 ||
        !command.fixtureOnly ||
        (command.status != CommandStatus::Ready &&
         command.status != CommandStatus::Saturated)) {
        return {MotorFramePreview::Status::NotDriveAllowed};
    }

    // 全フレームをローカルの候補に作ってから確定する。最後の1軸だけ
    // 符号化できなかった場合に、先頭3軸分を部分出力として出さない。
    for (std::size_t axis = 0; axis < result.can1Type1.size(); ++axis) {
        if (!b_rs06_torque_frame(
                rs06MotorIds_[axis], command.rs06[axis].limitedMotorTorqueNm,
                &result.can1Type1[axis])) {
            return {MotorFramePreview::Status::EncodingFailed};
        }
    }

    const float current[4]{
        command.c620[0].limitedCurrentA,
        command.c620[1].limitedCurrentA,
        0.0f, 0.0f, // 未搭載のC620 ID3,4には必ずゼロ
    };
    if (!b_c620_current_frame(current, kC620WireMaxA, &result.can2Current)) {
        return {MotorFramePreview::Status::EncodingFailed};
    }
    result.can1Count = result.can1Type1.size();
    result.status = MotorFramePreview::Status::Ready;
    return result;
}

} // namespace balancing
