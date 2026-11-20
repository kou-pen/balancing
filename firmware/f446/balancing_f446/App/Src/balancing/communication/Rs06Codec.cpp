#include "balancing/communication/Rs06Codec.hpp"

namespace balancing {

std::optional<Rs06StateReport> Rs06Codec::decodeFeedback(const BMotorCanFrame& frame) noexcept
{
    // ID/DLCとペイロードの既存検証を共有し、ここで報告種別と予約モードを区別する。
    BRS06Feedback decoded{};
    if (!b_rs06_parse_feedback(&frame, &decoded) ||
        decoded.mode > static_cast<std::uint8_t>(Rs06Mode::Running)) {
        return std::nullopt;
    }
    return Rs06StateReport{decoded.motor_id, decoded.host_id,
                           static_cast<Rs06Report>(frame.id >> 24),
                           static_cast<Rs06Mode>(decoded.mode), decoded.fault_bits,
                           decoded.angle_rad, decoded.velocity_rad_s,
                           decoded.torque_nm, decoded.temperature_c};
}

std::optional<BMotorCanFrame> Rs06Codec::encodeControl(
    Rs06Command command, std::uint8_t motorId, std::uint8_t hostId) noexcept
{
    // 既存の符号化と共通化し、Type 4の通常停止は全バイトをゼロにする。
    BMotorCanFrame frame{};
    if (!b_rs06_control_frame(motorId, hostId, command == Rs06Command::Enable, &frame)) {
        return std::nullopt;
    }
    return frame;
}

} // namespace balancing
