#pragma once

#include "balancing/app/RobotSequence.hpp"
#include "balancing/communication/balancing_motor_can.h"
#include "balancing/control/controller/MotorCommand.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace balancing {

/** @brief SILでのみ評価するCAN1 Type1×4・CAN2 0x200×1のフレーム集合。
 *  @note 停止用Type4は既存MotorCanLinkが所有し、この集合に入れない。
 *  @warning fixtureOnly=true固定。HAL/送信スケジューラへの登録APIはない。
 *  @spec{PROTO-RS06-001}
 *  @spec{PROTO-C620-001}
 */
struct MotorFramePreview {
    enum class Status : std::uint8_t {
        Ready,
        ZeroCurrentOnly,
        InvalidConfiguration,
        NotDriveAllowed,
        EncodingFailed,
    };

    Status status{Status::NotDriveAllowed};
    std::array<BMotorCanFrame, 4> can1Type1{};
    std::size_t can1Count{0};
    BMotorCanFrame can2Current{};
    bool fixtureOnly{true};
};

/** @brief 6軸の数学的な指令プレビューを既存C11 CAN Codecで符号化する。
 *  @note CAN1送信間隔4ms、CAN2 2msなどの周期調停は呼出し元が別に所有する。
 *  @warning 生成結果を実機CANへ接続しない。実機校正・駆動許可の証明ではない。
 *  @spec{PROTO-RS06-001}
 *  @spec{PROTO-C620-001}
 */
class MotorFramePlan final {
public:
    /** @brief RS06の4軸宛先IDを起動時に固定する。
     *  @param[in] rs06MotorIds 左脚A/B、右脚A/B。全て非ゼロで重複なし。
     *  @spec{PROTO-RS06-001}
     */
    explicit MotorFramePlan(std::array<std::uint8_t, 4> rs06MotorIds) noexcept;

    /** @brief 状態機械の最終要求を適用して、送信を伴わないフレーム値を計算する。
     *  @param[in] sequence その周期のRobotSequence結果。
     *  @param[in] command MotorCommandで変換・制限済みの数学プレビュー。
     *  @return BALANCING/DriveAllowed時のみType1×4と0x200を生成。
     *          非駆動時はC620ゼロ電流の0x200のみ。無効な要求は全フレーム空。
     *  @warning 非ゼロフレームの生成を実モータの送信許可と解釈してはならない。
     *  @spec{BEHAV-001}
     *  @spec{PROTO-C620-001}
     */
    MotorFramePreview compose(const RobotSequenceResult& sequence,
                              const MotorCommandPreview& command) const noexcept;

private:
    std::array<std::uint8_t, 4> rs06MotorIds_{};
    bool motorIdsValid_{false};
};

} // namespace balancing
