#pragma once

#include "balancing/app/MotorService.hpp"
#include "balancing/device/F446Can1Port.hpp"

namespace balancing {

/** @brief 主ループでCAN1イベントを軸の状態に反映する。
 *  @param[in,out] port CAN1ポート。
 *  @param[in,out] motors RS06四軸のサービス。
 *  @return この周期に追加してRobotSequenceへ渡すポート異常。
 *  @note バスオフと軸の異常はMotorService::takeObservedFaultsでも取得する。
 *  @spec{BOARD-F446-001}
 *  @spec{BEHAV-001}
 */
FaultFlags drainRs06Can1(F446Can1Port& port, MotorService& motors) noexcept;

/** @brief 安全判断を反映した後の候補をCAN1へ投入して受付結果を渡す。
 *  @param[in,out] port CAN1ポート。
 *  @param[in,out] motors RobotSequenceの当該周期の要求を反映したサービス。
 *  @param[in] nowUs 周期の単調時刻 [us]。
 *  @note メールボックス満杯の候補は次の周期で再評価する。
 *  @spec{PROTO-RS06-001}
 */
void submitRs06Can1(F446Can1Port& port, MotorService& motors, std::uint64_t nowUs) noexcept;

} // namespace balancing
