#pragma once

namespace balancing {

class CanPort;
class CanRouter;

/** @brief 2つのポートを交互に回収し、1バス分の上限で主ループに戻る。
 *  @param[in,out] can1 CAN1ポート。利用しない場合はnullptr。
 *  @param[in,out] can2 CAN2ポート。利用しない場合はnullptr。
 *  @param[in,out] router 回収したイベントを渡す配送先。
 *  @pre 主ループ上でのみ呼ぶ。有効なポートはバスごとに別インスタンス。
 *  @note 一度空だったバスに後から到着したイベントは次回処理する。
 *  @spec{BOARD-F446-001}
 */
void pollCanEvents(CanPort* can1, CanPort* can2, CanRouter& router) noexcept;

} // namespace balancing
