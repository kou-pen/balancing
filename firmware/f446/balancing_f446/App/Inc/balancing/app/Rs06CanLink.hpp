#pragma once

#include "balancing/communication/can/CanRouter.hpp"

#include <array>

namespace balancing {

class MotorService;
struct ScheduledTxProgress;

/** @brief CAN1の転送型とRS06の軸別要求・結果を主ループで接続する。
 *  @note 停止枠の登録、メールボックス受付、送信終端の3段階を区別する。
 *        同じCAN1スケジューラをCanRouterのTX照合先にも渡す。
 *  @spec{PROTO-RS06-001}
 *  @spec{BEHAV-001}
 */
class Rs06CanLink final : public CanEventSink {
public:
    /** @brief 軸の所有者とCAN1の送信調停を参照で受け取る。
     *  @param[in,out] motors RS06を管理するサービス。リンクより長寿命とする。
     *  @param[in,out] can1Tx CAN1専用のスケジューラ。その停止枠0〜3をリンクが使う。
     *  @spec{PROTO-RS06-001}
     */
    Rs06CanLink(MotorService& motors, CanTxScheduler& can1Tx) noexcept;

    /** @brief 現周期の4軸の送信候補を1回ずつ処理する。
     *  @param[in] nowUs CAN採取と共通の単調時計 [us]。
     *  @pre CANイベント回収とMotorService::applySequence()の後に呼ぶ。
     *  @note 停止は軸別の専用枠へ登録する。有効化はその場でCAN1の空き枠を試す。
     *  @spec{PROTO-RS06-001}
     */
    void submitPending(std::uint64_t nowUs) noexcept;

    /** @brief CAN1スケジューラの停止枠からの投入結果を軸へ返す。
     *  @param[in] progress CAN1のCanTxScheduler::service()が返した結果。
     *  @param[in] nowUs service()に渡した受付時刻 [us]。
     *  @pre service()ごとに、次のCANイベント回収より先に呼ぶ。
     *  @note BusyやNotReadyでは登録を維持する。他の所有者の結果は処理しない。
     *  @spec{PROTO-RS06-001}
     */
    void onScheduledTxProgress(const ScheduledTxProgress& progress, std::uint64_t nowUs) noexcept;

    /** @brief CAN1で分類済みのRS06状態報告を採取時刻付きで配送する。
     *  @spec{PROTO-RS06-001}
     */
    void onCanReceive(CanBus bus, CanRxTarget target, const CanFrame& frame,
                      std::uint64_t atUs) noexcept override;

    /** @brief Routerが照合したCAN1のRS06送信終端を軸へ返す。
     *  @spec{PROTO-RS06-001}
     */
    void onCanTransmit(CanBus bus, TxToken token, CanTxOutcome outcome,
                       std::uint64_t atUs) noexcept override;

    /** @brief CAN1のbus-off・復旧・受信欠落をMotorServiceへ渡す。
     *  @spec{BOARD-F446-001}
     */
    void onCanPortStatus(CanBus bus, CanEventKind kind, std::uint64_t atUs) noexcept override;

private:
    MotorService& motors_;
    CanTxScheduler& can1Tx_;
    // 停止枠に登録済みでメールボックスの受付待ちにあるトークン。0は空き。
    std::array<TxToken, 4> queuedStops_{};
};

} // namespace balancing
