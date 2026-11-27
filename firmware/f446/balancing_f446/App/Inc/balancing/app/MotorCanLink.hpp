#pragma once

#include "balancing/communication/can/CanRouter.hpp"

#include <array>
#include <optional>

namespace balancing {

class MotorService;
struct ScheduledTxProgress;

/** @brief 両バスの軸別フィードバックと、CAN1のRS06送信を主ループで接続する。
 *  @note 停止枠の登録、メールボックス受付、送信終端の3段階を区別する。
 *        同じCAN1スケジューラをCanRouterのTX照合先にも渡す。
 *  @spec{PROTO-RS06-001}
 *  @spec{BEHAV-001}
 */
class MotorCanLink final : public CanEventSink {
public:
    /** @brief 軸の所有者とCAN1の送信調停を参照で受け取る。
     *  @param[in,out] motors RS06とC620を管理するサービス。リンクより長寿命とする。
     *  @param[in,out] can1Tx CAN1専用のスケジューラ。その停止枠0〜3をリンクが使う。
     *  @spec{PROTO-RS06-001}
     */
    MotorCanLink(MotorService& motors, CanTxScheduler& can1Tx) noexcept;

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

    /** @brief CAN1のRS06報告とCAN2のC620報告を採取時刻付きで配送する。
     *  @spec{PROTO-RS06-001}
     *  @spec{PROTO-C620-001}
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

/** @brief 周期送信対象ごとの受付・終端の累計。物理動作の証明ではない。
 * @spec{BOARD-F446-001}
 */
struct MotorPeriodicTxStatus {
    std::uint64_t queued{},accepted{},sent{},dropped{},failed{};
    std::optional<std::uint64_t> lastSentUs{};
};

/** @brief 2バス5対象の周期送信管理。フレーム内容と運転許可は呼出側が所有。
 * @note CAN1の4対象、CAN2の共通1対象を固定。主ループ専用。
 * @spec{BOARD-F446-001}
 * @spec{BEHAV-001}
 */
class MotorPeriodicTx final {
public:
    /** @brief 既存Schedulerを借用。originは他の発行元と衝突しない固定キー。
     * @spec{BOARD-F446-001}
     */
    MotorPeriodicTx(CanTxScheduler&,CanTxScheduler&,std::uint32_t origin) noexcept;
    MotorPeriodicTx(const MotorPeriodicTx&)=delete;
    MotorPeriodicTx& operator=(const MotorPeriodicTx&)=delete;
    /** @brief CAN1は4ms、CAN2は2msの新世代で置換する。指令の符号化は呼出側。
     * @spec{BOARD-F446-001}
     */
    bool queue(CanBus,const std::array<CanFrame,4>&,std::size_t count,std::uint64_t nowUs) noexcept;
    /** @brief CAN2の全ゼロ電流を周期枠へ登録する。駆動鮮度の証拠にしない。
     * @spec{PROTO-C620-001}
     */
    bool queueZero(std::uint64_t nowUs) noexcept;
    /** @brief 受付・破棄通知を一度処理し、RS06停止の受付を既存リンクへ渡す。
     * @spec{BOARD-F446-001}
     */
    ScheduledTxProgress service(CanBus,MotorCanLink&,std::uint64_t nowUs) noexcept;
    /** @brief 照合済み終端のみを記録。旧世代は送達鮮度を回復しない。
     * @spec{BOARD-F446-001}
     */
    void onTxTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t atUs) noexcept;
    /** @brief 駆動開始時の送達監視。期限は呼出側の明示設定、未設定時は呼ばない。
     * @spec{BEHAV-001}
     */
    void beginActive(std::uint64_t nowUs,std::uint64_t rsTimeoutUs,std::uint64_t wheelTimeoutUs) noexcept;
    /** @brief 最初の駆動指令を登録する前に呼ぶ。以前のゼロ送達を監視から除外。
     * @spec{BEHAV-001}
     */
    void markDriving() noexcept;
    /** @brief 監視中の送達期限切れまたは時計逆行を報告。
     * @spec{BEHAV-001}
     */
    bool deadlineFault(std::uint64_t nowUs) const noexcept;
    /** @brief 未受付を破棄し、受付済みへ中止要求。Schedulerは終端を保持する。
     * @spec{BEHAV-001}
     */
    void withdraw() noexcept;
    /** @brief 記録済み送信異常を一度取り出す。
     * @spec{BEHAV-001}
     */
    bool takeFault() noexcept;
    /** @brief 軸0〜3と車輪共通4の診断参照。主ループでのみ参照する。
     * @spec{BOARD-F446-001}
     */
    const std::array<MotorPeriodicTxStatus,5>& summary() const noexcept { return status_; }
private:
    CanTxScheduler& can1_;
    CanTxScheduler& can2_;
    std::uint32_t origin_;
    std::uint64_t serial_{},generation1_{},generation2_{},rsTimeout_{},wheelTimeout_{};
    std::array<TxToken,5> queued_{},accepted_{},latest_{};
    std::array<std::optional<std::uint64_t>,5> acceptedAt_{},lastSent_{};
    std::array<bool,5> queuedDriving_{},acceptedDriving_{};
    std::array<MotorPeriodicTxStatus,5> status_{};
    std::optional<std::uint64_t> activeSince_{};
    bool driving_{},fault_{};
};

} // namespace balancing
