#pragma once

#include "balancing/communication/can/CanTypes.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace balancing {

class CanPort;
class CanRouter;

/** @brief 周期送信1件のフレームと終端照合キー。
 *  @spec{BOARD-F446-001}
 */
struct PeriodicTxRequest {
    CanFrame frame{};
    TxToken token{};
};

/** @brief 1バスの同一制御周期に送るフレームの集合。
 *  @note requestsの添字は呼出側で固定する対象キー。RS06 4軸は最大4件、C620は2軸を共通の1件で渡す。
 *  @spec{BOARD-F446-001}
 */
struct PeriodicTxBatch {
    std::uint64_t generation{0};
    std::uint64_t expiresAtUs{0};
    std::array<std::optional<PeriodicTxRequest>, 4> requests{};
};

/** @brief 周期一括更新の結果。ExpiredとStopPendingでは世代番号だけを進める。
 *  @spec{BOARD-F446-001}
 */
enum class PeriodicBatchResult : std::uint8_t { Queued, Expired, Invalid, StaleGeneration, StopPending };

/** @brief 通常イベント1件のフレーム・照合キー・絶対送信期限 [us]。
 *  @note expiresAtUsは未受付の要求を破棄する境界。CanPortが受け付けた後は
 *        期限を過ぎても終端通知を元のtokenで照合する。
 *  @spec{BOARD-F446-001}
 */
struct EventTxRequest {
    CanFrame frame{};
    TxToken token{};
    std::uint64_t expiresAtUs{0};
};

/** @brief 通常イベントをFIFOに登録した結果。
 *  @note Queuedは送信待ちへの登録。メールボックスの受付結果はservice()から受け取る。
 *  @spec{BOARD-F446-001}
 */
enum class EventEnqueueResult : std::uint8_t { Queued, QueueFull, Expired, Invalid };

/** @brief 停止要求を対象別の専用枠へ登録した結果。
 *  @spec{BOARD-F446-001}
 *  @spec{BEHAV-001}
 */
enum class StopEnqueueResult : std::uint8_t { Queued, TargetBusy, Invalid };

/** @brief 停止・通常イベント・周期送信のいずれかを最大1件処理した結果。
 *  @spec{BOARD-F446-001}
 */
enum class ScheduledTxResult : std::uint8_t { Idle, Accepted, Expired, Busy, NotReady, Invalid };

/** @brief 待ち要求の受付または破棄の通知。
 *  @note tokenはAccepted・Expired・Invalidで有効。Busy・NotReady・Idleでは通知対象がない。
 *        Accepted後の結果は、CanRouterがバスとトークンを組にして配送する。
 *  @spec{BOARD-F446-001}
 */
struct ScheduledTxProgress {
    ScheduledTxResult result{ScheduledTxResult::Idle};
    TxToken token{};
};

/** @brief 1バスの送信待ちと送信受付・終端の照合を主ループで扱う。
 *  @note 停止専用枠、通常FIFO、最大4件の周期送信待ちを別々に保持し、
 *        メールボックスが受け付けた最大3件を終端まで追跡する。
 *        IRQはCanPortにイベントを記録し、このクラスの状態は主ループだけが更新する。
 *  @spec{BOARD-F446-001}
 */
class CanTxScheduler final {
public:
    /** @brief 1バスで独立に追跡できる停止対象数。CAN1のRS06全4軸を収容する。
     *  @spec{BOARD-F446-001}
     */
    static constexpr std::size_t kStopTargetCount = 4;

    /** @brief 送信要求を投入するバスのポートを受け取る。
     *  @param[in,out] port このインスタンスの存続中、有効な専用ポート。
     *  @pre 主ループで、このバスの送信要求は当インスタンスを経由する。
     *  @spec{BOARD-F446-001}
     */
    explicit CanTxScheduler(CanPort& port) noexcept;
    /** @brief 通常イベントFIFO用の固定領域を指定する。
     *  @param[in,out] port このインスタンスが使用するポート。
     *  @param[in,out] storage スケジューラより長寿命で、他と共有しない待ち領域。
     *  @note Nは未受付イベント数の上限。0件なら通常イベントを受け付けない。
     *        FIFOの領域はスケジューラを通じて更新し、受付済みのイベントは3枠の照合表で管理する。
     *  @spec{BOARD-F446-001}
     */
    template<std::size_t N>
    CanTxScheduler(CanPort& port, std::array<EventTxRequest, N>& storage) noexcept
        : port_{port}, events_{storage.data()}, eventCapacity_{N}
    {
    }

    CanTxScheduler(const CanTxScheduler&) = delete;
    CanTxScheduler& operator=(const CanTxScheduler&) = delete;

    /** @brief 直ちに空きメールボックスへ投入し、受け付けたトークンを記録する。
     *  @param[in] frame 有効なIDとDLCを持つフレーム。
     *  @param[in] token 送信元と非ゼロの番号。同じバス・送信元で終端済みの番号は再利用しない。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。
     *  @return Acceptedは受付のみ。枠不足または受付済みトークンの再投入はBusy。
     *  @note 未完了の停止要求または待機中・受付済みの通常イベントがあればBusy。
     *  @spec{BOARD-F446-001}
     */
    CanTxSubmitResult trySubmit(const CanFrame& frame, TxToken token, std::uint64_t nowUs) noexcept;

    /** @brief 背景CAN通信を最大1メールボックスだけ受付。停止・通常・周期待ちが優先。
     * @note 受付済み背景通信の終端は周期送信を待たせない。物理バス遅延の保証ではない。
     * @spec{BOARD-F446-001}
     */
    CanTxSubmitResult trySubmitBackground(const CanFrame& frame, TxToken token,
                                         std::uint64_t nowUs) noexcept;
    /** @brief 新しい周期のフレーム一式で、このバスの周期送信待ちを置き換える。
     *  @param[in] batch 非ゼロで単調増加する世代番号、絶対期限 [us]、対象別フレーム。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。
     *  @return Queuedは新世代を登録。Expiredは新世代を記録して待ちを全破棄。
     *          StopPendingは停止未完了のためフレームを破棄し、新世代だけを記録する。
     *          InvalidとStaleGenerationでは待ちと世代を保持する。
     *  @note 未指定の対象も旧世代から消える。受付済みのフレームは終端まで別に管理する。
     *  @spec{BOARD-F446-001}
     */
    PeriodicBatchResult replacePeriodic(const PeriodicTxBatch& batch, std::uint64_t nowUs) noexcept;

    /** @brief 未受付周期を破棄し、受付済み周期に中止要求を出す。
     * @pre 周期requestsの添字と停止対象番号は同じ軸に割り当てる。
     * @note 対応軸の停止と新周期は終端確認を待つ。他軸の停止は独立。中止要求は取消成功の証明ではない。
     * @spec{BEHAV-001}
     */
    void invalidatePeriodic() noexcept;

    /** @brief 期限内の通常イベントをFIFO末尾に追加する。
     *  @param[in] request フレーム、固有トークン、絶対送信期限 [us]。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。
     *  @return Queuedで末尾にコピーする。QueueFullでは既存の順序と内容を保持する。
     *          Expiredは nowUs >= expiresAtUs、Invalidはフレームまたはトークンの不正を表す。
     *  @note 同じバスで待機中または受付済みのトークンは重複登録できない。
     *  @spec{BOARD-F446-001}
     */
    EventEnqueueResult enqueueEvent(const EventTxRequest& request, std::uint64_t nowUs) noexcept;

    /** @brief 送信待ちの停止要求を通常FIFOと独立した対象別の枠へ保存する。
     *  @param[in] target このバス上の対象番号 [0, kStopTargetCount)。対応付けは要求元が所有する。
     *  @param[in] frame 要求元が符号化した停止フレーム。
     *  @param[in] token 送信元と非ゼロの番号。送信終端後の再試行には新しい番号を使う。
     *  @return Queuedは登録済み。TargetBusyは同一対象の送信待ちまたは終端待ち。
     *          Invalidは範囲外、フレーム／トークンの不正、またはトークンの重複。
     *  @note 登録時に未受付の周期フレームを破棄する。他の停止対象や通常FIFOは保持する。
     *        対応するTx成功で枠を解放する。Tx失敗後は同じ対象を新しいトークンで再登録できる。
     *  @spec{BOARD-F446-001}
     *  @spec{BEHAV-001}
     */
    StopEnqueueResult enqueueStop(std::size_t target, const CanFrame& frame, TxToken token) noexcept;

    /** @brief 停止、通常イベント、周期送信の順に、期限切れの通知または投入を最大1件行う。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。期限ちょうどから破棄する。
     *  @return Busy・NotReadyでは待ちを保持する。Acceptedはメールボックス受付を表す。
     *  @pre 主ループでCAN終端を回収し、周期の新規要求があれば置換した後に呼ぶ。
     *  @note 停止要求は対象番号順で待機分を投入する。未完了の停止があれば通常・周期は投入しない。
     *        通常イベントは先頭だけを参照し、受付済みの通常イベントの終端まで次の投入を待つ。
     *        すでに受付済みのフレームは停止登録後も終端まで照合を続ける。
     *  @spec{BOARD-F446-001}
     */
    ScheduledTxProgress service(std::uint64_t nowUs) noexcept;

private:
    friend class CanRouter;

    // 終端の配送より先に枠を空ける。配送先が次の要求を投入しても、完了済みの枠が残らない。
    bool consumeTerminal(TxToken token, CanTxOutcome outcome, std::uint64_t atUs) noexcept;

    // CanPortが受け付けた要求だけを保持する。終端回収後に同じメールボックスを再利用できる。
    struct AcceptedTx {
        TxToken token{};
        std::uint64_t acceptedAtUs{0};
        std::uint64_t periodicGeneration{0};
        bool normalEvent{false};
        bool background{false};
        bool cancellationPending{false};
        std::size_t periodicTarget{0};
    };

    enum class StopState : std::uint8_t { Idle, Pending, InFlight, RetryRequired };

    struct StopSlot {
        CanFrame frame{};
        TxToken token{};
        StopState state{StopState::Idle};
    };

    CanTxSubmitResult submit(const CanFrame& frame, AcceptedTx accepted) noexcept;
    ScheduledTxProgress submitScheduled(const CanFrame& frame, AcceptedTx accepted) noexcept;
    ScheduledTxProgress serviceStop(std::uint64_t nowUs) noexcept;
    ScheduledTxProgress servicePeriodic(std::uint64_t nowUs) noexcept;
    bool isAccepted(TxToken token) const noexcept;
    bool isKnownToken(TxToken token) const noexcept;
    bool hasStop() const noexcept;
    bool hasEventInFlight() const noexcept;
    bool hasQueuedEvent(TxToken token) const noexcept;
    bool hasQueuedPeriodic(TxToken token) const noexcept;
    void popEvent() noexcept;

    CanPort& port_;
    std::array<AcceptedTx, 3> accepted_{};
    std::array<StopSlot, kStopTargetCount> stops_{};
    PeriodicTxBatch periodic_{};
    // 呼出側の固定領域をリングとして利用する。有効範囲は先頭からeventCount_件。
    EventTxRequest* events_{nullptr};
    std::size_t eventCapacity_{0};
    std::size_t eventHead_{0};
    std::size_t eventCount_{0};
};

} // namespace balancing
