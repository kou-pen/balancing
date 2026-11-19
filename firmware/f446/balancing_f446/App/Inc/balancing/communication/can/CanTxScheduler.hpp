#pragma once

#include "balancing/communication/can/CanTypes.hpp"

#include <array>
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

/** @brief 周期一括更新の結果。Expiredの世代では旧世代の待ち要求も破棄済み。
 *  @spec{BOARD-F446-001}
 */
enum class PeriodicBatchResult : std::uint8_t { Queued, Expired, Invalid, StaleGeneration };

/** @brief 周期送信を1件処理した結果。
 *  @spec{BOARD-F446-001}
 */
enum class PeriodicTxResult : std::uint8_t { Idle, Accepted, Expired, Busy, NotReady, Invalid };

/** @brief 周期送信の受付または破棄の通知。
 *  @note tokenはAccepted・Expired・Invalidで有効。終端はCanRouterが別途配送する。
 *  @spec{BOARD-F446-001}
 */
struct PeriodicTxProgress {
    PeriodicTxResult result{PeriodicTxResult::Idle};
    TxToken token{};
};

/** @brief 1バスの周期送信待ちと送信受付・終端の照合を主ループで扱う。
 *  @note 最大4件の周期送信待ちと、最大3件の受付済み要求をバスごとに保持する。
 *  @spec{BOARD-F446-001}
 */
class CanTxScheduler final {
public:
    /** @brief 送信要求を投入するバスのポートを受け取る。
     *  @param[in,out] port このインスタンスの存続中、有効な専用ポート。
     *  @pre 主ループで、このバスの送信要求は当インスタンスを経由する。
     *  @spec{BOARD-F446-001}
     */
    explicit CanTxScheduler(CanPort& port) noexcept;
    CanTxScheduler(const CanTxScheduler&) = delete;
    CanTxScheduler& operator=(const CanTxScheduler&) = delete;

    /** @brief 直ちに空きメールボックスへ投入し、受け付けたトークンを記録する。
     *  @param[in] frame 有効なIDとDLCを持つフレーム。
     *  @param[in] token 送信元と非ゼロの番号。同じバス・送信元で終端済みの番号は再利用しない。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。
     *  @return Acceptedは受付のみ。枠不足または受付済みトークンの再投入はBusy。
     *  @note Busy・Invalid・NotReadyでは新しいトークンを登録しない。
     *  @spec{BOARD-F446-001}
     */
    CanTxSubmitResult trySubmit(const CanFrame& frame, TxToken token, std::uint64_t nowUs) noexcept;

    /** @brief 新しい周期のフレーム一式で、このバスの周期送信待ちを置き換える。
     *  @param[in] batch 非ゼロで単調増加する世代番号、絶対期限 [us]、対象別フレーム。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。
     *  @return Queuedは新世代を登録。Expiredは新世代を記録して待ちを全破棄。
     *          InvalidとStaleGenerationでは待ちと世代を保持する。
     *  @note 未指定の対象も旧世代から消える。受付済みのフレームは終端まで別に管理する。
     *  @spec{BOARD-F446-001}
     */
    PeriodicBatchResult replacePeriodic(const PeriodicTxBatch& batch, std::uint64_t nowUs) noexcept;

    /** @brief 期限切れを1件破棄するか、期限内の周期フレームを最大1件投入する。
     *  @param[in] nowUs CanPortと共通の単調時計の時刻 [us]。nowUs >= expiresAtUsで期限切れ。
     *  @return Busy・NotReadyでは待ちを保持する。返されるAcceptedは受付のみ。
     *  @pre 主ループでCAN終端を回収し、その周期に新規要求があれば置換した後に呼ぶ。
     *  @spec{BOARD-F446-001}
     */
    PeriodicTxProgress servicePeriodic(std::uint64_t nowUs) noexcept;

private:
    friend class CanRouter;

    // 終端の配送より先に枠を空ける。配送先が次の要求を投入しても、完了済みの枠が残らない。
    bool consumeTerminal(TxToken token, std::uint64_t atUs) noexcept;

    CanTxSubmitResult submit(const CanFrame& frame, TxToken token, std::uint64_t nowUs,
                             std::uint64_t periodicGeneration) noexcept;
    bool isAccepted(TxToken token) const noexcept;

    struct AcceptedTx {
        TxToken token{};
        std::uint64_t acceptedAtUs{0};
        std::uint64_t periodicGeneration{0};
    };

    CanPort& port_;
    std::array<AcceptedTx, 3> accepted_{};
    PeriodicTxBatch periodic_{};
};

} // namespace balancing
