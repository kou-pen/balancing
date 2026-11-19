#pragma once

#include "balancing/communication/Rs06Codec.hpp"

#include <cstdint>
#include <optional>

namespace balancing {

struct Rs06TxToken {
    std::uint8_t motorId;
    std::uint64_t serial;
};

enum class Rs06TxResult : std::uint8_t {
    Busy, Accepted, Completed, RetryableFailure, Failed,
};

struct Rs06TxRequest {
    Rs06TxToken token;
    Rs06Command command;
    BMotorCanFrame frame;
};

enum class Rs06EnableStatus : std::uint8_t { Pending, Confirmed, Failed };

struct Rs06MotorStatus {
    Rs06EnableStatus enable;
    bool stopPending;
    bool disableConfirmed;
    bool feedbackFresh;
    bool motorFault;
    bool stopTxFailed;
};

class Rs06Motor final {
public:
    /** @brief 軸のCAN宛先を保持し、停止要求未完了で開始する。
     *  @param[in] motorId モータ番号。
     *  @param[in] hostId 応答宛先となる主機ID。
     *  @spec{PROTO-RS06-001}
     */
    Rs06Motor(std::uint8_t motorId, std::uint8_t hostId) noexcept;

    /** @brief 前回の停止送信後、新しい有効化要求を作る。
     *  @pre StopのTxが完了し、呼出側で新しいARMセッションを受理していること。
     *  @spec{BEHAV-001}
     */
    void requestEnable() noexcept;

    /** @brief 有効化を取り消し、この軸の停止送信を要求する。
     *  @note 受付済み送信の終端を待つ間も停止要求を保持する。
     *  @spec{PROTO-RS06-001}
     */
    void requestStop() noexcept;

    /** @brief 未受付の要求フレームと結果照合用のトークンを取得する。
     *  @return 受付待ちの送信候補。送信中または要求なしならnullopt。
     *  @spec{PROTO-RS06-001}
     */
    std::optional<Rs06TxRequest> nextRequest() const noexcept;

    /** @brief 現在のトークンに対応する送信受付・完了・失敗を反映する。
     *  @param[in] token 送信候補に紐づく軸と番号。
     *  @param[in] result Busyは未受付、Acceptedは受付、CompletedはCAN Tx完了。
     *  @param[in] atUs 送信側で記録した同一の単調時計の時刻 [us]。
     *  @note Accepted後の失敗通知はメールボックスが解放された終端イベントを表す。
     *  @spec{PROTO-RS06-001}
     */
    void onTxResult(Rs06TxToken token, Rs06TxResult result, std::uint64_t atUs) noexcept;

    /** @brief 対象のモータIDと主機IDの新しい状態報告を保存する。
     *  @param[in] feedback Codecで形式検証済みの報告。
     *  @param[in] receivedAtUs CAN受信時の単調時計時刻 [us]。
     *  @return この軸の最新報告として採用した場合にtrue。
     *  @spec{PROTO-RS06-001}
     */
    bool onFeedback(const Rs06StateReport& feedback, std::uint64_t receivedAtUs) noexcept;

    /** @brief 軸の有効化・停止・鮮度の判定結果を返す。
     *  @param[in] nowUs 制御周期の単調時計時刻 [us]。
     *  @return 停止のTx完了とモータの無効化報告は個別の判定。
     *  @spec{PROTO-RS06-001}
     */
    Rs06MotorStatus status(std::uint64_t nowUs) const noexcept;

private:
    enum class Phase : std::uint8_t { StopQueued, StopSent, EnableQueued, EnableSent, Failed };
    struct AcceptedTx {
        std::uint64_t serial;
        std::uint64_t acceptedAtUs;
    };
    struct StampedReport {
        Rs06StateReport report;
        std::uint64_t receivedAtUs;
    };

    void advanceToken() noexcept;
    void completeCurrent(Rs06TxResult result, std::uint64_t atUs) noexcept;

    std::uint8_t motorId_;
    std::uint8_t hostId_;
    Phase phase_{Phase::StopQueued};
    std::uint64_t serial_{1};
    std::uint64_t txCompletedAtUs_{0};
    std::optional<AcceptedTx> accepted_{};
    std::optional<StampedReport> latest_{};
    std::optional<StampedReport> latestStatus_{};
    bool stopTxFailed_{false};
};

} // namespace balancing
