#pragma once

#include "balancing/communication/can/CanTypes.hpp"

#include <array>

namespace balancing {

class CanPort;
class CanRouter;

/** @brief 1バスの送信受付と終端を、主ループで照合する。
 *  @note 受付済みの要求を最大3件保持する。BUSY時にフレームは保持しない。
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

private:
    friend class CanRouter;

    // 終端の配送より先に枠を空ける。配送先が次の要求を投入しても、完了済みの枠が残らない。
    bool consumeTerminal(TxToken token, std::uint64_t atUs) noexcept;

    struct AcceptedTx {
        TxToken token{};
        std::uint64_t acceptedAtUs{0};
    };

    CanPort& port_;
    std::array<AcceptedTx, 3> accepted_{};
};

} // namespace balancing
