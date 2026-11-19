#pragma once

#include <array>
#include <cstdint>

namespace balancing {

/** @brief Classic CANのデータフレーム。標準11 bitと拡張29 bitを扱う。
 *  @note dlcは0〜8。受信データの有効範囲はdataの先頭dlcバイト。
 *  @spec{BOARD-F446-001}
 */
struct CanFrame {
    std::uint32_t id{0};
    std::array<std::uint8_t, 8> data{};
    std::uint8_t dlc{0};
    bool extended{false};

    /** @brief 送信可能なデータフレームのID範囲とDLCを検証する。
     *  @spec{BOARD-F446-001}
     */
    constexpr bool validForTransmit() const noexcept
    {
        const auto idLimit = extended ? 0x1fffffffu : 0x7ffu;
        return id <= idLimit && dlc <= data.size();
    }
};

/** @brief 上位が発行する、1バス内の送信照合キー。
 *  @note originは発行元の不透明なキー。serialは非ゼロとし、保留中の組を再利用しない。
 *  @spec{BOARD-F446-001}
 */
struct TxToken {
    std::uint64_t serial{0};
    std::uint32_t origin{0};

    /** @brief 同じバス内で発行元とシリアルの両方が一致するかを判定する。
     *  @spec{BOARD-F446-001}
     */
    constexpr bool operator==(const TxToken& other) const noexcept
    {
        return serial == other.serial && origin == other.origin;
    }
};

/** @brief メールボックスへの投入結果。Acceptedはバス上の送信完了を意味しない。
 *  @spec{BOARD-F446-001}
 */
enum class CanTxSubmitResult : std::uint8_t { Accepted, Busy, Invalid, NotReady };

/** @brief 受付済みフレームのハードウェア終端結果。
 *  @note NotTransmittedは中止を含む、TXOK・TERR・ALSTのいずれもない終端。
 *  @spec{BOARD-F446-001}
 */
enum class CanTxOutcome : std::uint8_t { Sent, ArbitrationLost, TransmitError, NotTransmitted };

/** @brief 1バスの受信、送信終端、受信欠落、バス状態の通知種別。
 *  @spec{BOARD-F446-001}
 */
enum class CanEventKind : std::uint8_t { Rx, Tx, BusOff, BusRecovered, RxLost };

/** @brief IRQと主ループの間で渡す、バス単位の時刻付きイベント。
 *  @note atUsは単調時計の採取時刻 [us]。frameはRx、token・outcome・mailboxはTxで有効。
 *  @spec{BOARD-F446-001}
 */
struct CanEvent {
    CanEventKind kind{CanEventKind::Rx};
    std::uint64_t atUs{0};
    CanFrame frame{};
    TxToken token{};
    CanTxOutcome outcome{CanTxOutcome::NotTransmitted};
    std::uint8_t mailbox{0};
};

} // namespace balancing
