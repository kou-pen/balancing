#pragma once

#include "balancing/communication/can/CanTypes.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace balancing {
/** @brief CAN IRQと主ループで共有するDWT単調時計。32bit周回を差分で接続。
 * @pre 1周回未満の間隔で読み出すこと。停止したDWTは時刻を進めない。
 * @spec{BOARD-F446-001}
 */
class CanClock final {
public:
    static bool enable() noexcept {
        if (!SystemCoreClock || SystemCoreClock%1000000u ||
            (DWT->CTRL&DWT_CTRL_NOCYCCNT_Msk)) return false;
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        return (DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)!=0;
    }
    explicit CanClock(std::uint32_t cyclesPerUs) noexcept
        : cyclesPerUs_{cyclesPerUs},lastCycle_{DWT->CYCCNT} {}
    std::uint64_t sample() noexcept {
        const auto mask=__get_PRIMASK();
        __disable_irq();
        const auto count=DWT->CYCCNT;
        accumulated_+=static_cast<std::uint32_t>(count-lastCycle_);
        lastCycle_=count;
        const auto cycles=accumulated_;
        __set_PRIMASK(mask);
        return cycles/cyclesPerUs_;
    }
private:
    std::uint32_t cyclesPerUs_,lastCycle_;
    std::uint64_t accumulated_{};
};


/** @brief bxCANの1バス分のFIFO0、送信メールボックス、IRQイベントを所有する。
 *  @spec{BOARD-F446-001}
 */
class CanPort final {
public:
    /** @brief 1バスの送信メールボックスと受信FIFO0の所有権を受け取る。
     *  @param[in,out] handle 初期化済みのCAN1またはCAN2のHALハンドル。
     *  @pre ハンドルと周辺レジスタはこのインスタンスが排他的に使用する。
     *  @note 構築時はレジスタを操作しない。フィルタ設定とHAL_CAN_Startは基板側が行う。
     *  @spec{BOARD-F446-001}
     */
    explicit CanPort(CAN_HandleTypeDef& handle) noexcept;
    CanPort(const CanPort&) = delete;
    CanPort& operator=(const CanPort&) = delete;

    // 1周期で処理する上限。キュー16件とバス・受信欠落の通知3件を含む。
    static constexpr std::size_t kMaxEventsPerPoll = 19;

    /** @brief 空きメールボックスにフレームとトークンを登録する。
     *  @param[in] frame 有効なIDとDLCを持つ送信フレーム。
     *  @param[in] token 同一バスで保留中の要求と重複しない照合キー。
     *  @param[in] nowUs 共通の単調時計の受付時刻 [us]。
     *  @return Acceptedは受付のみ。満杯やバスオフはBusy、無効入力はInvalid、起動前はNotReady。
     *  @spec{BOARD-F446-001}
     */
    CanTxSubmitResult trySend(const CanFrame& frame, TxToken token, std::uint64_t nowUs) noexcept;

    /** @brief 該当トークンの送信中止をハードウェアに要求する。
     *  @param[in] token 対象の送信トークン。
     *  @return 中止要求を書き込んだ場合にtrue。終端結果はpopEventで返す。
     *  @spec{BOARD-F446-001}
     */
    bool requestAbort(TxToken token) noexcept;

    /** @brief 担当バスのIRQから、完了・受信・バス異常を採取する。
     *  @param[in] atUs 共通の単調時計で読んだイベント取得時刻 [us]。
     *  @pre 同じ周辺のフラグをHAL_CAN_IRQHandlerと二重に消費しないこと。
     *  @note FIFO0は1回最大3件。上位の復号・状態遷移は行わない。
     *  @spec{BOARD-F446-001}
     */
    void captureInterrupt(std::uint64_t atUs) noexcept;

    /** @brief メインループへイベントを返す。
     *  @param[out] event 取り出したイベント。空の場合は変更しない。
     *  @return 1件取り出した場合にtrue。
     *  @spec{BOARD-F446-001}
     */
    bool popEvent(CanEvent& event) noexcept;

private:
    static constexpr std::size_t kMailboxCount = 3;
    static constexpr std::size_t kQueueCapacity = kMaxEventsPerPoll - 3;

    struct Mailbox {
        std::optional<TxToken> token{};
        bool terminalQueued{false};
    };

    void observeBus(std::uint32_t txStatus, std::uint64_t atUs) noexcept;
    void queueEvent(const CanEvent& event) noexcept;
    void markRxLost(std::uint64_t atUs) noexcept;
    void captureRx(std::uint64_t atUs) noexcept;

    CAN_HandleTypeDef& handle_;
    std::array<Mailbox, kMailboxCount> mailboxes_{};
    std::array<CanEvent, kQueueCapacity> queue_{};
    std::size_t read_{0};
    std::size_t size_{0};
    bool busOff_{false};
    bool busOffDelivered_{false};
    std::optional<std::uint64_t> pendingBusOff_{};
    std::uint64_t busRecoveredAtUs_{0};
    std::optional<std::uint64_t> rxLostAtUs_{};
};

} // namespace balancing
