#pragma once

#include "balancing/device/Rs06Motor.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace balancing {

enum class Can1EventKind : std::uint8_t { Rx, Tx, BusOff, BusRecovered, RxLost };

struct Can1Event {
    Can1EventKind kind{Can1EventKind::Rx};
    std::uint64_t atUs{0};
    BMotorCanFrame frame{};
    Rs06TxToken token{};
    Rs06TxResult txResult{Rs06TxResult::Busy};
    std::uint8_t mailbox{0};
};

class F446Can1Port final {
public:
    /** @brief CAN1の送信メールボックスと受信FIFO0の所有権を受け取る。
     *  @param[in,out] handle 初期化済みのCAN1 HALハンドル。
     *  @pre CAN1の送受信をこのインスタンスだけが管理する。
     *  @note フィルタ設定とHAL_CAN_Startは呼出側が行う。
     *  @spec{BOARD-F446-001}
     */
    explicit F446Can1Port(CAN_HandleTypeDef& handle) noexcept;
    F446Can1Port(const F446Can1Port&) = delete;
    F446Can1Port& operator=(const F446Can1Port&) = delete;

    // 1周期で処理する上限。キュー16件とバス・受信欠落の通知3件を含む。
    static constexpr std::size_t maxEventsPerCycle = 19;

    /** @brief 特定の空きメールボックスに指令とトークンを登録する。
     *  @param[in] request 復号前のCANフレームと送信トークン。
     *  @param[in] nowUs 共通の単調時計の受付時刻 [us]。
     *  @return Acceptedは受付のみ。満杯やバスオフはBusy、無効な要求はFailed。
     *  @spec{PROTO-RS06-001}
     */
    Rs06TxResult trySend(const Rs06TxRequest& request, std::uint64_t nowUs) noexcept;

    /** @brief 該当トークンの送信中止をハードウェアに要求する。
     *  @param[in] token 対象の送信トークン。
     *  @return 中止要求を書き込んだ場合にtrue。終端結果はpopEventで返す。
     *  @spec{BOARD-F446-001}
     */
    bool requestAbort(Rs06TxToken token) noexcept;

    /** @brief CAN1のIRQまたはポーリングから、完了・受信・故障を確保する。
     *  @param[in] atUs 共通の単調時計で読んだイベント取得時刻 [us]。
     *  @pre CAN1のHAL_CAN_IRQHandlerと同時にフラグを消費しないこと。
     *  @note FIFO0は1回最大3件。上位の復号・状態遷移は行わない。
     *  @spec{BOARD-F446-001}
     */
    void captureInterrupt(std::uint64_t atUs) noexcept;

    /** @brief メインループへイベントを返す。
     *  @param[out] event 取り出したイベント。空の場合は変更しない。
     *  @return 1件取り出した場合にtrue。
     *  @spec{BOARD-F446-001}
     */
    bool popEvent(Can1Event& event) noexcept;

private:
    static constexpr std::size_t mailboxCount = 3;
    static constexpr std::size_t queueCapacity = maxEventsPerCycle - 3;

    struct Mailbox {
        std::optional<Rs06TxToken> token{};
        bool terminalQueued{false};
    };

    void observeBus(std::uint32_t txStatus, std::uint64_t atUs) noexcept;
    void queueEvent(const Can1Event& event) noexcept;
    void markRxLost(std::uint64_t atUs) noexcept;
    void captureRx(std::uint64_t atUs) noexcept;

    CAN_HandleTypeDef& handle_;
    std::array<Mailbox, mailboxCount> mailboxes_{};
    std::array<Can1Event, queueCapacity> queue_{};
    std::size_t read_{0};
    std::size_t size_{0};
    bool busOff_{false};
    bool busOffDelivered_{false};
    std::optional<std::uint64_t> pendingBusOff_{};
    std::uint64_t busRecoveredAtUs_{0};
    std::optional<std::uint64_t> rxLostAtUs_{};
};

} // namespace balancing
