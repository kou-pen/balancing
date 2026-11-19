#pragma once

#include "balancing/communication/can/CanTypes.hpp"

namespace balancing {

class CanTxScheduler;

/** @brief 回収したイベントの発生元。標準／拡張IDと独立に識別する。
 *  @spec{BOARD-F446-001}
 */
enum class CanBus : std::uint8_t { Can1, Can2 };

/** @brief フレーム外形から決まる受信先。Unmatchedは診断用に通知する。
 *  @note RS06の軸・主機IDとペイロードの妥当性はCodecと機器サービスが照合する。
 *  @spec{PROTO-RS06-001}
 *  @spec{PROTO-C620-001}
 */
enum class CanRxTarget : std::uint8_t { Unmatched, Rs06Status, C620LeftFeedback, C620RightFeedback };

/** @brief 主ループから受信・送信終端・ポート状態を受け取る境界。
 *  @note 各処理は非ブロッキングで、渡されたフレームの参照を保持しない。
 *  @spec{BOARD-F446-001}
 */
class CanEventSink {
public:
    /** @brief 発生元と分類結果、受信フレームと採取時刻 [us] を受け取る。
     *  @param[in] bus 発生元のバス。
     *  @param[in] target 外形が一致した配送先。未分類フレームは診断へ利用できる。
     *  @param[in] frame 受信フレーム。
     *  @param[in] atUs ポートが採取した時刻 [us]。
     *  @spec{BOARD-F446-001}
     */
    virtual void onCanReceive(CanBus bus, CanRxTarget target, const CanFrame& frame,
                              std::uint64_t atUs) noexcept = 0;

    /** @brief メールボックスで確定した送信終端を受け取る。
     *  @note トークンはバスと組にして照合する。atUsは採取時刻 [us]。
     *  @spec{BOARD-F446-001}
     */
    virtual void onCanTransmit(CanBus bus, TxToken token, CanTxOutcome outcome,
                               std::uint64_t atUs) noexcept = 0;

    /** @brief 受信欠落、bus-off、復旧の採取時刻 [us] と発生元を受け取る。
     *  @pre kindはRxLost / BusOff / BusRecovered。
     *  @spec{BOARD-F446-001}
     */
    virtual void onCanPortStatus(CanBus bus, CanEventKind kind, std::uint64_t atUs) noexcept = 0;

protected:
    // 呼出側が静的に所有する。インターフェース経由では破棄しない。
    ~CanEventSink() = default;
};

/** @brief CANイベントを発生元付きの受信・送信終端・状態通知へ配送する。
 *  @spec{BOARD-F446-001}
 */
class CanRouter final {
public:
    /** @brief ルーターの生存期間を通して有効な配送先とバス別の照合先を受け取る。
     *  @param[in,out] sink 受信・送信終端・バス状態の配送先。
     *  @param[in,out] can1Tx CAN1の照合先。nullptrの場合はポートからのTX終端を直接配送する。
     *  @param[in,out] can2Tx CAN2の照合先。nullptrの場合はポートからのTX終端を直接配送する。
     *  @pre 各照合先には、同じバスの CanPort を割り当てる。
     *  @spec{BOARD-F446-001}
     */
    explicit CanRouter(CanEventSink& sink, CanTxScheduler* can1Tx = nullptr,
                       CanTxScheduler* can2Tx = nullptr) noexcept;

    /** @brief 主ループで採取済みイベントを1件配送する。
     *  @param[in] bus 発生元のバス。
     *  @param[in] event ポートが採取した時刻付きイベント。
     *  @spec{BOARD-F446-001}
     */
    void dispatch(CanBus bus, const CanEvent& event) noexcept;

private:
    CanEventSink& sink_;
    CanTxScheduler* can1Tx_;
    CanTxScheduler* can2Tx_;
};

} // namespace balancing
