#pragma once

#include "balancing/app/RobotSequence.hpp"
#include "balancing/device/C620Motor.hpp"
#include "balancing/device/Rs06Motor.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace balancing {

struct Rs06BusConfig {
    // 順序は左脚A・B、右脚A・B。実機の宛先・主機IDは呼出側で検証する。
    std::array<std::uint8_t, 4> motorIds{};
    std::uint8_t hostId{0};
};

/** @brief 同じ周期時刻で評価したRS06 4軸とC620 2軸のフィードバック。
 *  @note sampledAtUsは評価時刻 [us]。各軸の受信時刻は個別に保持する。
 *  @spec{PROTO-RS06-001}
 *  @spec{PROTO-C620-001}
 */
struct MotorFeedbackSnapshot {
    std::uint64_t sampledAtUs;
    std::array<Rs06AxisFeedback, 4> rs06;  // 左脚A・B、右脚A・B
    std::array<C620AxisFeedback, 2> c620;  // 左車輪、右車輪
};

class MotorService final {
public:
    /** @brief RS06を4軸分所有し、バス設定を初期化する。
     *  @param[in] config 同一バス上の重複しないモータ番号と主機ID。
     *  @param[in] wheelContinuity 左右独立の検証済み取得間隔。未指定では位置無効。
     *  @note 重複またはゼロのモータ番号では送信を禁止する。
     *  @spec{PROTO-RS06-001}
     */
    explicit MotorService(const Rs06BusConfig& config,
                          const std::array<C620ContinuityConfig, 2>& wheelContinuity = {}) noexcept;

    /** @brief RobotSequenceが確定した当該周期の操作要求を反映する。
     *  @param[in] sequence 駆動状態とARMセッション番号。
     *  @param[in] nowUs 同一の単調時計で記録した時刻 [us]。
     *  @spec{BEHAV-001}
     */
    void applySequence(const RobotSequenceResult& sequence, std::uint64_t nowUs) noexcept;

    /** @brief CAN1で次に送信できる未受付の軸別候補を返す。
     *  @return 左脚A・B、右脚A・Bの順の候補。空きスロットだけに投入する。
     *  @note トランスポートは受付結果をその場で反映し、終端イベントに同じトークンを使う。
     *  @spec{PROTO-RS06-001}
     */
    std::array<std::optional<Rs06TxRequest>, 4> pendingRs06Requests() const noexcept;

    /** @brief 軸別送信の受付結果または完了・失敗イベントを照合する。
     *  @param[in] token 送信候補に紐づく軸と番号。
     *  @param[in] result CANポートの結果。
     *  @param[in] atUs CANポートで記録した単調時刻 [us]。
     *  @spec{PROTO-RS06-001}
     */
    void onRs06TxResult(Rs06TxToken token, Rs06TxResult result, std::uint64_t atUs) noexcept;

    /** @brief CAN1の受信フレームを復号して該当軸へ配送する。
     *  @param[in] frame RouterでCAN1のデータフレームと判定した内容。
     *  @param[in] receivedAtUs CAN受信時の単調時刻 [us]。処理時刻とは区別する。
     *  @spec{PROTO-RS06-001}
     */
    void onRs06Frame(const BMotorCanFrame& frame, std::uint64_t receivedAtUs) noexcept;

    /** @brief CAN2のC620報告を復号し、ID1・ID2に対応する車輪へ配送する。
     *  @param[in] frame CAN2で受信した状態報告。
     *  @param[in] receivedAtUs CANで採取した単調時刻 [us]。
     *  @spec{PROTO-C620-001}
     */
    void onC620Frame(const BMotorCanFrame& frame, std::uint64_t receivedAtUs) noexcept;

    /** @brief 6軸の観測値と鮮度を単一の制御周期のコピーとして返す。
     *  @param[in] nowUs 制御周期で固定した単調時刻 [us]。
     *  @pre 主ループで当該周期のCANイベントを回収してから呼ぶ。
     *  @spec{PROTO-RS06-001}
     *  @spec{PROTO-C620-001}
     */
    MotorFeedbackSnapshot feedback(std::uint64_t nowUs) const noexcept;

    /** @brief CAN1のバスオフ状態を更新する。
     *  @param[in] busOff CANポートが報告した現在の状態。
     *  @note 受付済み送信の解放は各トークンの終端イベントで反映する。
     *  @spec{BOARD-F446-001}
     */
    void setCan1BusOff(bool busOff) noexcept;

    /** @brief CAN1の受信欠落を次回のtakeObservedFaults()まで保持する。
     *  @spec{BOARD-F446-001}
     */
    void reportCan1RxLost() noexcept;

    /** @brief CAN2のbus-off/復旧を通知する。復旧だけでは位置を復元しない。
     *  @param[in] busOff 現在のバス状態。
     *  @spec{BOARD-F446-001}
     */
    void setCan2BusOff(bool busOff) noexcept;

    /** @brief CAN2の受信喪失で両車輪の連続性を無効にする。
     *  @spec{CTRL-EST-001}
     */
    void reportCan2RxLost() noexcept;

    /** @brief 両車輪の原点世代を明示的に更新する。
     *  @return bus-off中または世代更新失敗ならfalse。
     *  @pre DISARMED、旧受信キュー排除済み、StateEstimator::reset()と同期。
     *  @warning 本番呼出はまだ接続しない。
     *  @spec{CTRL-EST-001}
     */
    bool resetWheelReference() noexcept;

    /** @brief 軸別の安全判断を順序固定のコピーで返す。
     *  @param[in] nowUs 制御周期の単調時刻 [us]。
     *  @spec{PROTO-RS06-001}
     */
    std::array<Rs06MotorStatus, 4> rs06Status(std::uint64_t nowUs) const noexcept;

    /** @brief 現在のARMセッションに対応する4軸の集約結果を返す。
     *  @param[in] nowUs 制御周期の単調時刻 [us]。
     *  @spec{BEHAV-001}
     */
    Rs06ArmFeedback rs06ArmFeedback(std::uint64_t nowUs) const noexcept;

    /** @brief CAN1・RS06で観測した明示故障を返す。
     *  @param[in] nowUs 制御周期の単調時刻 [us]。
     *  @note 取得周期までに復旧した異常も一度返す。ARM固有の有効化失敗は別途返す。
     *  @spec{BEHAV-001}
     */
    FaultFlags takeObservedFaults(std::uint64_t nowUs) noexcept;

    /** @brief 受信専用段階の実故障を消費する。未設定軸は故障でなく観測未成立。
     *  @spec{BOARD-F446-001}
     */
    FaultFlags takeObservationFaults(std::uint64_t nowUs) noexcept;

    /** @brief RS06の構築時ID検査結果。物理配線の証明ではない。
     *  @spec{PROTO-RS06-001}
     */
    bool hasRs06Configuration() const noexcept { return configured_; }
    /** @brief 起動時固定の軸割当コピー。送信側と受信側のIDを二重設定しない。
     * @spec{PROTO-RS06-001}
     */
    Rs06BusConfig rs06BusConfig() const noexcept { return config_; }

private:
    Rs06Motor* findMotor(std::uint8_t id) noexcept;

    Rs06BusConfig config_;
    std::array<Rs06Motor, 4> rs06_;
    std::array<C620Motor, 2> c620_;
    bool configured_;
    bool can1BusOff_{false};
    bool can2BusOff_{false};
    FaultFlags pendingFaults_{FaultFlags::None};
    bool armActive_{false};
    ArmSessionId armSession_{0};
};

} // namespace balancing
