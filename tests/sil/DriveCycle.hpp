#pragma once
// この経路はホストのbxCANレジスタ模型専用。ARMバイナリに組み込まない。
#if defined(__arm__) || defined(__thumb__)
#error "The SIL drive implementation must never be built for ARM firmware."
#endif
#include "balancing/app/ControlCycle.hpp"
#include "balancing/app/MotorCanLink.hpp"
#include "balancing/communication/can/MotorFramePlan.hpp"
#include "balancing/communication/can/CanTxScheduler.hpp"

namespace balancing {

/** @brief SILの独立した送信完了期限。実機閾値ではない。
 * @spec{PROTO-RS06-001}
 * @spec{PROTO-C620-001}
 */
struct DriveSilConfig {
    std::uint64_t rs06TxTimeoutUs{}, c620TxTimeoutUs{};
};
struct DriveSilResult {
    ControlTickResult control{};
    std::optional<MotorFramePreview> frames{};
    bool outputInhibited{true};
};

/** @brief ホストだけで一周期の状態決定とモータ送信を統合する。
 * @warning 実機ビルドには存在しない。fixtureOnlyは維持する。
 * @spec{BEHAV-001}
 * @spec{BOARD-F446-001}
 */
class DriveCycle final {
public:
    /** @brief 各単一所有者とバス別送信調停を参照する。全て本クラスより長寿命。
     * @spec{BOARD-F446-001}
     */
    DriveCycle(ControlCycle&,RobotSequence&,MotorService&,MotorCanLink&,
               CanTxScheduler&,CanTxScheduler&,DriveSilConfig) noexcept;
    /** @brief 回収済み観測と操作を一周期で処理。ARM結果はMotorServiceから得る。
     * @spec{BEHAV-001}
     */
    DriveSilResult step(ControlTickInput input) noexcept;
    /** @brief 待機中の送信を有限回処理。新しい周期指令は生成しない。
     * @spec{BOARD-F446-001}
     */
    void poll(std::uint64_t nowUs) noexcept;
    /** @brief Routerが照合した終端だけを受ける。別バス・旧番号は無視。
     * @spec{BOARD-F446-001}
     */
    void onTxTerminal(CanBus,TxToken,CanTxOutcome,std::uint64_t atUs) noexcept;
private:
    void withdraw() noexcept;
    bool queue(CanBus,const MotorFramePreview&,std::uint64_t nowUs) noexcept;
    void service(CanBus,std::uint64_t nowUs) noexcept;
    FaultFlags txFaults(std::uint64_t nowUs) const noexcept;
    ControlCycle& control_;
    RobotSequence& sequence_;
    MotorService& motors_;
    MotorCanLink& link_;
    MotorPeriodicTx tx_;
    MotorFramePlan frames_;
    DriveSilConfig config_;
    std::optional<std::uint64_t> activeSince_{}, lastRsDue_{}, lastTick_{};
    FaultFlags pending_{FaultFlags::None};
    bool active_{false};
    bool configured_{false};
};
}
