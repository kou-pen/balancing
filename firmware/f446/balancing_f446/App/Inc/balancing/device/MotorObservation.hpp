#pragma once

#include "balancing/app/MotorService.hpp"

namespace balancing {

/** @brief 通常起動で停止指令だけを送る明示設定。
 * @note verifiedはRS06宛先/主機、CAN2 ID1/2とID3/4未使用、バス配線の照合を表す。
 * @spec{BOARD-F446-001}
 */
struct StopOutputConfig {
    bool enabled{},verified{};
};

/** @brief 起動時固定の受信軸設定。未検証ではRS06軸を割り当てない。
 *  @spec{BOARD-F446-001}
 */
struct MotorObservationConfig {
    Rs06BusConfig rs06{};
    bool rs06MappingVerified{false};
    std::array<C620ContinuityConfig, 2> wheelContinuity{};
    StopOutputConfig stopOutput{};
};

/** @brief 両バス回収後の同一時刻の観測コピー。駆動許可ではない。
 *  @spec{CTRL-EST-001}
 */
struct MotorObservationSnapshot {
    MotorFeedbackSnapshot motors{};
    std::uint64_t generation{0}; // 回収順。同じµsの別回収を識別する
    bool can1BusOff{false};
    bool can2BusOff{false};
    FaultFlags observedFaults{FaultFlags::None};
    bool rs06MappingVerified{false};
};

/** @brief 最後に完了したCAN回収のコピー。読み出しは故障を消費しない。
 *  @pre 主ループのみ。
 *  @spec{BOARD-F446-001}
 */
std::optional<MotorObservationSnapshot> motorObservation() noexcept;

/** @brief 上位周期が消費した故障を明示的に解除する。
 *  @param[in] generation 消費した最新コピーの回収番号。古いコピーでは解除しない。
 *  @pre 主ループで最新コピーを使った周期処理直後のみ。
 *  @spec{BOARD-F446-001}
 */
bool acknowledgeMotorObservation(std::uint64_t generation) noexcept;

/** @brief CANのIRQと同一の単調時計。CAN起動前は空。
 *  @spec{BOARD-F446-001}
 */
std::optional<std::uint64_t> observationClockUs() noexcept;
/** @brief Applicationの原点再設定専用。両輪更新成功時の採取境界を返す。
 * @pre DISARMED・周期外・推定器世代検査済み。成功後は必ず推定器も同期する。
 * @spec{CTRL-EST-001}
 */
std::optional<std::uint64_t> resetMotorWheelReference(std::uint64_t expectedGeneration) noexcept;

} // namespace balancing
