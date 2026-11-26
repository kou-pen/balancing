#pragma once
#include <cstdint>

namespace balancing {
/** @brief TIM7イベント番号と主ループ開始時刻。番号は32bit周回を許容。
 * @spec{BOARD-F446-001}
 */
struct CycleTicket {
    std::uint32_t eventSequence{};
    std::uint64_t startedAtUs{};
};
/** @brief 完了した一周期の証跡。実行終了時刻は開始時刻以上。
 * @spec{BOARD-F446-001}
 */
struct CycleCompletion {
    CycleTicket ticket{};
    std::uint64_t completedAtUs{};
};
}
