#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace balancing {

/** @brief bxCAN 32bit Listの1バンク分の完全一致ワード。
 * @spec{BOARD-F446-001}
 */
struct Can2FilterWord {
    std::uint32_t first{};
    std::uint32_t second{};
};

/** @brief CAN2用の固定フィルタ構成。bank14からcount個を有効にする。
 * @spec{BOARD-F446-001}
 */
struct Can2FilterPlan {
    std::array<Can2FilterWord,7> entries{};
    std::size_t count{};
};

/** @brief C620用1バンクと、必要に応じ自作受信ID用6バンクを生成。
 * @note HAL非依存。自作通信の受信有効化は呼出側の責務。
 * @spec{BOARD-F446-001}
 * @spec{PROTO-PWR-001}
 * @spec{PROTO-REMOTE-001}
 */
Can2FilterPlan makeCan2FilterPlan(bool includeProtocol) noexcept;

} // namespace balancing
