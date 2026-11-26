#pragma once
#include "balancing/communication/can/CanTypes.hpp"
#include <optional>

namespace balancing {
/** @brief 車輪ロータの1回転内角度・RPM・電流生値・温度。
 * @spec{PROTO-C620-001}
 */
struct C620Feedback {
    std::uint8_t motorId{};
    std::uint16_t encoderRaw{};
    std::int16_t rpm{};
    std::int16_t currentRaw{};
    std::uint8_t temperatureC{};
};
class C620Codec final {
public:
    /** @brief ID1〜4の共通電流フレームを構築。全チャネル検証後に確定。
     * @spec{PROTO-C620-001}
     * @warning 20Aは通信表現上限であり実機許容電流ではない。
     */
    static std::optional<CanFrame> encodeCurrent(const std::array<float,4>& currentA,
                                                 float maxAbsCurrentA) noexcept;
    /** @brief 標準ID 0x201〜0x204、DLC8のフィードバックを復号する。
     * @spec{PROTO-C620-001}
     */
    static std::optional<C620Feedback> decodeFeedback(const CanFrame& frame) noexcept;
};
} // namespace balancing
