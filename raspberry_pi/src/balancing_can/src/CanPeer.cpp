#include "balancing/pi/CanPeer.hpp"
#include <type_traits>

namespace balancing::pi {

std::optional<Incoming> decodeIncoming(const CanFrame& frame, bool remoteRequest,
                                       bool errorFrame) noexcept {
    using namespace protocol;
    // 同一バスのC620標準フレーム、RTR、エラーは上位操作のデータではない。
    // 各Codecの固定ID照合以前に、データフレームとして有効な外形を要求する。
    if (remoteRequest || errorFrame || !frame.extended || !frame.validForTransmit())
        return std::nullopt;
    if (auto p = decodePowerStatus(frame)) return Incoming{*p};
    if (auto p = decodePowerResult(frame)) {
        // F446宛ての応答をPi自身の要求完了として扱わない。
        if (p->destination == NodeId::Pi) return Incoming{*p};
        return std::nullopt;
    }
    if (auto p = decodeRemoteResult(frame)) return Incoming{*p};
    if (auto p = decodeEmergencyStop(frame)) {
        // SocketCANの反射設定に依存せず、自機の送信反射は受信イベントにしない。
        if (*p != NodeId::Pi) return Incoming{*p};
        return std::nullopt;
    }
    if (auto p = decodeDiagnostic(frame)) {
        // 既存Codecの型をそのまま使用し、独自のバイト配置を複製しない。
        // 設定読出し・詳細購読・周期診断の拡張は保留。要求を状態通知と誤認しない。
        return std::visit([](const auto& v) -> std::optional<Incoming> {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_constructible_v<Incoming,T>) return Incoming{v};
            else return std::nullopt;
        }, *p);
    }
    return std::nullopt;
}

std::optional<CanFrame> powerRequest(protocol::PowerRequestKind kind,
                                     std::uint16_t requestId) noexcept {
    // 送信元を固定し、UIやROSメッセージから他機器のIDを選べないようにする。
    return protocol::encodePowerRequest({protocol::NodeId::Pi,kind,requestId});
}

std::optional<CanFrame> emergencyStop() noexcept {
    return protocol::encodeEmergencyStop(protocol::NodeId::Pi);
}

} // namespace balancing::pi
