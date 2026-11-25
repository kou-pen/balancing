#include "balancing/communication/can/ProtocolId.hpp"

namespace balancing::protocol {
namespace {

constexpr bool nodeValid(NodeId node) noexcept
{
    return node == NodeId::F446 || node == NodeId::F303 || node == NodeId::Pi;
}

constexpr bool typeValid(DataType type) noexcept
{
    switch (type) {
    case DataType::Common:
    case DataType::Power:
    case DataType::Remote:
    case DataType::Diagnostics:
    case DataType::Parameters:
        return true;
    }
    return false;
}

constexpr bool reserved(std::uint32_t id) noexcept
{
    // DJIの標準IDとの数値一致に加え、調停に使う上位11bitも避ける。
    return isDjiReserved(id) || isDjiReserved(id >> 18);
}

constexpr bool fieldsValid(ProtocolId f) noexcept
{
    // マスクして切り詰めると誤った機器番号や優先度が有効IDになるため拒否する。
    return f.priority <= 3 && typeValid(f.type) && nodeValid(f.source)
        && (nodeValid(f.destination) || f.destination == NodeId::Broadcast)
        && f.reg <= 0x3ffu;
}

constexpr std::uint32_t pack(ProtocolId f) noexcept
{
    return (std::uint32_t{f.priority} << 26)
        | (static_cast<std::uint32_t>(f.type) << 18)
        | (static_cast<std::uint32_t>(f.source) << 14)
        | (static_cast<std::uint32_t>(f.destination) << 10)
        | f.reg;
}

// 仕様書の送信元・宛先・優先度・DLCまで含む許可リスト。
// Register IDだけでは方向や優先度の違う未定義フレームを誤受理する。
constexpr std::array<MessageSpec, 35> messages{{
    {{0, DataType::Common, NodeId::F446, NodeId::Broadcast, 0x001}, 0},
    {{0, DataType::Common, NodeId::F303, NodeId::Broadcast, 0x001}, 0},
    {{0, DataType::Common, NodeId::Pi, NodeId::Broadcast, 0x001}, 0},
    {{2, DataType::Power, NodeId::F446, NodeId::F303, 0x001}, 2},
    {{1, DataType::Power, NodeId::F446, NodeId::F303, 0x002}, 2},
    {{2, DataType::Power, NodeId::Pi, NodeId::F303, 0x001}, 2},
    {{1, DataType::Power, NodeId::Pi, NodeId::F303, 0x002}, 2},
    {{2, DataType::Power, NodeId::F303, NodeId::Broadcast, 0x003}, 8},
    {{2, DataType::Power, NodeId::F303, NodeId::F446, 0x004}, 8},
    {{2, DataType::Power, NodeId::F303, NodeId::Pi, 0x004}, 8},
    {{2, DataType::Remote, NodeId::Pi, NodeId::F446, 0x001}, 8},
    {{2, DataType::Remote, NodeId::Pi, NodeId::F446, 0x002}, 2},
    {{1, DataType::Remote, NodeId::Pi, NodeId::F446, 0x003}, 2},
    {{2, DataType::Remote, NodeId::Pi, NodeId::F446, 0x004}, 2},
    {{2, DataType::Remote, NodeId::Pi, NodeId::F446, 0x005}, 3},
    {{2, DataType::Remote, NodeId::F446, NodeId::Pi, 0x006}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x001}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x002}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x003}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x004}, 4},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x005}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x006}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x007}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x008}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x009}, 8},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x00a}, 8},
    {{1, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x020}, 8},
    {{1, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x021}, 8},
    {{1, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x022}, 8},
    {{3, DataType::Diagnostics, NodeId::Pi, NodeId::F446, 0x080}, 8},
    {{3, DataType::Diagnostics, NodeId::Pi, NodeId::F446, 0x081}, 4},
    {{3, DataType::Diagnostics, NodeId::F446, NodeId::Pi, 0x082}, 8},
    {{3, DataType::Parameters, NodeId::Pi, NodeId::F446, 0x001}, 4},
    {{3, DataType::Parameters, NodeId::F446, NodeId::Pi, 0x002}, 8},
    {{3, DataType::Parameters, NodeId::F446, NodeId::Pi, 0x003}, 6},
}};

constexpr bool validRegistry() noexcept
{
    for (std::size_t i=0; i<messages.size(); ++i) {
        const auto& m=messages[i];
        if (!fieldsValid(m.fields) || reserved(pack(m.fields)) || m.dlc > 8) return false;
        for (std::size_t j=0; j<i; ++j)
            if (pack(messages[j].fields)==pack(m.fields)) return false;
    }
    return true;
}
static_assert(validRegistry(), "Invalid or duplicate CAN2 protocol allocation");

} // namespace

bool isReservedExtendedId(std::uint32_t id) noexcept
{
    return reserved(id);
}

std::optional<std::uint32_t> encodeId(ProtocolId fields) noexcept
{
    if (!fieldsValid(fields)) return std::nullopt;
    const auto id=pack(fields);
    if (reserved(id)) return std::nullopt;
    return id;
}

std::optional<ProtocolId> decodeId(std::uint32_t id) noexcept
{
    // 上位bitを無視して分解しない。予約領域も通さない。
    if (id > 0x1fffffffu || reserved(id)) return std::nullopt;
    const ProtocolId fields{
        static_cast<std::uint8_t>((id >> 26) & 7u),
        static_cast<DataType>((id >> 18) & 0xffu),
        static_cast<NodeId>((id >> 14) & 0xfu),
        static_cast<NodeId>((id >> 10) & 0xfu),
        static_cast<std::uint16_t>(id & 0x3ffu)
    };
    if (!fieldsValid(fields)) return std::nullopt;
    return fields;
}

std::optional<MessageSpec> findMessage(std::uint32_t id) noexcept
{
    // 完全一致のみ。将来の拡張用Register IDを暗黙に許可しない。
    for (const auto& m : messages)
        if (pack(m.fields) == id) return m;
    return std::nullopt;
}

bool matchesProtocolFrame(const CanFrame& frame) noexcept
{
    if (!frame.extended || !frame.validForTransmit()) return false;
    const auto spec=findMessage(frame.id);
    return spec && frame.dlc == spec->dlc;
}

} // namespace balancing::protocol
