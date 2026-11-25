#include "balancing/communication/can/Can2FilterPlan.hpp"

namespace balancing {
namespace {

// bxCANの32bit照合形式。IDE=bit2、RTR=bit1。
// 標準データはSTID、拡張データは29bit全体を配置する。RTRは常に0。
constexpr std::uint32_t standardWord(std::uint32_t id) noexcept { return id << 21u; }
constexpr std::uint32_t extendedWord(std::uint32_t id) noexcept { return (id << 3u) | 4u; }

// F446受信対象の12個のみ。送信IDや他機器間通信を含めない。
// DLCと内容は後段CanRouter/Codecで検査する。
constexpr std::array<std::uint32_t,12> protocolIds{
    0x0840bc03u,0x08408404u, // 電源状態・F446宛応答
    0x0004bc01u,0x0004fc01u, // F303/Pi非常停止
    0x0880c401u,0x0880c402u, // 運動・ARM要求
    0x0480c403u,0x0880c404u, // DISARM・異常解除
    0x0880c405u,0x0cc0c480u, // 操作元・購読
    0x0cc0c481u,0x0d00c401u  // 購読解除・設定読出し
};
}

Can2FilterPlan makeCan2FilterPlan(bool includeProtocol) noexcept
{
    Can2FilterPlan plan{};
    // 2個の標準IDを1バンクに収める。未搭載軸と0x700は通さない。
    plan.entries[0]={standardWord(0x201u),standardWord(0x202u)};
    plan.count=1;
    if (includeProtocol) {
        // List ModeではFR1/FR2ともIDであり、マスクではない。
        for (std::size_t i=0;i<protocolIds.size();i+=2) {
            plan.entries[plan.count++]={extendedWord(protocolIds[i]),extendedWord(protocolIds[i+1])};
        }
    }
    return plan;
}
} // namespace balancing
