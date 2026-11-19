#include "balancing/app/CanEventPoll.hpp"

#include "balancing/communication/can/CanPort.hpp"
#include "balancing/communication/can/CanRouter.hpp"

namespace balancing {

void pollCanEvents(CanPort* can1, CanPort* can2, CanRouter& router) noexcept
{
    CanEvent event{};
    // 一方に受信が集中しても、もう一方の送信終端・バス状態へ同じ周回で進む。
    // コールバック中にIRQが新しいイベントを積んでも、次の制御処理へ有限回で戻る。
    for (std::size_t i = 0; i < CanPort::kMaxEventsPerPoll && (can1 || can2); ++i) {
        if (can1) {
            if (can1->popEvent(event)) {
                router.dispatch(CanBus::Can1, event);
            } else {
                can1 = nullptr;
            }
        }
        if (can2) {
            if (can2->popEvent(event)) {
                router.dispatch(CanBus::Can2, event);
            } else {
                can2 = nullptr;
            }
        }
    }
}

} // namespace balancing
