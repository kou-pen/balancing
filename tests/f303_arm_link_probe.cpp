// ARMリンク専用のエントリ。実機用FWではなく、GPIO/CAN/ADC/IWDGを一切開始しない。
#include "balancing/power/PowerController.hpp"
#include "balancing/power/PowerMonitor.hpp"
#include "balancing/power/PowerCan.hpp"
#include "balancing/power/BoardIO.hpp"

using namespace balancing::power;

extern "C" void _init() {}
extern "C" void _fini() {}

int main() {
    // 未検証設定のまま、各C++実装をリンク対象に残す。
    // 物理出力に渡す処理はなく、結果はデバッガ確認用のローカル値のみ。
    PowerController controller;
    PowerMonitor monitor;
    PowerCan can;
    BoardIO io;
    const auto obs = monitor.evaluate({}, controller.output().state, 0);
    PowerInputs in{};
    in.observationValid = obs.valid;
    in.protectionOk = obs.protectionOk;
    const auto out = can.step(controller, in, {}, 0);
    const auto board = io.evaluate(out.power, {}, 0);
    volatile bool linkResult = board.mainOn || board.prechargeOn;
    (void)linkResult;
    for (;;) {}
}
