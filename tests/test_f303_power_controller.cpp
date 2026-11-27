#include "balancing/power/PowerController.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

using namespace balancing::power;

#define CHECK(expression) do { if (!(expression)) { \
    std::cerr << "failed line " << __LINE__ << ": " << #expression << '\n'; \
    std::exit(1); } } while (false)

// 人工の時間・電圧値。実機の閾値、負荷、MOSFET定格ではない。
constexpr PowerConfig config{true, 1.0F, 2000, 10000, 3000, 3000};
PowerInputs sample(std::uint64_t time, float bus = 47.0F,
                   PowerCommand command = PowerCommand::None) {
    PowerInputs in{};
    in.command = command;
    in.estopOk = true;
    in.observationValid = true;
    in.protectionOk = true;
    in.batteryVoltageV = 48.0F;
    in.driveBusVoltageV = bus;
    in.observedAtUs = time;
    return in;
}
void expect(PowerOutput o, PowerState state, bool pre, bool main) {
    CHECK(o.state == state);
    CHECK(o.prechargeRequested == pre);
    CHECK(o.mainRequested == main);
}
void charging(PowerController& c) {
    expect(c.output(), PowerState::Off, false, false);
    expect(c.step(sample(0, 0.0F), 0), PowerState::Precharge, true, false);
}
void overlap(PowerController& c) {
    charging(c);
    expect(c.step(sample(1000), 1000), PowerState::Precharge, true, false);
    expect(c.step(sample(2000), 2000), PowerState::Precharge, true, false);
    expect(c.step(sample(3000), 3000), PowerState::Precharge, true, true);
}
void on(PowerController& c) {
    overlap(c);
    auto in = sample(4000);
    in.mainConfirmed = true;
    expect(c.step(in, 4000), PowerState::On, false, true);
}

int main() {
    // 設定未検証・時間設定矛盾・非有限閾値は明示ONでも投入不可。
    for (auto bad : {PowerConfig{}, PowerConfig{true, 1.0F, 2000, 2000, 3000, 3000},
                     PowerConfig{true, std::numeric_limits<float>::quiet_NaN(), 2000, 10000, 3000, 3000}}) {
        PowerController c(bad, ResetCause::BatteryConnection);
        auto out = c.step(sample(0, 0.0F, PowerCommand::On), 0);
        expect(out, PowerState::Off, false, false);
        CHECK(out.commandResult == PowerCommandResult::SafetyRejected);
    }
    {
        PowerController c(config); // 未判別/IWDGリセットでは自動起動しない。
        expect(c.output(), PowerState::Fault, false, false);
        expect(c.step(sample(0), 0), PowerState::Fault, false, false);
        auto old = sample(0, 47.0F, PowerCommand::On);
        CHECK(c.step(old, 1000).commandResult == PowerCommandResult::SafetyRejected);
        auto out = c.step(sample(2000, 47.0F, PowerCommand::On), 2000);
        expect(out, PowerState::Precharge, true, false);
        CHECK(out.commandResult == PowerCommandResult::Accepted);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        expect(c.step({}, 0), PowerState::Off, false, false);
        expect(c.step(sample(1000, 0.0F), 1000), PowerState::Precharge, true, false);
        // 閾値ちょうど、成立時間ちょうど。途中で電圧差が外れたら再計測。
        expect(c.step(sample(2000), 2000), PowerState::Precharge, true, false);
        expect(c.step(sample(3000, 46.9F), 3000), PowerState::Precharge, true, false);
        expect(c.step(sample(4000), 4000), PowerState::Precharge, true, false);
        expect(c.step(sample(5000), 5000), PowerState::Precharge, true, false);
        expect(c.step(sample(6000), 6000), PowerState::Precharge, true, true);
        // 同じ観測の主電源確認値を書き換えてもONには進まない。
        auto duplicate = sample(6000); duplicate.mainConfirmed = true;
        expect(c.step(duplicate, 7000), PowerState::Precharge, true, true);
        auto fresh = sample(8000); fresh.mainConfirmed = true;
        expect(c.step(fresh, 8000), PowerState::On, false, true);
        CHECK(c.step(sample(9000, 47.0F, PowerCommand::On), 9000).commandResult ==
              PowerCommandResult::AlreadyInState);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        charging(c);
        // 同じサンプルを再評価しても連続成立にはならない。
        expect(c.step(sample(1000), 1000), PowerState::Precharge, true, false);
        expect(c.step(sample(1000), 2000), PowerState::Precharge, true, false);
        expect(c.step(sample(1000), 3000), PowerState::Precharge, true, false);
        expect(c.step(sample(1000), 4000), PowerState::Fault, false, false);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        charging(c);
        for (std::uint64_t t = 1000; t < 8000; t += 1000)
            expect(c.step(sample(t, 0.0F), t), PowerState::Precharge, true, false);
        expect(c.step(sample(8000), 8000), PowerState::Precharge, true, false);
        expect(c.step(sample(9000), 9000), PowerState::Precharge, true, false);
        // 完了条件と期限が同時なら期限優先。
        expect(c.step(sample(10000), 10000), PowerState::Fault, false, false);
        expect(c.step(sample(11000), 11000), PowerState::Fault, false, false);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        overlap(c);
        expect(c.step(sample(4000), 4000), PowerState::Precharge, true, true);
        expect(c.step(sample(5000), 5000), PowerState::Precharge, true, true);
        auto in = sample(6000); in.mainConfirmed = true;
        expect(c.step(in, 6000), PowerState::Fault, false, false);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        overlap(c);
        auto in = sample(4000, 46.0F); in.mainConfirmed = true;
        expect(c.step(in, 4000), PowerState::Fault, false, false);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        on(c);
        auto stop = sample(5000); stop.estopOk = false;
        expect(c.step(stop, 5000), PowerState::Off, false, false);
        expect(c.step(sample(6000), 6000), PowerState::Precharge, true, false);
        auto off = c.step(sample(7000, 47.0F, PowerCommand::Off), 7000);
        expect(off, PowerState::Off, false, false);
        CHECK(off.commandResult == PowerCommandResult::Accepted);
        stop.observedAtUs = 8000;
        expect(c.step(stop, 8000), PowerState::Off, false, false);
        expect(c.step(sample(9000), 9000), PowerState::Off, false, false);
        expect(c.step(sample(10000, 47.0F, PowerCommand::On), 10000),
               PowerState::Precharge, true, false);
    }
    // 各能動段階でCAN ESTOP、ハード異常、ソフト異常、観測失効、通常OFFを検証。
    for (int stage = 0; stage < 3; ++stage) {
        for (int fault = 0; fault < 7; ++fault) {
            PowerController c(config, ResetCause::BatteryConnection);
            std::uint64_t t = 1000;
            if (stage == 0) charging(c);
            else if (stage == 1) { overlap(c); t = 4000; }
            else { on(c); t = 5000; }
            auto in = sample(t);
            if (fault == 0) in.command = PowerCommand::EmergencyStop;
            if (fault == 1) { in.hardwareFault = true; in.command = PowerCommand::On; }
            if (fault == 2) in.protectionOk = false;
            if (fault == 3) in.observationValid = false;
            if (fault == 4) in.batteryVoltageV = std::numeric_limits<float>::infinity();
            if (fault == 5) in.command = PowerCommand::Off;
            if (fault == 6) in.observedAtUs = t + 1;
            auto out = c.step(in, t);
            expect(out, fault == 5 ? PowerState::Off : PowerState::Fault, false, false);
            expect(c.step(sample(t+1000), t+1000),
                   fault == 5 ? PowerState::Off : PowerState::Fault, false, false);
            auto off = c.step(sample(t+2000, 47.0F, PowerCommand::Off), t+2000);
            expect(off, fault == 5 ? PowerState::Off : PowerState::Fault, false, false);
            auto blocked = sample(t+3000, 47.0F, PowerCommand::On);
            blocked.estopOk = false;
            CHECK(c.step(blocked, t+3000).commandResult == PowerCommandResult::SafetyRejected);
            expect(c.step(sample(t+4000), t+4000),
                   fault == 5 ? PowerState::Off : PowerState::Fault, false, false);
            expect(c.step(sample(t+5000, 47.0F, PowerCommand::On), t+5000),
                   PowerState::Precharge, true, false);
        }
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        on(c);
        // サンプルの時刻が新しくても、観測が途中で途絶したことを隠せない。
        expect(c.step(sample(7000), 7000), PowerState::Fault, false, false);
    }
    {
        PowerController c(config, ResetCause::BatteryConnection);
        on(c);
        expect(c.step(sample(3000), 3000), PowerState::Fault, false, false);
    }
    std::cout << "F303 PowerController logical boundary tests PASS\n";
}
