#include "balancing/power/PowerMonitor.hpp"

#include <cmath>
#include <limits>

namespace balancing::power {
namespace {

bool validCalibration(const AdcCalibration& c, bool voltage) noexcept {
    // 電圧は正方向、電流は回路の極性を含めて校正可能。ゼロ係数は禁止。
    return c.minCount < c.maxCount && std::isfinite(c.scale) &&
           std::isfinite(c.offset) && (voltage ? c.scale > 0.0F : c.scale != 0.0F);
}

bool convert(const AdcCalibration& c, std::uint16_t raw, float& value) noexcept {
    if (raw < c.minCount || raw > c.maxCount) return false;
    // 中間値のオーバーフローを明示的に検査してからfloatへ戻す。
    const double physical = static_cast<double>(raw) * c.scale + c.offset;
    if (!std::isfinite(physical) || std::fabs(physical) > std::numeric_limits<float>::max()) return false;
    value = static_cast<float>(physical);
    return std::isfinite(value);
}

bool sameCounts(const PowerAdcSample& a, const PowerAdcSample& b) noexcept {
    return a.batteryCount == b.batteryCount && a.driveBusCount == b.driveBusCount &&
           a.currentCount == b.currentCount;
}

} // namespace

bool PowerMonitor::validConfig() const noexcept {
    return config_.verified && validCalibration(config_.battery, true) &&
           validCalibration(config_.driveBus, true) && validCalibration(config_.current, false) &&
           std::isfinite(config_.minBatteryV) && std::isfinite(config_.maxBatteryV) &&
           config_.minBatteryV > 0.0F && config_.maxBatteryV > config_.minBatteryV &&
           std::isfinite(config_.minOnBusV) && std::isfinite(config_.maxBusV) &&
           config_.minOnBusV > 0.0F && config_.maxBusV > config_.minOnBusV &&
           std::isfinite(config_.maxAbsCurrentA) && config_.maxAbsCurrentA > 0.0F &&
           config_.overcurrentHoldUs > 0 && config_.observationMaxAgeUs > 0;
}

PowerObservation PowerMonitor::evaluate(const PowerAdcSample& s, PowerState state,
                                        std::uint64_t nowUs) noexcept {
    // Controllerが所有する状態だけを参照する。Monitor自身はFAULTを保持しない。
    const bool active = state == PowerState::Precharge || state == PowerState::On;
    const bool stateValid = active || state == PowerState::Off || state == PowerState::Fault;
    const bool clockOk = !seenClock_ || nowUs >= lastNowUs_;
    if (clockOk) {
        seenClock_ = true;
        lastNowUs_ = nowUs;
    }
    const bool newSample = !seenSample_ || s.observedAtUs > lastSample_.observedAtUs;
    const bool duplicate = seenSample_ && s.observedAtUs == lastSample_.observedAtUs &&
                           sameCounts(s, lastSample_);
    const bool gap = newSample && seenSample_ &&
        s.observedAtUs - lastSample_.observedAtUs >= config_.observationMaxAgeUs;
    // 観測が途切れた場合や同一時刻の内容が変わった場合に、保持時間を捏造しない。
    // 期限切れ・時刻逆行・換算失敗は値を一部だけ公開せず、Controllerへ無効を返す。
    PowerObservation out{};
    if (!validConfig() || !stateValid || !clockOk || !s.valid ||
        s.observedAtUs > nowUs || nowUs - s.observedAtUs >= config_.observationMaxAgeUs ||
        (!newSample && !duplicate) ||
        !convert(config_.battery, s.batteryCount, out.batteryVoltageV) ||
        !convert(config_.driveBus, s.driveBusCount, out.driveBusVoltageV) ||
        !convert(config_.current, s.currentCount, out.currentA) ||
        out.batteryVoltageV <= 0.0F || out.driveBusVoltageV < 0.0F) {
        currentHolding_ = false;
        return {};
    }
    if (gap) currentHolding_ = false;
    if (newSample) {
        seenSample_ = true;
        lastSample_ = s;
    }
    out.valid = true;
    out.observedAtUs = s.observedAtUs;

    // バス低電圧はONでのみ故障。OFF/PRECHARGE/FAULTでは未充電が正常。
    const bool voltageOk = out.batteryVoltageV >= config_.minBatteryV &&
        out.batteryVoltageV <= config_.maxBatteryV &&
        out.driveBusVoltageV <= config_.maxBusV &&
        (state != PowerState::On || out.driveBusVoltageV >= config_.minOnBusV);

    const bool overcurrent = std::fabs(out.currentA) > config_.maxAbsCurrentA;
    bool currentOk = !overcurrent;
    if (!active || !overcurrent) {
        // OFF/FAULTの過電流中に新規投入・FAULT解除を許さない。
        currentHolding_ = false;
    } else {
        // 同一サンプルでは時間を進めない。新規観測間の継続だけを判定する。
        if (!currentHolding_ && newSample) {
            currentHolding_ = true;
            currentSinceUs_ = s.observedAtUs;
        }
        currentOk = currentHolding_ && s.observedAtUs - currentSinceUs_ < config_.overcurrentHoldUs;
    }
    // 能動状態の欠測は正常扱いにしない。Controllerも独立に観測期限を監視する。
    out.protectionOk = voltageOk && currentOk && !(active && gap);
    return out;
}

} // namespace balancing::power
