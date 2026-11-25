#pragma once
#include "balancing/device/Bno055.hpp"
#include "balancing/control/filter/balancing_attitude_filter.h"
#include "balancing/control/estimator/StateEstimator.hpp"
namespace balancing {
/** @brief 既存相補フィルタと静止バイアスの単一所有者。
 * @spec{CTRL-EST-001}
 */
class ImuAttitudeSource final {
public:
    /** @brief 起動時の検証済み数学設定。未検証なら姿勢無効。
     * @spec{CTRL-EST-001}
     */
    explicit ImuAttitudeSource(const BAttitudeFilterConfig& config) noexcept;
    /** @brief 新規性が証明された観測だけを受理する。静止証明は独立入力。
     * @spec{CTRL-EST-001}
     */
    bool accept(const TimedImuSample& sample,bool stationaryProven,std::uint64_t nowUs) noexcept;
    /** @brief 元時刻のまま50ms鮮度を評価する。
     * @spec{CTRL-EST-001}
     */
    ImuAttitude attitude(std::uint64_t nowUs) const noexcept;
    /** @brief 通信・センサ故障後は再初期化まで旧姿勢を使わない。
     * @spec{CTRL-EST-001}
     */
    void invalidate() noexcept { failed_=true; }
private:
    BAttitudeFilter filter_{};
    bool failed_{false};
};
}
