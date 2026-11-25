#include "balancing/control/filter/ImuAttitudeSource.hpp"
namespace balancing {
ImuAttitudeSource::ImuAttitudeSource(const BAttitudeFilterConfig& config) noexcept
    : failed_{!b_attitude_init(&filter_,&config)}
{}
bool ImuAttitudeSource::accept(const TimedImuSample& s,bool stationaryProven,std::uint64_t nowUs) noexcept
{
    if (failed_ || !s.newMeasurementProven || s.earliestUs>s.completedAtUs ||
        s.completedAtUs>nowUs || nowUs-s.earliestUs>=50000) return false;
    if (!filter_.bias_ready) {
        return b_attitude_add_static_sample(&filter_,&s.value,stationaryProven,true,s.earliestUs,nowUs);
    }
    // 同一報告や古い時刻を再積分しない。新規証明済みの異常測定では姿勢をラッチ無効化。
    if (filter_.latest.valid && s.earliestUs<=filter_.latest.stamp_us) return false;
    BAttitudeEstimate estimate{};
    if (!b_attitude_update(&filter_,&s.value,true,s.earliestUs,nowUs,&estimate)) {
        failed_=true;
        return false;
    }
    return true;
}
ImuAttitude ImuAttitudeSource::attitude(std::uint64_t nowUs) const noexcept
{
    const auto& e=filter_.latest;
    const bool valid=!failed_ && e.valid && e.stamp_us<=nowUs;
    return {e.stamp_us,e.pitch_rad,e.roll_rad,e.pitch_rate_rad_s,e.roll_rate_rad_s,
            e.yaw_rate_rad_s,valid,valid && nowUs-e.stamp_us<50000};
}
}
