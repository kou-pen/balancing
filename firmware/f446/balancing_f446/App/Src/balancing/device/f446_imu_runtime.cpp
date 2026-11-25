#include "balancing_f446_imu_bridge.h"
#include "balancing/device/ImuObservation.hpp"
#include "balancing/device/MotorObservation.hpp"

namespace {
struct ImuRuntime {
    balancing::I2cPort port;
    balancing::Bno055 sensor;
    balancing::ImuAttitudeSource attitude;
    ImuRuntime(I2C_HandleTypeDef& h,const balancing::ImuRuntimeConfig& config) noexcept
        : port{h,config.port},sensor{port,config.sensor},attitude{config.filter} {}
};
std::optional<ImuRuntime> runtime;
}
bool balancing::startImuObservation(I2C_HandleTypeDef* handle,const ImuRuntimeConfig& config) noexcept
{
    if (runtime || handle==nullptr || handle->Instance!=I2C1 || !observationClockUs()) return false;
    // 既定未検証ならI2Cレジスタに触れない。物理運用の期限を推測しない。
    if (!config.port.verified || !config.port.timeoutUs || !config.sensor.maxRequestWaitUs) return false;
    runtime.emplace(*handle,config);
    // IRQは同一I2CハンドルのHALイベントのみ。優先度は既存CAN/TIMより低くする。
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6, 0);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    return true;
}
extern "C" bool balancing_f446_imu_start(I2C_HandleTypeDef* handle)
{
    return balancing::startImuObservation(handle,{});
}
extern "C" void balancing_f446_imu_poll(void)
{
    if (!runtime) return;
    const auto now=balancing::observationClockUs();
    if (!now) { runtime->attitude.invalidate(); return; }
    runtime->sensor.poll(*now);
    if (runtime->sensor.state()==balancing::Bno055State::Failed) runtime->attitude.invalidate();
    if (const auto sample=runtime->sensor.takeSample()) {
        // 実機静止を証明する上位入力は未接続。自動でバイアスを学習させない。
        runtime->attitude.accept(*sample,false,*now);
    }
}
balancing::ImuAttitude balancing::imuAttitude(std::uint64_t nowUs) noexcept
{
    return runtime ? runtime->attitude.attitude(nowUs) : ImuAttitude{};
}
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* h)
{ if (runtime) runtime->port.onCompletion(h,false,false); }
extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* h)
{ if (runtime) runtime->port.onCompletion(h,true,false); }
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* h)
{ if (runtime) runtime->port.onCompletion(h,false,true); }
