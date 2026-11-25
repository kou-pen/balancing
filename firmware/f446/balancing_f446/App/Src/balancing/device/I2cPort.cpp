#include "balancing/device/I2cPort.hpp"
namespace balancing {
namespace { constexpr std::uint16_t bnoAddress = 0x28u << 1u; }
I2cPort::I2cPort(I2C_HandleTypeDef& handle, I2cPortConfig config) noexcept
    : handle_{handle}, config_{config},
      state_{config.verified && config.timeoutUs ? I2cState::Idle : I2cState::Unverified}
{}
I2cState I2cPort::requestRead(std::uint8_t reg, std::uint8_t length, std::uint64_t nowUs) noexcept
{ return request(reg, length, false, 0, nowUs); }
I2cState I2cPort::requestWrite(std::uint8_t reg, std::uint8_t value, std::uint64_t nowUs) noexcept
{ return request(reg, 1, true, value, nowUs); }
I2cState I2cPort::request(std::uint8_t reg, std::uint8_t length, bool write,
                           std::uint8_t value, std::uint64_t nowUs) noexcept
{
    if (state_ != I2cState::Idle) return state_;
    if (length == 0 || length > transfer_.bytes.size()) {
        state_ = I2cState::Error;
        return state_;
    }
    // F4 HALのIT入口にもBUSY待機ループがある。バスが既にBUSYなら呼ばない。
    // 単一マスター前提。検査直後のハード異常によるHAL内待機は実機計測が必要。
    if (HAL_I2C_GetState(&handle_) != HAL_I2C_STATE_READY ||
        (handle_.Instance->SR2 & I2C_SR2_BUSY) != 0u) return I2cState::Busy;
    transfer_ = {};
    transfer_.reg = reg;
    transfer_.size = length;
    transfer_.write = write;
    transfer_.startedAtUs = nowUs;
    transfer_.bytes[0] = value;
    // 通知がHAL呼出し中に到着しても失わないよう、先に受理先を公開。
    const auto mask = __get_PRIMASK();
    __disable_irq();
    irqResult_ = 0;
    writing_ = write;
    awaiting_ = true;
    state_ = I2cState::Pending;
    __set_PRIMASK(mask);
    const auto result = write
        ? HAL_I2C_Mem_Write_IT(&handle_, bnoAddress, reg, I2C_MEMADD_SIZE_8BIT, transfer_.bytes.data(), length)
        : HAL_I2C_Mem_Read_IT(&handle_, bnoAddress, reg, I2C_MEMADD_SIZE_8BIT, transfer_.bytes.data(), length);
    if (result != HAL_OK) {
        awaiting_ = false;
        // HALが転送を開始していない保証を置かず、失敗後はバッファ再利用禁止。
        state_ = I2cState::Error;
    }
    return state_;
}
void I2cPort::onCompletion(I2C_HandleTypeDef* handle, bool write, bool error) noexcept
{
    if (handle == &handle_ && awaiting_ && (error || write == writing_)) {
        // IRQは終端通知のみ。物理量の変換や次転送は主ループに残す。
        if (error || irqResult_ == 0) irqResult_ = error ? 2 : 1;
    }
}
I2cState I2cPort::poll(std::uint64_t nowUs) noexcept
{
    if (state_ != I2cState::Pending) return state_;
    const auto mask = __get_PRIMASK();
    __disable_irq();
    const auto result = irqResult_;
    if (nowUs < transfer_.startedAtUs || nowUs - transfer_.startedAtUs >= config_.timeoutUs) {
        // 完了時刻は主ループ観測の上限。期限後に読んだ完了は採用しない。
        state_ = I2cState::TimedOut;
        awaiting_ = false;
    } else if (result != 0) {
        awaiting_ = false;
        state_ = result == 1 ? I2cState::Complete : I2cState::Error;
        transfer_.completedAtUs = nowUs;
    }
    __set_PRIMASK(mask);
    return state_;
}
std::optional<I2cCompletion> I2cPort::takeCompletion() noexcept
{
    if (state_ != I2cState::Complete) return std::nullopt;
    state_ = I2cState::Idle;
    return transfer_;
}
}
