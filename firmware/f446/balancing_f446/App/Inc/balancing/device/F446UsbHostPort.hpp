#pragma once
#include "balancing/communication/usb/DualSenseProtocol.hpp"
#include "stm32f4xx_hal.h"

namespace balancing {
/** @brief 実基板で照合するUSB Host起動条件。既定では起動しない。
 * @note VBUSコールバックは基板固有で、待機を持たない。contextの寿命は起動後全期間。
 * @spec{PROTO-PS5-001}
 * @spec{BOARD-F446-001}
 */
struct F446UsbHostBoardConfig {
    HCD_HandleTypeDef* hcd{};
    void* vbusContext{};
    bool (*setVbus)(void*,bool) noexcept{};
    std::uint64_t enumerationTimeoutUs{},recoveryTimeoutUs{};
    std::uint8_t irqPriority{};
    bool verified{};
};
/** @brief 検証済み設定を使って既存HCDをApplicationへ明示接続。
 * @warning 実機VBUS供給・IRQ優先度・HID報告は別途照合。既定mainからは呼ばない。
 * @spec{PROTO-PS5-001}
 */
bool startF446UsbHost(const F446UsbHostBoardConfig&,const UsbHostConfig&) noexcept;
}
