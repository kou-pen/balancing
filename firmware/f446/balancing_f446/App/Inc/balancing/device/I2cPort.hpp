#pragma once
#include "stm32f4xx_hal.h"
#include <array>
#include <cstdint>
#include <optional>

namespace balancing {
enum class I2cState : std::uint8_t { Idle, Pending, Complete, Busy, Error, TimedOut, Unverified };
struct I2cPortConfig { std::uint64_t timeoutUs{0}; bool verified{false}; };
struct I2cCompletion {
    std::uint8_t reg{0}, size{0};
    bool write{false};
    std::array<std::uint8_t, 18> bytes{};
    std::uint64_t startedAtUs{0}, completedAtUs{0};
};
/** @brief I2C1の単一非同期メモリ転送。タイムアウト後にHALバッファを再利用しない。
 * @spec{BOARD-F446-001}
 */
class I2cPort final {
public:
    /** @brief 起動時に検証済みの転送期限を固定する。
     * @param[in,out] handle HAL I2Cハンドル。唯一の転送所有者とする。
     * @param[in] config 期限 [us]。
     * @spec{BOARD-F446-001}
     */
    I2cPort(I2C_HandleTypeDef& handle, I2cPortConfig config) noexcept;
    /** @brief BNO055のレジスタ読み出しを開始。BUSYならHALを呼ばない。
     * @spec{BOARD-F446-001}
     */
    I2cState requestRead(std::uint8_t reg, std::uint8_t length, std::uint64_t nowUs) noexcept;
    /** @brief 1バイトのレジスタ書き込みを開始。
     * @spec{BOARD-F446-001}
     */
    I2cState requestWrite(std::uint8_t reg, std::uint8_t value, std::uint64_t nowUs) noexcept;
    /** @brief 主ループで完了フラグ・実測時刻を確認する。
     * @spec{BOARD-F446-001}
     */
    I2cState poll(std::uint64_t nowUs) noexcept;
    /** @brief 成功転送のみ取り出す。取得前は次転送を開始しない。
     * @spec{BOARD-F446-001}
     */
    std::optional<I2cCompletion> takeCompletion() noexcept;
    /** @brief IRQから対応ハンドルの終端だけを記録する。
     * @spec{BOARD-F446-001}
     */
    void onCompletion(I2C_HandleTypeDef* handle, bool write, bool error) noexcept;
    /** @brief 終端故障後もバッファを所有し続けるため破棄・再構築しない。
     * @spec{BOARD-F446-001}
     */
    I2cState state() const noexcept { return state_; }
private:
    I2cState request(std::uint8_t reg, std::uint8_t length, bool write,
                     std::uint8_t value, std::uint64_t nowUs) noexcept;
    I2C_HandleTypeDef& handle_;
    I2cPortConfig config_;
    I2cState state_;
    I2cCompletion transfer_{};
    volatile std::uint8_t irqResult_{0}; // 0:未完了、1:成功、2:エラー
    volatile bool awaiting_{false};
    volatile bool writing_{false};
};
}
