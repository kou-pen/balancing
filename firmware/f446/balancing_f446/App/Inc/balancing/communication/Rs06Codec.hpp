#pragma once

#include "balancing/communication/can/CanTypes.hpp"

#include <cstdint>
#include <optional>

namespace balancing {

enum class Rs06Command : std::uint8_t { Enable, Stop };
enum class Rs06Report : std::uint8_t { Status = 2, Periodic = 24 };
enum class Rs06Mode : std::uint8_t { Reset, Calibration, Running };

struct Rs06StateReport {
    std::uint8_t motorId;
    std::uint8_t hostId;
    Rs06Report report;
    Rs06Mode mode;
    std::uint8_t faultBits;
    float angleRad;
    float velocityRadPerSec;
    float torqueNm;
    float temperatureC;
};

class Rs06Codec final {
public:
    /** @brief CAN1のRS06 Type 2/24の形式を検証し、状態報告を返す。
     *  @param[in] frame RouterでCAN1のデータフレームと判定した受信内容。
     *  @return 解析結果。予約モードや不正フレームはnullopt。
     *  @spec{PROTO-RS06-001}
     */
    static std::optional<Rs06StateReport> decodeFeedback(const CanFrame& frame) noexcept;

    /** @brief Type 3有効化またはType 4通常停止を構築する。
     *  @param[in] command 有効化または通常停止。
     *  @param[in] motorId 設定されたモータ番号（1〜255）。
     *  @param[in] hostId 設定された主機ID。
     *  @return 送信候補。モータ番号が無効ならnullopt。
     *  @note 通常停止では故障解除のビットを立てない。
     *  @spec{PROTO-RS06-001}
     */
    static std::optional<CanFrame> encodeControl(
        Rs06Command command, std::uint8_t motorId, std::uint8_t hostId) noexcept;
    /** @brief 純トルクType1。通信表現範囲外または非有限値ならnullopt。
     * @spec{PROTO-RS06-001}
     * @warning 駆動許可・物理上限は上位の責務。
     */
    static std::optional<CanFrame> encodeTorque(std::uint8_t motorId,float torqueNm) noexcept;
};

} // namespace balancing
