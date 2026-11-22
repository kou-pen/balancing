#pragma once

#include "balancing/communication/balancing_motor_can.h"

#include <cstdint>
#include <optional>

namespace balancing {

struct C620FeedbackSample {
    BC620Feedback report;
    std::uint64_t receivedAtUs;
};

struct C620AxisFeedback {
    std::optional<C620FeedbackSample> latest;
    bool fresh;
};

/** @brief C620の1軸分の受信値を保持する。
 *  @spec{PROTO-C620-001}
 */
class C620Motor final {
public:
    /** @brief この軸に配送された復号済みフィードバックを記録する。
     *  @param[in] report ロータエンコーダ生値、速度 [rpm]、電流生値、温度 [℃]。
     *  @param[in] receivedAtUs CANで採取した単調時刻 [us]。
     *  @pre MotorServiceが受信IDに対応する軸を選んでから呼ぶ。
     *  @spec{PROTO-C620-001}
     */
    void onFeedback(const BC620Feedback& report, std::uint64_t receivedAtUs) noexcept;

    /** @brief 最新報告と、今周期の時刻で評価した鮮度を返す。
     *  @param[in] nowUs 制御周期で固定した単調時刻 [us]。
     *  @return 未受信ではlatestが空。期限切れでも最終受信値と受信時刻は保持する。
     *  @spec{PROTO-C620-001}
     */
    C620AxisFeedback feedback(std::uint64_t nowUs) const noexcept;

private:
    std::optional<C620FeedbackSample> latest_{};
};

} // namespace balancing
