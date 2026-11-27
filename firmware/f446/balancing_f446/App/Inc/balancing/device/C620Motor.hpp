#pragma once

#include "balancing/communication/C620Codec.hpp"

#include <cstdint>
#include <optional>

namespace balancing {

/** @brief 検証済みの隣接報告間隔上限 [us]。実機の速度・通信遅延から別途決定する。
 *  @spec{PROTO-C620-001}
 */
struct C620ContinuityConfig {
    std::uint64_t maxCaptureGapUs{0};
    bool verified{false};
};

enum class C620ContinuityStatus : std::uint8_t {
    Waiting, Unverified, Continuous, TransportLost, InvalidValue,
    ClockRegression, GapExceeded, AmbiguousDelta, CounterOverflow
};

struct C620FeedbackSample {
    C620Feedback report;
    std::uint64_t receivedAtUs;
    std::uint64_t sequence{0}; // 受理順。同一IRQ時刻でも別報告を識別する
};

struct C620EncoderPosition {
    std::int64_t counts;        // 原点からのロータ累積カウント [count]
    std::uint64_t sequence;     // 対応する最新報告の受理番号
    std::uint64_t generation;   // 原点世代。初期値1
};

struct C620AxisFeedback {
    std::optional<C620FeedbackSample> latest;
    bool fresh{false}; // 通信鮮度のみ。位置の連続性とは独立
    std::optional<C620EncoderPosition> position{};
    C620ContinuityStatus continuity{C620ContinuityStatus::Unverified};
    std::uint64_t generation{1};
};

/** @brief C620の1軸分の報告と受信順の相対ロータ位置を保持する。
 *  @note 特殊な巻数推定は行わない。隣接実測値の変位が半周未満という外部検証が必須。
 *  @spec{PROTO-C620-001}
 *  @spec{CTRL-EST-001}
 */
class C620Motor final {
public:
    /** @brief 起動時に連続性の検証条件を固定する。既定では位置を公開しない。
     *  @param[in] config 検証済み取得間隔の排他的上限 [us]。
     *  @spec{PROTO-C620-001}
     */
    explicit C620Motor(C620ContinuityConfig config = {}) noexcept;

    /** @brief 受信順に復号済み報告を処理する。同一時刻でも別報告として積算する。
     *  @param[in] report 該当軸のロータ角 [count]、速度 [rpm]、電流、温度。
     *  @param[in] receivedAtUs CAN採取時刻 [us]。
     *  @pre 主ループから軸別の到着順に呼ぶ。
     *  @spec{PROTO-C620-001}
     */
    void onFeedback(const C620Feedback& report, std::uint64_t receivedAtUs) noexcept;

    /** @brief 最新報告、通信鮮度、連続性が成立した累積位置をコピーする。
     *  @param[in] nowUs 周期の評価時刻 [us]。
     *  @spec{PROTO-C620-001}
     */
    C620AxisFeedback feedback(std::uint64_t nowUs) const noexcept;

    /** @brief 欠落をラッチし、以降の正常報告で原点を自動再設定しない。
     *  @pre 主ループのみ。
     *  @spec{CTRL-EST-001}
     */
    void invalidateContinuity() noexcept;

    /** @brief 明示的に原点世代を進め、次の新規報告を待つ。
     *  @return 世代を更新できた場合true。世代の周回は禁止。
     *  @pre DISARMED、旧受信キュー排除済み、上位推定器のresetと同期。
     *  @warning 運用中の自動回復には使わない。
     *  @spec{CTRL-EST-001}
     */
    bool resetReference(std::optional<std::uint64_t> boundaryUs={}) noexcept;
    /** @brief 原点更新前の形式・世代・時刻検査。更新は行わない。
     * @spec{CTRL-EST-001}
     */
    bool canResetReference(std::uint64_t generation,
                           std::optional<std::uint64_t> boundaryUs={}) const noexcept;

    /** @brief 受信連続性設定の形式検査。半周未満の実測条件は別。
     * @spec{PROTO-C620-001}
     */
    static bool validConfig(C620ContinuityConfig) noexcept;

private:
    void lose(C620ContinuityStatus reason) noexcept;

    C620ContinuityConfig config_;
    bool configValid_;
    C620ContinuityStatus continuity_;
    std::optional<C620FeedbackSample> latest_{};
    std::optional<std::uint64_t> lastTimestamp_{}, referenceBoundaryUs_{};
    std::int64_t counts_{0};
    std::uint64_t sequence_{0};
    std::uint64_t generation_{1};
};

} // namespace balancing
