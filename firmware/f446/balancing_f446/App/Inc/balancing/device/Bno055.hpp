#pragma once
#include "balancing/device/I2cPort.hpp"
#include "balancing/control/estimator/balancing_observation.h"

namespace balancing {
struct Bno055Config {
    std::uint64_t maxEvidenceWindowUs{0}; // 取得の時間窓上限、実機検証値のみ
    std::uint64_t maxRequestWaitUs{0};    // BUSY待ち上限
    bool timingVerified{false};
};
struct TimedImuSample {
    BImuSample value{};
    std::uint64_t earliestUs{0}; // 実測時刻の下限であり厳密な測定時刻ではない
    std::uint64_t completedAtUs{0};
    bool newMeasurementProven{false};
};
enum class Bno055State : std::uint8_t { Initializing, Observing, Failed, Unverified };
/** @brief ACCGYROのレジスタ取得。DRDYを消費し、新規性を確認する。
 * @spec{CTRL-EST-001}
 */
class Bno055 final {
public:
    /** @brief バスと検証済み時間窓を固定する。
     * @spec{BOARD-F446-001}
     */
    Bno055(I2cPort& port, Bno055Config config) noexcept;
    /** @brief 非同期転送を1段階進める。待ち時間は時刻比較する。
     * @spec{BOARD-F446-001}
     */
    void poll(std::uint64_t nowUs) noexcept;
    /** @brief 最後の生観測を一度だけ取得。未証明なら制御に使わない。
     * @spec{CTRL-EST-001}
     */
    std::optional<TimedImuSample> takeSample() noexcept;
    Bno055State state() const noexcept { return state_; }
private:
    enum class Step : std::uint8_t { Page0, Identity, ConfigMode, WaitConfig, Units, Map,
        MapCheck, Page1, Enable, EnableCheck, Page0Again, Run, WaitRun, CheckMode,
        Flush, Status, Data };
    void complete(const I2cCompletion& result) noexcept;
    I2cPort& port_;
    Bno055Config config_;
    Bno055State state_;
    Step step_{Step::Page0};
    std::uint64_t waitSince_{0};
    std::optional<std::uint64_t> requestSince_{};
    std::optional<std::uint64_t> lastPoll_{};
    std::uint64_t lowerBound_{0};
    std::uint8_t ready_{0};
    std::optional<TimedImuSample> sample_{};
};
}
