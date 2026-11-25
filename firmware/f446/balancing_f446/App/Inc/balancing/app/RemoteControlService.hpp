#pragma once
#include "balancing/app/RobotSequence.hpp"
#include "balancing/app/balancing_operator_input.h"

namespace balancing {
/** @brief 復号済み操作入力を単一所有し、既存の変化率・復帰処理を利用する。
 * @spec{BEHAV-001}
 * @spec{PROTO-REMOTE-001}
 * @spec{PROTO-PS5-001}
 */
class RemoteControlService final {
public:
    /** @brief 検証済みの物理量設定で初期化する。既定は無効。
     * @spec{BEHAV-001}
     */
    explicit RemoteControlService(const BOperatorInputConfig& config = {}) noexcept;
    /** @brief DISARMED中の明示切替のみを受理する。
     * @spec{PROTO-REMOTE-001}
     */
    bool selectSource(BOperatorSource source, RobotState state) noexcept;
    /** @brief 通信層で新規性・形式・送信元を確認済みの入力のみ受理する。
     * @spec{BEHAV-001}
     */
    bool acceptDecoded(BOperatorSource source, const BOperatorSample& sample,
                       bool newSampleProven, std::uint64_t nowUs) noexcept;
    /** @brief 操作量の周期コピー。ARM要求は生成しない。
     * @spec{BEHAV-001}
     */
    std::optional<BOperatorTarget> update(RobotState state, std::uint64_t nowUs) noexcept;
private:
    BOperatorInput input_{};
};
}
