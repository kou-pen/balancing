#pragma once

#include "balancing/app/MotorService.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace balancing {

/** @brief ロータ角を車輪の移動距離に換算する、起動時固定の校正設定。
 *  @note 数学fixtureの値を実機検証済み設定として流用しない。
 *  @spec{CTRL-EST-001}
 */
struct WheelOdometryConfig {
    float wheelRadiusM;          // 車輪半径 [m]
    float motorRevsPerWheelRev;  // ロータ回転数 / 車輪回転数
    float direction;             // 前進に対するロータの符号（+1 / -1）
    bool verified;               // 実機校正の検証結果。数学fixtureは試験内に限定
};

struct WheelMotion {
    float positionM;             // C620Motorの原点世代に対する変位 [m]
    float velocityMps;           // ロータRPMからの車輪周速度 [m/s]
    std::uint16_t encoderRaw;    // 最後に受理した0..8191の生カウント
    std::uint64_t receivedAtUs;  // 元のCAN受信時刻（制御周期時刻ではない）
};

struct WheelOdometryAxis {
    std::optional<WheelMotion> latest;
    bool fresh;                  // MotorService側の鮮度、かつ最新報告の換算に成功
};

/** @brief 左右の位置・周速度を同一の制御周期時刻で固定した読み取り用コピー。
 *  @spec{CTRL-EST-001}
 */
struct WheelOdometrySnapshot {
    std::uint64_t sampledAtUs;
    std::array<WheelOdometryAxis, 2> wheel; // 左車輪・右車輪
};

enum class WheelUpdateResult : std::uint8_t {
    Updated,
    NoSample,
    NoNewSample,
    InvalidConfiguration,
    InvalidTimestamp,
    InvalidValue,
    ContinuityUnavailable,
    ReferenceMismatch,
    InvalidSequence
};

/** @brief C620生フィードバックを左右車輪の相対位置・周速度へ換算する。
 *  @note MotorServiceが検査するCAN鮮度は再計算しない。
 *        積算はC620Motorのみ。受信番号・原点世代を照合する。
 *  @spec{CTRL-EST-001}
 *  @spec{PROTO-C620-001}
 */
class WheelOdometry final {
public:
    /** @brief 両車輪の校正値を固定する。構築時にのみ設定を検査する。
     *  @param[in] config 左右車輪の独立した半径・減速比・方向。
     *  @note 設定を変更するAPIは提供しない。設定変更時は新インスタンスを構築する。
     *  @spec{CTRL-EST-001}
     */
    explicit WheelOdometry(const std::array<WheelOdometryConfig, 2>& config) noexcept;

    /** @brief 同じ周期のMotorServiceフィードバックから累積位置を物理量へ換算する。
     *  @param[in] motors CANイベント回収後の6軸の読み取り専用コピー。
     *  @return 軸別の更新結果。片側の不正値で他方の観測は捨てない。
     *  @pre 呼出し元が同じ周期の単調時刻でMotorService::feedback()を取得する。
     *  @spec{CTRL-EST-001}
     *  @spec{PROTO-C620-001}
     */
    std::array<WheelUpdateResult, 2> update(const MotorFeedbackSnapshot& motors) noexcept;

    /** @brief 最後の周期評価時刻と左右の観測をコピーする。
     *  @return 未受信の軸はlatestが空。失敗・期限切れならfreshはfalse。
     *  @spec{CTRL-EST-001}
     */
    WheelOdometrySnapshot snapshot() const noexcept;

    /** @brief 次の原点世代を待つ。MotorServiceの原点再設定と同期して呼ぶ。
     *  @pre DISARMEDかつ旧受信キューを排除し、MotorService::resetWheelReference()が成功済み。
     *  @spec{CTRL-EST-001}
     */
    void reset() noexcept;

    /** @brief 起動時の校正形式検査。実測妥当性とは別。
     * @spec{CTRL-EST-001}
     */
    static bool validConfig(const WheelOdometryConfig& config) noexcept;

private:
    WheelUpdateResult accept(std::size_t axis, const C620AxisFeedback& report,
                             std::uint64_t sampledAtUs) noexcept;

    std::array<WheelOdometryConfig, 2> config_;
    std::array<bool, 2> configValid_;
    std::array<std::optional<WheelMotion>, 2> last_{};
    std::array<bool, 2> fresh_{};
    std::array<std::uint64_t, 2> lastSequence_{};
    std::array<std::int64_t, 2> lastCounts_{};
    std::array<bool, 2> continuityLost_{};
    std::uint64_t generation_{1};
    bool generationExhausted_{false};
    std::uint64_t sampledAtUs_{0};
};

} // namespace balancing
