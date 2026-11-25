#include "balancing_app_bridge.h"
#include "balancing/device/Can1Observation.h"

#include <cstdint>
#include <optional>

/*
 * CubeMX生成main.cからはC ABIのみを公開する。C++のApplicationは
 * HALから独立した入口を保持し、初期化前や実機パラメータ未検証時には
 * いかなるモータ送信・ARM処理も開始しない。
 */
namespace balancing {

class Application final {
public:
    /*
     * 静的オブジェクトの初期値だけを確定する。通信初期化やモータ有効化は
     * 本番配線・校正条件が決まるまで追加しない。
     */
    void init() noexcept
    {
        timing_ = {};
        previousUs_.reset();
        initialized_ = true;
    }

    /*
     * 既存mainがCAN1イベントを回収した直後の診断周期。
     * 実測時刻を採用し、TIM7の呼出し回数から2msを仮定しない。
     */
    void controlTick() noexcept
    {
        if (!initialized_) {
            return;
        }

        ++timing_.control_ticks;
        timing_.sample_valid = false;
        timing_.elapsed_valid = false;
        timing_.sampled_at_us = 0;
        timing_.elapsed_us = 0;

        BF446Can1Snapshot observation{};
        if (!balancing_f446_can1_snapshot(&observation) || !observation.sampled) {
            // 取得不能区間をまたいだ差分は一周期の実測値として公開しない。
            previousUs_.reset();
            return;
        }
        const auto nowUs = observation.sampled_at_us;
        if (previousUs_) {
            if (nowUs == *previousUs_) {
                ++timing_.duplicate_samples;
                return;
            }
            if (nowUs < *previousUs_) {
                // 逆行した時計で正常な基準を巻き戻さない。
                ++timing_.clock_regressions;
                return;
            }
            timing_.elapsed_us = nowUs - *previousUs_;
            timing_.elapsed_valid = true;
        }
        // 初回は採取時刻だけ有効。次の異なる時刻が来て初めて差分を得る。
        previousUs_ = nowUs;
        timing_.sampled_at_us = nowUs;
        timing_.sample_valid = true;
        // TODO(T-F446-11): 実IMU/CAN2/校正を確認してからControlCycleへ接続する。
    }

    /*
     * メインループの非ブロッキング診断処理を置く場所。
     * I2C/USB/CAN受信を統合する場合は制御周期の締切を妨げない設計が必要。
     */
    void poll() noexcept
    {
        if (!initialized_) {
            return;
        }
        // TODO(T-F446-05): 非ブロッキング通信・診断処理を置く。
    }

    /* 試験・診断でC ABIから観測するだけのカウンタ。
     * 実行時刻の正確性や安全動作の証明にはならない。 */
    std::uint32_t controlTickCount() const noexcept
    {
        return timing_.control_ticks;
    }

    bool timingSnapshot(BF446AppTimingSnapshot* snapshot) const noexcept
    {
        if (!initialized_ || snapshot == nullptr) {
            return false;
        }
        *snapshot = timing_;
        return true;
    }

private:
    BF446AppTimingSnapshot timing_{};
    std::optional<std::uint64_t> previousUs_{};
    bool initialized_{false};
};

// 静的に一つだけ保持。malloc/newも静的初期化時のHAL利用も行わない。
Application application;

} // namespace balancing

// main.cはCとしてコンパイルするので、公開関数だけCリンケージにする。
extern "C" {

/* CubeMX CからC++を初期化するブリッジ。リンク境界以外の処理を持たない。 */
void balancing_app_init(void)
{
    balancing::application.init();
}

/* メインのタイマ側から呼ぶC ABI。制御ロジックはC++内に閉じる。 */
void balancing_app_control_tick(void)
{
    balancing::application.controlTick();
}

/* 低優先度のポーリング入口。ISR内のブロッキング動作を増やさない。 */
void balancing_app_poll(void)
{
    balancing::application.poll();
}

/* ホスト試験でブリッジの呼出し整合を検証するための読み取り専用API。 */
uint32_t balancing_app_control_tick_count(void)
{
    return balancing::application.controlTickCount();
}

/* 主ループの診断コピー。センサ・駆動の状態判定とは分離する。 */
bool balancing_app_timing_snapshot(BF446AppTimingSnapshot* snapshot)
{
    return balancing::application.timingSnapshot(snapshot);
}

} // extern "C"
