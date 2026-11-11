#include "../../../Inc/balancing_app_bridge.h"

#include <cstdint>

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
        control_ticks_ = 0;
        initialized_ = true;
    }

    /*
     * 2ms制御ティックの将来的な接続点。
     * 現状は呼出し回数を記録するだけで、C11コアやCANは駆動しない。
     * 単なるTIM7割込みを安全な500Hz制御実行と混同しない。
     */
    void controlTick() noexcept
    {
        if (!initialized_) {
            return;
        }

        /* wrapするuint32_tは診断用のカウンタであり、
         * 安全タイムアウトや測定時刻の代用には使わない。 */
        ++control_ticks_;
        // TODO(T-F446-11): 既存のC11制御コア、CAN1/2アダプタと結合する。
        // 明示的な安全確認とモータID/換算値の確定までは出力を行わない。
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
        return control_ticks_;
    }

private:
    std::uint32_t control_ticks_{0};
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

} // extern "C"
