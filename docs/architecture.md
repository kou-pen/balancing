# ソフトウェア構成

**基準：#91までのmasterと車輪原点再設定のレビュー版。** 実機非ゼロ指令は未接続です。このページは目標構成ではなく実際のコードを中心に記述します。要求は [仕様](spec-index.md)、実装根拠は [検証記録](verification/index.md)。

## 1. ロボット全体

<pre class="mermaid">
flowchart LR
    PS5["DualSense"] -->|USB| F446["STM32F446"]
    PI["Raspberry Pi"] <-->|CAN2| F446
    PWR["F303 電源基板"] <-->|CAN2| F446
    IMU["BNO055"] -->|I2C| F446
    F446 <-->|CAN1| LEGS["RS06 脚4軸"]
    F446 <-->|CAN2| WHEEL["C620 車輪2軸"]
</pre>

図は機器の接続関係。F446に通信処理があっても実機での通信・駆動が確認済みという意味ではありません。F303/Piの対向コードは本リポジトリでは未確認です。

## 2. F446内部の構成

<pre class="mermaid">
flowchart TB
    CORE["Core/Src/main.c と割り込み"] --> APP["app/ Application"]
    CORE --> IO["device/ HAL・機器接続"]
    IO --> COMM["communication/ CAN・USB"]
    COMM --> APP
    APP --> CYCLE["app/ControlCycle"]
    CYCLE --> EST["control/estimator"]
    IO --> FILTER["control/filter 姿勢フィルタ"]
    FILTER --> APP
    CYCLE --> SEQ["RobotSequence 状態判定"]
    CYCLE --> CTRL["control/controller 数学演算"]
    APP --> DRIVE["DriveRuntime 停止専用出力"]
    DRIVE --> TX["MotorPeriodicTx 共通周期送信管理"]
    TX --> COMM
    CORE -. 別ビルド .-> BENCH["bench/ MotorBench"]
</pre>

| 場所 | 主な責務 | 入口 |
|---|---|---|
| `Core/` | CubeMX生成初期化、TIM7割り込み、主ループ | `Core/Src/main.c` |
| `App/.../app/` | 一周期の管理、運転状態、入力・電源・診断、モータ集合 | `application.cpp` |
| `App/.../control/` | 状態推定、LQR/VMC、機構側→モータ側変換 | `ControlCycle.cpp` から呼ぶ |
| `App/.../communication/` | ワイヤ形式の符号化、CANメールボックス、送信順序 | `CanRouter.cpp`、`CanTxScheduler.cpp` |
| `App/.../device/` | 機器1台の状態・HAL接続・受信時刻 | `f446_can1_runtime.cpp`、`f446_imu_runtime.cpp` |
| `App/.../bench/` | 通常運転と独立した単軸試運転 | `f446_motor_bench_runtime.cpp` |

詳細は [フォルダ別コード案内](design/public-api.md)。実行順は [起動・周期処理図](diagrams.md)。

## 3. 三つの実行経路を混同しない

| 対象 | 何が起動するか | 駆動出力 |
|---|---|---|
| Debug / Release | 通常Application、CAN1/2、IMU起動判断、TIM7、IWDG起動判断 | 既定はモータ送信なし。#88の明示設定時のみRS06停止＋C620全ゼロ |
| BenchDebug | 試運転専用の起動経路 | 既定の未検証設定ではCAN開始前に停止。明示設定で停止確認。パルス入力は未接続 |
| PCのSIL | `tests/sil/DriveRuntime.cpp` と `DriveCycle.cpp` | 人工モデルによるARMING・6軸指令を模擬CANで検証。上位の駆動実装はSTM32にはリンクしない。周期送信管理は共通 |

実行中に通常/試運転を切り替える機能はありません。通常ビルドにSILの人工ゲインを投入していません。

## 4. 周期と実装状況

| 処理 | 設計周期 | 現在 |
|---|---|---|
| TIM7、状態推定・制御 | 500 Hz / 2 ms | 実装・ホスト検証。実時間未測定 |
| RS06 4軸の通常駆動 | 250 Hz / 4 ms | SILのみ。実機用Type1/Type3は未接続 |
| C620共通電流 | 500 Hz / 2 ms | SIL通常指令、#88通常停止専用ゼロ指令 |
| CAN・IMU受信 | 機器依存 | IRQ／非同期回収と時刻管理を実装 |
| IWDG | 公称500 ms候補 | 更新経路を実装、既定無効、実測未実施 |

状態機械、制御数学、CAN・USB・I2C、表示・診断は実装済みの部分とホスト検証済みの部分があります。どの設定も実機の許可を意味しません。 [最新の検証](verification/index.md) と [残作業](todo/f446.md) を確認してください。

## 5. 安全境界

運転状態は DISARMED / ARMING / BALANCING / FAULT。起動時固定設定の不足と周期観測の不成立は ApplicationObservation::armReadiness に別々に記録する。これは読み取り専用の診断で、許可判定を代替しない。数学fixtureが形式検査を通っても実機駆動未接続・機構安全未検証は解消しない。`RobotSequence` が状態を決定し、`ControlCycle` の数学結果だけではモータ送信を許可しません。通常運転の非ゼロ出力はまだ未接続です。物理非常停止、支持治具、原点・極性・ゲイン・物理上限、実時間・実機動作は未検証。 [動作仕様](specs/behavior/robot_operation.md) と [基板仕様](specs/boards/f446_control.md) が要求の正本です。
