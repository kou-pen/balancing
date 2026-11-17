# ソフトウェア構成

**目標は、入力・推定・制御・駆動を分離し、最後に安全状態で出力を許可すること。** このページの図には未実装の機能も含まれます。

## 全体像

<pre class="mermaid">
flowchart TB
    INPUT["操作・観測・電源状態"] --> APP["Application：周期入力を固定"]
    APP --> EST["StateEstimator：機体・車体・脚・車輪"]
    APP --> SEQ["RobotSequence：状態と駆動許可"]
    EST --> CTRL["BalanceControl：6軸の要求トルク"]
    EST --> SEQ
    SEQ --> CTRL
    CTRL --> CMD["MotorCommand：変換と上限"]
    CMD --> FINAL["Application：周期の出力を確定"]
    SEQ --> FINAL
    FINAL --> MOTOR["MotorService：RS06／C620"]
    SEQ --> DIAG["DiagnosticsService：PCへCAN診断"]
</pre>

C++17の設計構成です。現在のC++入口は非駆動です。各型・関数の責務は [公開API設計](design/public-api.md)、呼び出し順は [起動・周期・停止の詳細図](diagrams.md) を参照してください。

## 処理周期

| 処理 | 設計周期 | 現在の検証 |
|---|---|---|
| 状態推定・LQR/VMC | 500 Hz（2 ms） | 制御コアの一部をホスト検証。実機の時間保証は未検証 |
| RS06 4軸への送信要求 | 250 Hz（4 ms） | 3メールボックスの仮想時刻モデルでホスト検証 |
| C620共通電流指令 | 500 Hz（2 ms） | 送信要求モデルとCAN codecをホスト検証 |

受信した値がすべて500 Hzで更新されるという意味ではありません。実際の通信周期・バス負荷・送信完了期限は実機で別途確認します。

## 実装済みと未接続を分ける

| 層 | 現状 | 詳細 |
|---|---|---|
| C++入口・CubeIDE | 現行10ソースのDebug/Release ARMビルドPASS | [現行ホスト・ビルド記録](verification/issue10-observation-host.md) |
| 状態機械・LQR数値関数 | ホスト試験PASS。一部の仕様不一致は残存 | [制御コア](software-core.md)・[仕様差分](verification/core-mode-contract-audit.md) |
| RS06/C620通信と送信調停 | コーデック・スケジューラのホスト試験PASS | [モータ通信](design/motor-layer.md)・[送信モデル](design/can-scheduler.md) |
| IMU/車輪と操作入力 | 時刻付き入力、相補フィルタ、USB/CAN操作元モックのホスト試験PASS。実I/Oは未接続 | [観測](design/observation-snapshot.md)・[入力](design/operator-input.md)・[姿勢フィルタ](design/attitude-filter.md) |
| VMC・実CAN駆動 | 通常運転経路は未完成 | [ロードマップ](todo/roadmap.md) |
| F303電源連携・実機倒立 | 未検証 | [実機検証TODO](todo/verification.md) |

## 安全に関する境界

公開状態は **DISARMED／ARMING／BALANCING／FAULT** です。ARMINGでは全体の絶対期限内に、RS06全4軸のType 3のTx完了後に届く正常応答を確認します。駆動許可が落ちた周期は、旧非ゼロ指令を無効化して停止要求に切り替えます。独立した物理非常停止はソフトウェアとCANの外側で電源を遮断します。

安全条件と数値の正本は [動作仕様](specs/behavior/robot_operation.md)、[F446仕様](specs/boards/f446_control.md)、[F303仕様](specs/boards/f303_power.md) です。未確定のモータ設定やゲインを推測で採用しません。
