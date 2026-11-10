# ソフトウェア構成

**目標は、入力・推定・制御・駆動を分離し、最後に安全状態で出力を許可すること。** このページの図には未実装の機能も含まれます。

## 全体像

<pre class="mermaid">
flowchart LR
    INPUT["操作入力：PS5 / 上位CAN"] --> APP["F446 アプリケーション"]
    SENSOR["IMU / モータ / 電源の観測値"] --> SNAP["時刻付き入力"]
    SNAP --> EST["状態推定・脚の運動学"]
    EST --> CTRL["LQR / PD・PI / VMC"]
    APP --> CTRL
    SNAP --> SAFE["安全状態機械"]
    APP --> SAFE
    CTRL --> GATE{"出力許可"}
    SAFE --> GATE
    GATE --> RS["RS06：CAN1"]
    GATE --> C620["C620：CAN2"]
    PWR["F303：電源・非常停止"] --> SAFE
</pre>

これは**目標とする論理構成**です。現在のC++入口は非駆動であり、図のすべてが実機へ接続されているわけではありません。[起動・周期・停止の詳細図](diagrams.md)では、各場面を分けて確認できます。

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
| C++入口・CubeIDE | 6ソースのDebug/Release ARMビルドPASS | [F446配置・ビルド記録](verification/f446-module-layout.md) |
| 状態機械・LQR数値関数 | ホスト試験PASS。一部の仕様不一致は残存 | [制御コア](software-core.md)・[仕様差分](verification/core-mode-contract-audit.md) |
| RS06/C620通信と送信調停 | コーデック・スケジューラのホスト試験PASS | [モータ通信](design/motor-layer.md)・[送信モデル](design/can-scheduler.md) |
| センサ統合・VMC・実CAN駆動 | 通常運転経路は未完成 | [ロードマップ](todo/roadmap.md) |
| F303電源連携・実機倒立 | 未検証 | [実機検証TODO](todo/verification.md) |

## 安全に関する境界

**DISARMED／BALANCING／FAULT** の3状態を扱います。RS06有効化には各軸の新規応答が必要で、送信要求と送信完了は区別します。ソフトウェアからの停止指令だけでは、バスオフやMCU停止時の安全停止を保証できません。独立した物理非常停止が必要です。

安全条件と数値の正本は [動作仕様](specs/behavior/robot_operation.md)、[F446仕様](specs/boards/f446_control.md)、[F303仕様](specs/boards/f303_power.md) です。未確定のモータ設定やゲインを推測で採用しません。
