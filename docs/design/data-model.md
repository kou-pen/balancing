# データ構造と所有権

型の定義・単位・失敗条件は [フォルダ別コード案内](public-api.md) から実コードを参照。ここでは「どのデータがどこにあり、誰が変えるか」だけを示します。

## 状態の所有者

| 所有者 | 保持する状態 | 他の層へ渡す値 |
|---|---|---|
| `CanRuntime`（f446_can1_runtime.cpp） | 2バスのCanPort・Scheduler、MotorService、共通DWT時計、回収番号 | MotorObservationSnapshot |
| `MotorService` | Rs06Motor 4個・C620Motor 2個、ARMセッション | MotorFeedbackSnapshot、Rs06ArmFeedback |
| `Rs06Motor` / `C620Motor` | 軸ごとの受信値・元時刻、RS06送信状態または車輪連続性 | 軸別フィードバック・状態 |
| `ImuAttitudeSource` | 校正・相補フィルタ・姿勢 | ImuAttitude |
| `ObservationCycle`（application.cpp） | PowerBoardService、RemoteControlService、DualSenseService、UsbHostPort、診断・表示、推定・状態機械・制御 | 周期処理用の各サービス |
| `RobotSequence` | 運転状態、ARM期限・セッション、FAULT記録 | RobotSequenceResult |
| `StateEstimator` | WheelOdometryと脚・車輪の設定 | ControlSnapshot または StateEstimateError |
| `BalanceControl` | 脚長基準、Yaw積分、固定ゲイン | BalanceControlOutput（機構側Nm） |
| `MotorCommand` | 6軸別の固定換算・制限設定 | MotorCommandPreview（モータNm/A） |
| `DriveRuntime` | #88の停止専用予約・受付・終端の記録 | StopOutputSummary |
| `Application` | 周期時刻・完了証跡、上記ObservationCycle、最新診断・固定設定と観測の不足ビット | ApplicationObservation |
| `MotorBench` | 別ビルドの単軸機器・停止・試運転状態 | MotorBenchSummary |

## 一周期に流れる値

<pre class="mermaid">
flowchart LR
    RAW["機器ごとの受信値＋時刻"] --> MOT["MotorObservationSnapshot"]
    MOT --> IN["ControlTickInput"]
    IMU["ImuAttitude"] --> IN
    OPS["操作イベント・電源・故障"] --> IN
    IN --> EST["StateEstimator"]
    EST --> CS["ControlSnapshot"]
    IN --> SEQ["RobotSequence"]
    CS --> CTRL["BalanceControl"]
    CTRL --> TOR["BalanceControlOutput"]
    TOR --> PRE["MotorCommandPreview"]
    SEQ --> RESULT["ControlTickResult"]
    PRE --> RESULT
    RESULT --> OBS["ApplicationObservation"]
</pre>

| 型 | 定義ファイル | 意味 |
|---|---|---|
| MotorObservationSnapshot | device/MotorObservation.hpp | CAN回収番号、共通時刻、6軸、バス故障 |
| MotorFeedbackSnapshot | app/MotorService.hpp | 脚4軸・車輪2軸の観測コピー |
| StateEstimatorInput / ControlSnapshot | control/estimator/StateEstimator.hpp | IMU＋6軸 → 機体・車体・脚・車輪の数学状態 |
| RobotSequenceInput / RobotSequenceResult | app/RobotSequence.hpp | 保護条件・操作・応答 → 状態と駆動要求 |
| ControlTickInput / ControlTickResult | app/ControlCycle.hpp | 一周期の入力と結果（送信指令ではない） |
| BalanceControlOutput / MotorCommandPreview | control/controller/ の各.hpp | 機構トルク → モータ軸トルク・電流の数学プレビュー |
| ApplicationObservation | app/ApplicationObservation.hpp | 最新の周期・電源・操作・制御・診断・監視のコピー |

観測時刻は機器の新規受信でだけ更新する。コピー時刻と元の測定時刻を混同しない。`verified` は確認済み設定の前提であり、人工試験でtrueにしただけでは実機の確認にならない。`fixtureOnly` は数学試験値であることを示す。

設計規則は [AGENTS.md](../../AGENTS.md)。以前の型・移行履歴は [整理前の全文（Git履歴）](https://github.com/kou-pen/balancing/blob/cb3abeda89acdbcb08fd9890735f2b8c95ed9d83/docs/design/data-model.md)。
