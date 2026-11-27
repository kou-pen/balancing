# F446 フォルダ別コード案内

基準は駆動実行共通化の実装版（51bd7bf）。全98 Appファイルの所在はこのページとDoxygenの実フォルダ一覧で確認できます。旧tests/sil/DriveCycleの2ファイルはAppのDriveExecutionへ移設しました。

## 1. ディレクトリ

```text
firmware/f446/balancing_f446/
├─ Core/          CubeMX生成 main.c・IRQ・HAL初期化
├─ Drivers/       STM32 HAL / CMSIS
├─ Middlewares/   ST USB Hostライブラリ
└─ App/           自作C++17（Inc=宣言、Src=実装）
   ├─ Examples/  コメント例
   ├─ Inc/        C ABIブリッジとbalancing/以下の宣言
   └─ Src/balancing/
      ├─ app/              周期・状態・機器集合
      ├─ bench/            別ビルドの単軸試運転
      ├─ communication/
      │  ├─ can/           ポート・配送・調停・Codec
      │  └─ usb/           HIDプロトコル
      ├─ control/
      │  ├─ controller/    LQR・VMC・最終指令換算
      │  ├─ estimator/     脚・車輪・機体状態
      │  └─ filter/        姿勢フィルタ
      └─ device/           HAL接続・モータ個体
```

Inc側には同名の機能フォルダと、ヘッダーだけのcore/があります。Appの自作実装は45本の.cpp、旧.cは0本。Core/Drivers/MiddlewaresのCコードは別です。

## 2. 最初に追うべきファイル

| 目的 | 起点 | 次に読む場所 |
|---|---|---|
| 起動処理 | Core/Src/main.c | application.cpp または f446_motor_bench_runtime.cpp |
| 制御周期 | application.cpp | DriveRuntime.cpp → DriveExecution.cpp（明示設定時）→ ControlCycle.cpp |
| 車輪原点の明示再設定 | application.cpp の resetApplicationWheelReference | f446_can1_runtime.cpp → CanPort / MotorService → StateEstimator |
| ARM準備・不足設定 | application.cpp の inspectArmConfiguration / inspectArmObservations | DriveRuntime.hpp、ApplicationObservation.hpp |
| 運転許可・FAULT | RobotSequence.cpp | MotorService.cpp、Rs06Motor.cpp |
| CAN受信・送信 | f446_can1_runtime.cpp | CanPort.cpp → CanRouter.cpp、CanTxScheduler.cpp |
| 数学処理 | ControlCycle.cpp | StateEstimator.cpp、BalanceControl.cpp、MotorCommand.cpp |

[処理順序の図](../diagrams.md) と合わせて読むと、どの順に呼ばれるか確認できます。以下のファイル名は基準コミットのGitHubソースへのリンクです。オフラインのDoxygenでは左側の firmware/ → f446/ → balancing_f446/ → App/ → Inc/ または Src/ から同じファイルを開けます。実際の引数・戻り値はヘッダーのDoxygenを参照してください。

## 3. フォルダ別ファイル一覧

パスは App/Inc/balancing/ と App/Src/balancing/ を基準とします。対応する実装が別名の場合も実ファイルを示します。「ヘッダー内」は独立した.cppを持たない型または実装です。

### app/ — 周期・安全状態・サービス管理

| 宣言 | 実装 | 責務 |
|---|---|---|
| [ApplicationObservation.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/ApplicationObservation.hpp) | [application.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/application.cpp) | 非駆動周期の入力・状態・停止送信・診断コピー、明示原点再設定 |
| [Can2ServiceLink.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/Can2ServiceLink.hpp) | [Can2ServiceLink.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/Can2ServiceLink.cpp) | 自作CAN2の受信・周期入力・応答・配信契約 |
| [ControlCycle.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/ControlCycle.hpp) | [ControlCycle.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/ControlCycle.cpp) | 時刻付き観測・状態機械・数学制御を1周期で処理する契約 |
| [CycleHealthMonitor.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/CycleHealthMonitor.hpp) | [CycleHealthMonitor.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/CycleHealthMonitor.cpp) | 開始番号・完了証跡の照合、IWDG更新境界 |
| [DiagnosticsService.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/DiagnosticsService.hpp) | [DiagnosticsService.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/DiagnosticsService.cpp) | 診断イベント・固定領域・送信進捗 |
| [DriveRuntime.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/DriveRuntime.hpp) | [DriveRuntime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/DriveRuntime.cpp) | 周期実行・送信終端・CAN所有者結合の共通契約 |
| [DriveExecution.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/DriveExecution.hpp) | [DriveExecution.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/DriveExecution.cpp) | 共通周期実行、駆動要求・停止・周期送信の接続 |
| [DualSenseService.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/DualSenseService.hpp) | [DualSenseService.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/DualSenseService.cpp) | USB報告連続性、操作イベントと長押し判定 |
| [MotorCanLink.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/MotorCanLink.hpp) | [MotorCanLink.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/MotorCanLink.cpp) | 機器報告・RS06停止/有効化の配送、MotorPeriodicTxによる共通周期送信管理 |
| [MotorService.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/MotorService.hpp) | [MotorService.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/MotorService.cpp) | RS06のARM・停止と、脚・車輪6軸の周期観測をまとめるAPI |
| [PowerBoardService.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/PowerBoardService.hpp) | [PowerBoardService.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/PowerBoardService.cpp) | 電源報告の受信API・生メタデータ・500ms期限・故障保持 |
| [RemoteControlService.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/RemoteControlService.hpp) | [RemoteControlService.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/RemoteControlService.cpp) | 操作元・設定・サンプル・目標型と操作管理API |
| [RobotSequence.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/RobotSequence.hpp) | [RobotSequence.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/RobotSequence.cpp) | 運転状態、FAULT、要求と結果の公開型 |
| [StatusIndicator.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/app/StatusIndicator.hpp) | [StatusIndicator.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/StatusIndicator.cpp) | 状態別の論理表示パターン |

### control/controller/ — LQR・VMC・脚長/Roll/Yaw・指令換算

| 宣言 | 実装 | 責務 |
|---|---|---|
| [BalanceControl.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/control/controller/BalanceControl.hpp) | [BalanceControl.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/control/controller/BalanceControl.cpp) | LQR・LegPD/RollPD・VMC・Yaw PIの数値合成API |
| [MotorCommand.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/control/controller/MotorCommand.hpp) | [MotorCommand.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/control/controller/MotorCommand.cpp) | RS06・C620の6軸独立換算設定と数学プレビューAPI |

### control/estimator/ — 機構・車輪・状態推定

| 宣言 | 実装 | 責務 |
|---|---|---|
| [LegKinematics.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/control/estimator/LegKinematics.hpp) | [LegKinematics.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/control/estimator/LegKinematics.cpp) | 2入力閉リンクの脚状態と数学fixtureの公開契約 |
| [StateEstimator.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/control/estimator/StateEstimator.hpp) | [StateEstimator.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/control/estimator/StateEstimator.cpp) | IMU・RS06・車輪の同一周期スナップショットAPI |
| [WheelOdometry.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/control/estimator/WheelOdometry.hpp) | [WheelOdometry.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/control/estimator/WheelOdometry.cpp) | C620左右の位置・速度換算と原点リセットの公開契約 |

### control/filter/ — 姿勢推定

| 宣言 | 実装 | 責務 |
|---|---|---|
| [ImuAttitudeSource.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/control/filter/ImuAttitudeSource.hpp) | [ImuAttitudeSource.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/control/filter/ImuAttitudeSource.cpp) | 新規IMU測定と姿勢フィルタ |

### communication/ — フレーム変換・送受信

| 宣言 | 実装 | 責務 |
|---|---|---|
| [C620Codec.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/C620Codec.hpp) | [C620Codec.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/C620Codec.cpp) | C620の電流フレーム・フィードバックの型とAPI |
| [Rs06Codec.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/Rs06Codec.hpp) | [Rs06Codec.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/Rs06Codec.cpp) | RS06通信の状態型とフレーム変換の宣言 |

### communication/can/ — CANポート・配送・送信調停・プロトコル

| 宣言 | 実装 | 責務 |
|---|---|---|
| [Can2FilterPlan.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/Can2FilterPlan.hpp) | [Can2FilterPlan.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/Can2FilterPlan.cpp) | CAN2の32bit Listフィルタ構成と生成API |
| [CanPort.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/CanPort.hpp) | [CanPort.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/CanPort.cpp) | 1バスの送受信、IRQ、メールボックスと共通DWT時計のAPI |
| [CanRouter.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/CanRouter.hpp) | [CanRouter.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/CanRouter.cpp) | CANイベントの交互回収と配送API・CAN2自作通信6分類 |
| [CanTxScheduler.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/CanTxScheduler.hpp) | [CanTxScheduler.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/CanTxScheduler.cpp) | 対象別の停止要求、通常FIFO、周期送信、受付と終端の照合API |
| [CanTypes.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/CanTypes.hpp) | ヘッダー内 | フレーム、トークン、送信結果、IRQイベントの共通型 |
| [DiagnosticsCodec.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/DiagnosticsCodec.hpp) | [DiagnosticsCodec.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/DiagnosticsCodec.cpp) | 診断と設定読み出し19種類の型と公開API |
| [MotorFramePlan.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/MotorFramePlan.hpp) | [MotorFramePlan.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/MotorFramePlan.cpp) | SIL専用Type1×4＋C620共通0x200の符号化計画 |
| [PowerBoardCodec.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/PowerBoardCodec.hpp) | [PowerBoardCodec.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/PowerBoardCodec.cpp) | 電源状態・要求・応答と非常停止の公開API |
| [ProtocolId.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/ProtocolId.hpp) | [ProtocolId.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/ProtocolId.cpp) | CAN2 v1のID構造と固定許可リストの公開API |
| [ProtocolPayload.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/ProtocolPayload.hpp) | [ProtocolPayload.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/ProtocolPayload.cpp) | Big endianの整数・float32変換API |
| [RemoteControlCodec.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/can/RemoteControlCodec.hpp) | [RemoteControlCodec.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/can/RemoteControlCodec.cpp) | 運動指令・操作要求・応答の公開API |

### communication/usb/ — USB HID報告・descriptor

| 宣言 | 実装 | 責務 |
|---|---|---|
| [DualSenseProtocol.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/communication/usb/DualSenseProtocol.hpp) | [DualSenseProtocol.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/communication/usb/DualSenseProtocol.cpp) | 非Boot HID descriptor・操作報告の型と検査 |

### device/ — HAL接続・個別機器

| 宣言 | 実装 | 責務 |
|---|---|---|
| [Bno055.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/Bno055.hpp) | [Bno055.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/Bno055.cpp) | ACCGYRO取得とDRDY・時間窓 |
| [C620Motor.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/C620Motor.hpp) | [C620Motor.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/C620Motor.cpp) | C620 1軸の受信順累積・連続性・鮮度のAPI |
| [F446UsbHostPort.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/F446UsbHostPort.hpp) | [f446_usb_host_port.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/f446_usb_host_port.cpp) | 基板設定と明示Host起動 |
| [F446Watchdog.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/F446Watchdog.hpp) | [F446Watchdog.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/F446Watchdog.cpp) | IWDG固定設定・起動診断とC ABI |
| [I2cPort.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/I2cPort.hpp) | [I2cPort.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/I2cPort.cpp) | 非同期I2C転送の契約 |
| [ImuObservation.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/ImuObservation.hpp) | [f446_imu_runtime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/f446_imu_runtime.cpp) | 姿勢の周期読取API |
| [IndicatorOutput.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/IndicatorOutput.hpp) | ヘッダー内 | GPIO極性、LED・RGB・ブザー出力 |
| [MotorObservation.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/MotorObservation.hpp) | [f446_can1_runtime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/f446_can1_runtime.cpp) | 6軸観測コピー・回収番号・故障保持・停止専用設定、原点切替境界 |
| [Rs06Motor.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/Rs06Motor.hpp) | [Rs06Motor.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/Rs06Motor.cpp) | RS06 1軸の有効化、停止、状態報告のAPI |
| [StatusIndicatorPort.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/StatusIndicatorPort.hpp) | [StatusIndicatorPort.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/StatusIndicatorPort.cpp) | RGB・ブザー出力と出力失敗診断 |
| [UsbHostPort.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/device/UsbHostPort.hpp) | [UsbHostPort.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/UsbHostPort.cpp) | 接続世代・転送連番・期限の契約 |
| — | [f446_indicator_output.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/f446_indicator_output.cpp) | 生成GPIO割当と出力ラッチ照合 |

### bench/ — 通常運転と独立した単軸試運転

| 宣言 | 実装 | 責務 |
|---|---|---|
| [MotorBench.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/bench/MotorBench.hpp) | [MotorBench.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/bench/MotorBench.cpp) | 単軸試運転のC++実装 |
| — | [f446_motor_bench_runtime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/bench/f446_motor_bench_runtime.cpp) | 試運転専用CANの初期化・フィルタ・周期・IRQ |

### core/ — 共通の小さな状態機械基底

| 宣言 | 実装 | 責務 |
|---|---|---|
| [StateMachine.hpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing/core/StateMachine.hpp) | ヘッダー内 | 状態機械で共用する型付きの状態保持 |

### Inc/ — 生成Cコードとの境界

| C ABI | 対応実装 | 役割 |
|---|---|---|
| [balancing_app_bridge.h](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing_app_bridge.h) | [application.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/app/application.cpp) | 生成Cコードからアプリに入る初期化・周期通知と周期診断API |
| [balancing_f446_bench_bridge.h](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing_f446_bench_bridge.h) | [f446_motor_bench_runtime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/bench/f446_motor_bench_runtime.cpp) | 試運転専用ビルドの起動・周期・IRQ入口 |
| [balancing_f446_can1_bridge.h](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing_f446_can1_bridge.h) | [f446_can1_runtime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/f446_can1_runtime.cpp) | CAN1/2の起動、IRQ取り込み、共通イベント回収のC API |
| [balancing_f446_imu_bridge.h](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Inc/balancing_f446_imu_bridge.h) | [f446_imu_runtime.cpp](https://github.com/kou-pen/balancing/blob/51bd7bf2e1c77c0c78fa027c3a438c49198fadd8/firmware/f446/balancing_f446/App/Src/balancing/device/f446_imu_runtime.cpp) | BNO055 I2C起動・背景回収・I2C/DRDYイベントのC ABI |

IWDGのC ABIはF446Watchdog.hppに同居。GPIOアダプタの宣言はStatusIndicatorPort.hppにあり、実装はf446_indicator_output.cppです。生成Coreを移動・編集する必要はありません。

## 4. App外のファイル

| パス | 何を調べるか |
|---|---|
| firmware/f446/balancing_f446/Core/Src/main.c | 初期化・周期の実行要求・主ループ |
| firmware/f446/balancing_f446/Core/Src/stm32f4xx_it.c | CAN、I2C、USBなどの割り込み入口 |
| firmware/f446/balancing_f446/balancing_f446.ioc | CubeMXの周辺回路・ピン・クロック |
| tests/sil/DriveRuntime.cpp、DriveSil.hpp | PCでだけ使う人工設定・フレーム生成境界・共通実行への接続 |
| tests/test_f446_stop_output.cpp | #88の実製品停止専用実装を模擬CANで検証 |
| tests/test_f446_bench_runtime.cpp | 独立試運転の起動・停止検証 |
| tools/run_host_tests.py | 通常/UBSanホスト試験 |
| tools/regenerate_cubeide_f446.py | ARM管理ビルド・ELF検査 |

## 5. 詳細設計

[データ所有権](data-model.md) ／ [観測](observation-snapshot.md) ／ [CAN送信調停](can-scheduler.md) ／ [モータ・試運転](motor-layer.md) ／ [制御数学](../software-core.md)。

整理前の全API設計記録は [Git履歴の固定版](https://github.com/kou-pen/balancing/blob/cb3abeda89acdbcb08fd9890735f2b8c95ed9d83/docs/design/public-api.md) に残しています。過去の受入・検証記録は [verification/](../verification/index.md) から参照できます。
