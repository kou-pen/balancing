# 検証記録

各記録は当時のコード・環境・試験結果です。最新の実装説明は[全体構成](../architecture.md)、[フォルダ別コード案内](../design/public-api.md)に置き、過去のPASSを現在の実機動作に読み替えません。

## 最新の基準（起動処理統合レビュー版 e1e279a）

| 項目 | 結果 |
|---|---|
| App自作コード | 98ファイル、46 .cpp、旧 .c 0 |
| CTest・通常/UBSan | 79/79・全件PASS |
| ARM管理ビルド | Debug / Release / BenchDebug すべて0 error / 0 warning |
| 実機 | 書込み、実通信、実時間、物理停止、倒立は未検証 |

出典：[F446起動処理統合](f446-startup-integration.md)。レビュー版とmasterを区別する。#88〜#95はmaster反映済み。

## F303電源基板

- [PowerControllerの論理境界試験](f303-power-controller-host.md)（実機未接続）
- [PowerMonitorの換算・保護とController結合試験](f303-power-monitor-host.md)（人工fixture）
- [PowerCanの要求処理とF446対向試験](f303-power-can-host.md)（ホストフレーム接続）
- [BoardIOの論理出力境界・LED表示](f303-board-io-host.md)（実GPIO未接続）
- [CubeF3開発環境とARMリンク検証](f303-cube-environment.md)（試験ELFのみ・書き込み禁止）

## 機能別の記録

### 最新の統合・起動・停止

- [F446起動処理統合と初期化失敗伝達](f446-startup-integration.md)
- [F446 AppのC++移行統合検証](f446-all-cpp.md)
- [通常運転の停止専用CAN送信経路](f446-stop-output.md)
- [F446 IWDG実機用アダプタ](f446-iwdg-adapter.md)
- [単軸試運転の明示操作接続](f446-bench-operation.md)
- [BenchDebug専用起動とCAN IRQの排他検証](f446-bench-startup.md)
- [Application実行境界の分離](f446-application-runtime-refactor.md)
- [F446 Application通常運転のSIL統合](f446-drive-cycle-sil.md)
- [F446 PR #68統合版のARMクロスビルド](f446-crossbuild-20261126.md)
- [F446の小規模ファイル統合](f446-file-consolidation.md)

### 周期・状態機械・安全・診断

- [F446 ARM準備診断](f446-arm-readiness.md)
- [RobotSequenceのホスト検証](robot-sequence-host.md)
- [F446 ControlCycleのC++17本体化・ホスト検証](f446-control-cycle-core-host.md)
- [F446 Applicationの共通時計・周期診断](f446-application-timing-host.md)
- [Applicationの観測周期と統合SIL](f446-application-observation-cycle-host.md)
- [制御周期の完了監視とIWDG更新境界](f446-cycle-watchdog-host.md)
- [診断イベント管理（論理層）](f446-diagnostics-service-host.md)
- [F446 RGB・ブザー出力境界](f446-indicator-output-host.md)
- [LED/Buzzer論理表示と非ブロッキング道具](f446-status-indicator-host.md)
- [旧C11状態機械・観測ストアの廃止](f446-legacy-observation-retirement.md)

### CAN・モータ・試運転

- [通常運転とSILの周期送信管理共通化](f446-motor-periodic-tx.md)
- [F446 CANイベント回収・配送の検証](f446-can-event-dispatch-host.md)
- [F446 CAN1/CAN2共通受信ランタイムのホスト検証](f446-dual-can-runtime-host.md)
- [F446 CAN1の起動・割込みの検証](f446-can1-runtime-host.md)
- [F446 共通CANポートとRS06のホスト検証](f446-can1-rs06-port-host.md)
- [CANランタイムの6軸観測接続](f446-runtime-motor-observation-host.md)
- [モータ通信コーデックのC++統合](f446-motor-codecs-cpp.md)
- [RS06 4軸の模擬CAN検証](motor-service-rs06-host.md)
- [F446 MotorCommand 6軸変換・飽和のSIL検証](f446-motor-command-host.md)
- [6軸CANフレーム構築・2バス優先順位（SIL）](f446-motor-frame-plan-host.md)
- [旧C11送信モデルのC++実装への統合](f446-c-scheduler-migration.md)
- [独立単軸試運転のC++統合](f446-motor-bench-cpp.md)
- [単軸試運転：RX期限をCANポートへ集約](motorbench-rx-owner.md)
- [C620受信順の累積位置と連続性の検証](f446-c620-continuity-host.md)

### 推定・制御数学

- [F446 通常とSILの駆動実行共通化](f446-drive-execution.md)
- [F446 車輪原点再設定の統合](f446-wheel-reference.md)
- [F446 6軸・IMU同一周期StateEstimatorのホスト検証](f446-state-estimator-host.md)
- [F446 C++17 車輪オドメトリのホスト検証](f446-wheel-odometry-host.md)
- [BNO055非同期取得・姿勢フィルタ接続](f446-bno055-observation-host.md)
- [F446 C++17 BalanceControl・500 Hz SIL周期結合の検証](f446-sil-control-cycle-host.md)
- [BalanceControlのC++数値演算統合](f446-balance-math-cpp.md)
- [脚運動学・VMCのC++統合](f446-leg-math-cpp.md)
- [姿勢フィルタのC++統合](f446-attitude-math-cpp.md)

### 操作・電源・USB・CAN2

- [操作入力のC++統合](f446-operator-cpp.md)
- [復号済み操作入力のC++17境界](f446-remote-service-host.md)
- [F446 DualSense USB入力境界](f446-dualsense-input-host.md)
- [F446 USB Host転送とHCD接続](f446-usb-host-transport.md)
- [F303電源の論理観測](f446-power-service-host.md)
- [F446 CAN2 電源状態の受信境界（ホスト検証）](f446-power-status-receive-host.md)
- [F446 CAN2通信の非駆動統合](f446-can2-service-integration-host.md)
- [CAN2自作通信の受信分類ホスト検証](f446-can2-rx-routing-host.md)
- [CAN2受信フィルタのホスト検証](f446-can2-filter-plan-host.md)
- [CAN2 v1共通コーデックのホスト検証](f446-protocol-codec-host.md)
- [F303電源通信コーデックのホスト検証](f446-power-board-codec-host.md)
- [Raspberry Pi操作通信コーデックのホスト検証](f446-remote-control-codec-host.md)
- [診断・設定読み出しコーデックのホスト検証](f446-diagnostics-codec-host.md)

### 初期設計・移行前・過去の監査（履歴）

- [CAN通信契約の静的照合（Issue #6）](can-contract-audit.md)
- [制御コア状態機械の仕様差分監査](core-mode-contract-audit.md)
- [C++17集中移植前のmaster基準検証（2026-11-12）](cpp17-migration-preflight-20261112.md)
- [STM32F446ヘッドレスクロスビルドの検証記録](f446-crossbuild-20261109.md)
- [F446自作C11/C++ファイルの機能別一括配置（構造変更PR）](f446-module-layout.md)
- [F446ソース配置統合と検証手順](f446-source-layout.md)
- [Issue #10：センサ・操作入力のホスト検証](issue10-observation-host.md)
- [Issue #5：CANスケジューラの仮想時刻ホスト検証](issue5-can-scheduler-host.md)
- [Issue #7：脚運動学の数学fixture・ホスト検証](issue7-kinematics-host.md)
- [Issue #9：固定脚長PD・Roll PDホスト検証](issue9-leg-pd-host.md)
- [Issue #9：Yaw PIホスト検証](issue9-yaw-pi-host.md)

## 合否の意味

PASSは記録された条件で試験が成功したこと、FAILは実行後の失敗、NOT RUN / NOT VERIFIEDは未実施・証拠不足です。ホスト・ARMビルド・実機を別々に記録します。[実機受入条件](../todo/verification.md)は変更していません。

- [ガスシリンダ補償IF](f446-gas-cylinder-compensation.md)：VMC後の補償と数値失敗時の停止（Issue #9）。
