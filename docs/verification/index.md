# 検証記録の案内

**PASSは、その条件で実際に検査した範囲だけを意味します。** ホスト試験、ARMクロスビルド、実機動作を同じ「完成」として扱いません。

## どの結果を見ればよいか

| 目的 | 記録 | 確認できた範囲 |
|---|---|---|
| **2026-11-12 移植前master基準** | [C++17集中移植の事前検証](cpp17-migration-preflight-20261112.md) | PR #24/#25/#26統合後、ホスト通常/UBSan・CTest 12/12・CubeIDE Debug/Release PASS。MOTOR-151/198の既知仕様差分を再現（実機未実施） |
| C++17のARM・FAULT状態機械 | [RobotSequenceのホスト検証](robot-sequence-host.md) | 仮想時計、RS06 4軸の有効化結果、期限境界、異常記録 |
| F446 CAN1の起動と割込み | [CAN1の起動・割込みの検証](f446-can1-runtime-host.md) | フィルタ、TX／RX0／SCE、DWT時計、受信観測 |
| F446 CANイベント回収・配送 | [2バスの回収と配送](f446-can-event-dispatch-host.md) | 1バス最大19件、RS06・C620の受信振り分け、バスごとのTX終端照合、周期フレームの置換・期限、通常イベントFIFO |
| C620車輪のC++17位置・速度換算 | [車輪オドメトリのホスト検証](f446-wheel-odometry-host.md) | CAN2生報告から左右の相対位置・周速度への変換、二重積算の防止、異常時刻・曖昧半回転・鮮度の境界 |
| IMU・6軸のC++17状態スナップショット | [StateEstimatorのホスト検証](f446-state-estimator-host.md) | 姿勢・左右脚数学fixture・左右車輪の同一周期結合、不成立時の出力禁止 |
| SIL 500 Hz状態遷移とC++17制御合成 | [SIL制御周期のホスト検証](f446-sil-control-cycle-host.md) | LQR→脚PD/Roll→VMCと倒立優先Yaw配分、100周期、6ms欠落、FAULT/再ARM防止 |
| MotorCommandの6軸最終換算・上限 | [MotorCommand数学プレビュー](f446-motor-command-host.md) | 4軸RS06 Nm、2軸C620電流Aの換算と上限、全ゼロ棄却、SILのARM・FAULT結合 |
| 6軸CAN指令計画（SIL） | [CAN符号化・停止優先のホスト検証](f446-motor-frame-plan-host.md) | Type1×4と0x200×1の実コーデック照合、停止優先・旧指令不復活・ゼロ電流 |
| ControlCycleのC++17本体化 | [時刻付き制御周期のホスト検証](f446-control-cycle-core-host.md) | 旧テスト専用実行器の本体移動、実測dt、稼働中更新欠落・故障時演算抑止 |
| Applicationの周期診断 | [共通DWT時計とApplication](f446-application-timing-host.md) | CAN1受信時刻・実測周期、周回、未採取・重複・逆行 |
| CAN1/2の共通受信ランタイム | [両CANの起動・共通時計](f446-dual-can-runtime-host.md) | 共有フィルタ、バス別IRQ・異常、復帰不能時の起動拒否 |
| C620の受信順累積・連続性 | [C620累積処理のホスト検証](f446-c620-continuity-host.md) | 複数報告、同一IRQ時刻、受信喪失、原点世代、SIL回帰 |
| F446 CAN1の送受信と軸別トークン | [CAN1ポートとRS06の接続検証](f446-can1-rs06-port-host.md) | メールボックス再利用、受信あふれ、バスオフ、4軸ARMの結合 |
| RS06 4軸の有効化と停止 | [MotorServiceの模擬CAN検証](motor-service-rs06-host.md) | Type 3完了と新規応答の照合、4軸別のType 4再試行、バスオフ・期限の故障注入 |
| F446の11ソース構成 | [Issue #9脚長・Rollホスト検証](issue9-leg-pd-host.md) | Debug/Releaseで新しい脚長PDソースを含むARMクロスビルドPASS（実機未実施） |
| F446の直前10ソース構成 | [Issue #10ホスト・ARM検証](issue10-observation-host.md) | PR #23時点の10ソース、実機未実施 |
| CAN周期と送信調停 | [Issue #5ホスト試験](issue5-can-scheduler-host.md) | 仮想時刻・3メールボックス・故障注入 |
| 単軸試運転の受信期限 | [CANポート集約](motorbench-rx-owner.md) | ベンチの重複期限除去とポート境界の故障注入 |
| CAN仕様の整合とID重複 | [Issue #6 CAN静的照合](can-contract-audit.md) | RS06/C620の既知フレーム、F303/Piの未確定項目 |
| ARM・FAULT判定の差分 | [状態機械の監査](core-mode-contract-audit.md) | ホスト回帰PASSと既知の仕様不一致 |
| 脚運動学の数値演算 | [Issue #7数学fixture](issue7-kinematics-host.md) | 解析ヤコビアンと有限差分、実機CAD未検証 |
| 脚長保持・Roll差動 | [Issue #9ホスト検証](issue9-leg-pd-host.md) | ARM前ラッチ、PDの単位・符号、異常入力拒否 |
| センサ・操作入力・IMUフィルタ | [Issue #10ホスト検証](issue10-observation-host.md) | C620折返し、IMU静止バイアス、操作元・通信断復帰 |
| 過去のCubeIDE配置 | [5ソース配置の検証](f446-source-layout.md) | PR #16時点の構成と再生成結果 |
| 初期のF446クロスビルド | [2026-11-09の記録](f446-crossbuild-20261109.md) | 統合前のC++骨組み |

移植開始の判断には、一番上の**2026-11-12統合master基準**の検証記録を使います。過去のビルド記録は当時の事実として残し、現在の検証結果へ読み替えません。

## 検証状態の区別

| 表記 | 意味 |
|---|---|
| PASS | 記載された環境と対象で試験成功 |
| FAIL | 試験実行後に失敗 |
| NOT RUN / NOT VERIFIED | 未実施、または合否に必要な証拠がない |

## 現時点の境界

通常運転SILレビュー版: [Application通常運転・6軸模擬CAN](f446-drive-cycle-sil.md)は追加10モード、CTest 44/44。実機への通常指令はコンパイル境界で分離。

2026-11-26 CAN2統合レビュー版: [非駆動Application配送・診断/操作応答](f446-can2-service-integration-host.md)でCTest 34/34、通常/UBSan、全46 AppソースのDebug/Release ARMビルドが合格。通常モータ駆動と実機試験は未実施。

2026-11-26、PR #68統合時点の最新ホスト結果はCTest 33/33、通常/UBSan全件合格。CAN2 v1の受信分類・フィルタ計画・電源状態受信、非駆動Application周期、BNO055、6軸観測、SIL制御・指令計画を検証済み。現行master 0692c4eの[全44 AppソースのARMクロスビルド](f446-crossbuild-20261126.md)はDebug/Releaseとも合格。本番操作・送信経路の統合と実機試験は未確認。進行順は [P0ロードマップ](../todo/roadmap.md) を参照。

制御コア・CAN通信・送信要求モデル、運動学fixture、脚長PD/Roll PDの**ホスト試験**、F446の**11ソースCubeIDE Debug/Release管理ビルド**は2026-11-11にPASSしました。一方、実機書込み、CAN実通信、トルク出力、実時間性能、独立非常停止、実機での倒立は未検証です。

次の試験を計画するときは [実機検証TODO](../todo/verification.md) の受入条件を参照し、作業上の最新状態はGitHub Issue/PRで確認します。

- [CANランタイムの6軸観測接続](f446-runtime-motor-observation-host.md)

- [BNO055非同期取得と姿勢フィルタ](f446-bno055-observation-host.md)

- [Application観測周期・統合SIL](f446-application-observation-cycle-host.md)

- [F303電源の論理観測](f446-power-service-host.md)

- [操作入力C++境界](f446-remote-service-host.md)

- [診断イベント管理](f446-diagnostics-service-host.md)

- [LED/Buzzer論理表示](f446-status-indicator-host.md)

- [CAN2 v1共通コーデック](f446-protocol-codec-host.md)

- [F303電源通信コーデック](f446-power-board-codec-host.md)

- [Raspberry Pi操作通信コーデック](f446-remote-control-codec-host.md)

- [診断・設定読み出しコーデック](f446-diagnostics-codec-host.md)

- [CAN2自作通信の受信分類](f446-can2-rx-routing-host.md)

- [CAN2受信フィルタ](f446-can2-filter-plan-host.md)

- [F303電源状態の受信境界](f446-power-status-receive-host.md)

- [CAN2通信の非駆動統合](f446-can2-service-integration-host.md)

- [Application実行境界の分離](f446-application-runtime-refactor.md)（ホスト・ARMビルド検証済み）

- [DualSense USB入力境界と操作統合](f446-dualsense-input-host.md)
- [F446 USB Host転送とHCD接続](f446-usb-host-transport.md)

- [RGB・ブザー出力境界と表示診断](f446-indicator-output-host.md)

- [制御周期完了監視とIWDG更新境界](f446-cycle-watchdog-host.md)

- [F446小規模ファイル統合](f446-file-consolidation.md)

- [旧C11送信モデルのC++統合](f446-c-scheduler-migration.md)

- [独立単軸試運転のC++統合](f446-motor-bench-cpp.md)

- [操作入力のC++統合](f446-operator-cpp.md)

- [旧C状態機械・観測ストアの廃止](f446-legacy-observation-retirement.md)

- [モータ通信コーデックのC++統合](f446-motor-codecs-cpp.md)

- [BalanceControlのC++数値演算統合](f446-balance-math-cpp.md)

- [姿勢フィルタのC++統合](f446-attitude-math-cpp.md)
