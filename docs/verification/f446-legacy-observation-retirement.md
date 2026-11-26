# 旧C11状態機械・観測ストアの廃止

基準 #80 dc9b4e6（先行PRのマージ後にmasterを基準とする）。BEHAV-001 / CTRL-EST-001 / BOARD-F446-001。

旧BModeMachineと観測ストアは製品コードから未使用。状態機械・鮮度判断は現行C++に一本化する。balancing_core.h/.c内の旧状態・鮮度関数を削除し、LQRと車輪配分の数学関数を維持する。balancing_observation.h/.c、balancing_observation_time.hを廃止。BImuSampleは既存balancing_attitude_filter.hに移設し、C姿勢フィルタとBno055の共通型を保つ（次の姿勢フィルタ移行で再整理）。新規ファイルは追加しない。

## 移行時の契約差

旧C11の姿勢ゲート、3状態、送信受付と完了の混同、鮮度期限のinclusive境界は現行C++へ移植しない。現行RobotSequence、Rs06Motor、C620Motor、WheelOdometry、ImuAttitudeSource、StateEstimator、PowerBoardService、ControlCycleの挙動を変更しない。現行の機器鮮度期限はIMU 50ms、RS06 30ms、車輪20ms、電源500ms（期限ちょうどは無効）。操作入力は別契約。安全閾値と物理実機条件は今回変更なし。

## 試験移行

| 旧試験 | 現行の検証 |
|---|---|
| test_core.cのARM・DISARM・FAULT・鮮度 | test_robot_sequence.cppの4軸確認・セッション・期限・明示解除・故障優先、追加の6個別インターロック喪失。実CAN経路の--drive-sil-gap/--drive-sil-rx-loss/--watchdog。数学試験はtest_core.cに残す |
| test_observation.c | test_f446_imu_runtime.cppの実際の姿勢取得・元時刻・非新規/未来/異常入力・期限境界、test_f446_wheel_odometry.cppの受信順・半回転・欠落・原点世代・左右符号・新規追加の単軸受信と対称折返し |
| test_issue10_flow.c | test_f446_state_estimator.cppの実姿勢フィルタ＋6軸観測の周期コピー、test_f446_remote_control_service.cppの現行RobotSequenceとの非自動ARM、模擬CAN/USBのApplication全経路 |
| spec_gap_core_mode.c | 当時の既知不一致再現用診断。製品の試験対象外。Git履歴に保存 |

旧状態の内部フラグを検査する目的はなくなったため、同じテスト名の互換実装は作らない。CTestの2件減は旧観測・旧結合試験を現行の試験内へ集約したため。新規テストでは現在の公開APIの状態、タイムスタンプ、停止要求を確認する。

## 検証
App 105→102、Cソース7→6。CTest 48/48（8da7d9039f1a40be）。通常/UBSan全件PASS（16c3f401db6c4d3b）、CubeIDE Debug/Release全48 Appソース0 errors/0 warnings（ceb2e6271bb14469）。Release text=69656/data=2828/bss=12216 Bで#80と同一。仕様455参照、Doxygen全sourceリンク、CubeIDE設定、diff check PASS。旧関数・型・ファイル参照なし、両ELFの旧関数/SIL/試運転シンボルなし、残したLQR/配分コードは#80とバイト単位で同一。実機通電・書込みなし。
