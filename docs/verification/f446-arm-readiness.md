# F446 ARM準備診断

基準 master f9f7587、実装 f56c72c、テスト 57be013。BEHAV-001、BOARD-F446-001、CTRL-EST-001、CTRL-LQR-001。既定のARM禁止・停止専用出力・SIL隔離を維持する。

## 診断の所在

- ApplicationObservation.hpp: ArmReadinessIssue（内部診断ビット、CAN wireではない）とArmReadinessSummary。
- DriveRuntime.hpp: inspectArmConfiguration、inspectArmMotorConfiguration、inspectArmObservationsの読み取り専用API。
- application.cpp: 数学設定を起動時に検査、成功したCAN起動の設定検査結果を保持、周期終了後に観測不足を更新。SILも同じ診断コードをリンクするが、診断は実機準備を表す。
- 既存LegKinematics / WheelOdometry / MotorCommand / C620Motor / MotorServiceの形式検査を再利用。ゲインは既存演算に局所的なゼロ誤差を渡す。実運転の積分器や状態には触れない。数値形式の検査を物理妥当性の確認と混同しない。

configurationはARM期限、左右の脚校正・数学幾何、左右車輪校正、LQR・脚PD・Yaw PI・車輪トルク上限、6軸個別変換、RS06割当、左右車輪連続性、CAN2設定を区別する。実機駆動未接続ビットを常に残す。observationは周期観測の成立、IMU、6軸、電源、推定、機構安全の未検証、FAULTと非DISARMEDを区別する。観測がない周期は前回の正常値を再利用しない。実際のARM許可とFAULT遷移は既存RobotSequenceが所有する。診断ビットを新しいインターロックとして使わない。

## 検証

tests/test_f446_stop_output.cpp --readiness で既定の全不足、人工値の形式成立、各設定の単独不成立（左右別・6軸別、NaN・不正値・重複ID・取得間隔）、観測不足・FAULTを検査する。人工値が成立しても実機駆動未接続と機構安全未検証の診断は消えない。既存停止経路の試験ではCAN起動後に軸割当ビットが更新されることを追加確認。

CTest 71/71 PASS（21c302abf3be4a5d）。実機操作なし。ARM拒否の原因を診断として取得可能にしたが、Piへの新規CAN診断配信、実機駆動、機構保護モデルは未実装。

通常・UBSan全件PASS（4a0775cc49364a99）。CubeIDE Debug/Release/BenchDebug cleanBuild（a6142b6f599e4c36）全構成0 errors/0 warnings、45 Appソース。Release text76108/data2828/bss12600 B（基準よりtext+756 B、bss+16 B）。Debug text219572/data2828/bss12600 B、BenchDebug text83732/data2748/bss14288 B。nmでARM準備診断は通常ELFのみ、全構成にSIL/試運転パルス/MotorFramePlan::composeなし。仕様493 API参照、メタデータ24/24、CAN監査19/19、Doxygenソースリンク・diff check合格。App96ファイル、C++45、旧C0を維持。
