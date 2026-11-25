# Applicationの観測周期と統合SIL

仕様：BOARD-F446-001、CTRL-EST-001、CTRL-LQR-001、BEHAV-001。Issue #27 バッチC。依存：CAN観測 #54、IMU #55。

## 本番の読み取り専用経路

application.cpp はMotorObservationSnapshotを回収時刻付きで取得し、同じ時刻でImuAttitudeを評価する。ApplicationがStateEstimator、RobotSequence、BalanceControl、MotorCommand、ControlCycleの寿命を所有し、今周期の入力を固定して`ControlCycle::step()`を一度実行する。初回の実測経過時間が未成立のときはInvalidPeriod。観測取得不能時は周期差分を破棄し、非駆動のControlCycleの時刻基準だけを再構築する。重複・逆行では旧周期結果を再公開しない。車輪の原点、RobotSequenceのFAULT履歴は保持する。周期処理後、最新回収番号を指定してCAN観測の故障通知を消費する。消費はRobotSequenceのFAULT解除ではない。

起動時の機構・校正・ゲイン・駆動制限・ARM期限設定は未検証の既定値。OperatorEventsは空、電源・機構条件は未検証。観測から実機送信を開始しない。診断コピーをApplicationObservation.hppから取得できる。未取得/時刻不正周期はcycle空。`MotorService::applySequence()`、MotorFramePlan、CAN送信要求は本番Applicationから未接続。

## 統合ホスト試験

test_cpp_app.cppとtest_f446_can1_runtime.cppで実Applicationの初回・通常・重複・逆行・取得不能後の周期、未検証設定、CAN異常のFAULTラッチと旧プレビュー抑止を確認した。

test_f446_can_event_poll.cppの`testIntegratedObservationAndFrameScheduling()`は模擬FIFO→Router/Link→MotorServiceと、人工の時刻付きIMU報告→ImuAttitudeSourceを同時に入力し、既存StateEstimator/ControlCycle/RobotSequenceからMotorCommand/MotorFramePlanへ結合する。模擬RS06の起動時停止・ARM要求→送信受付→送信終端→新規報告でBALANCINGを確認。100周期で車輪100世代、脚50世代を2/4msごとに生成し、通常受付済みは車輪99件・脚196件。残り未受付の車輪1件と脚4件を次のE-STOP周期に破棄し、CAN1 Type4×4とCAN2 0x200ゼロを模擬メールボックスへ投入する。生成周期は試験の仮想時計が所有し、Schedulerの期限管理と区別する。既存別試験で6ms周期欠落、受信欠落、BusOff、IMU鮮度、故障後の旧指令抑止を検証。実機の2/4ms送信スケジュールはまだ接続していない。

模擬試験はI2Cレジスタの物理取得から実機Applicationの正常ARMまでの一本の試験ではない。IMUレジスタ取得は#55の専用試験、実Applicationは既定非駆動構成、正常ARMと非ゼロフレームは人工値のSILに限る。

## 未検証

ARMクロスビルド、実機周期とジッタ、実機CAN/I2C、IMU静止証明、RS06軸ID・主機ID、C620取得間隔、CAD幾何・実機ゲイン、F303・PS5のプロトコル、IWDG、物理非常停止、実機送信完了監視は残件。CAN通常モードのACK等は発生し得るが、実機指令送信を追加していない。ホストPASSは物理安全の証明ではない。

CTest 25/25 PASS。
