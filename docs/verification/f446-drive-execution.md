# F446 通常運転とSILの駆動実行共通化

基準 master eae21b4、実装 51bd7bf、試験 738d940。BOARD-F446-001、BEHAV-001、T-F446-04/08〜10、T-CTRL-11。旧tests/sil/DriveCycle.hpp/.cppをAppのDriveExecution.hpp/.cppへ移設・改修。App98ファイル、46 C++、旧C0。ソースファイル総数は不変。

## 実装

DriveExecutionは既存ControlCycleを一周期に一度だけ実行し、MotorServiceへの状態要求とMotorPeriodicTxの周期予約・停止・送信終端を接続。通常DriveRuntimeは停止専用の明示設定に限り構築し、生成関数なし・期限なしとする。番号付き新規観測・連続番号・単調時刻条件を維持し、StopAll、RS06停止、C620全ゼロのみ。未設定は送信なし。SILアダプタだけが人工設定、左右校正、移動目標、独立期限とMotorFramePlanの生成関数を渡す。生成関数は起動時固定。fixtureOnlyとRobotSequenceの遷移は変更しない。既存の4ms/2ms予約、旧指令の中止、CAN wire、安全閾値、IWDG、BenchDebugを維持する。非ゼロ実機出力を有効化しない。

## 検証

- CTest 76/76（1cb33be1cded453d）。既存74回帰と独立RS06/C620送達期限ゼロによるARM拒否2件。既定送信なし、明示停止のみ、欠落・重複、SILのARMING→4軸確認→6軸駆動、DISARM/FAULT、送信期限、原点同期、IWDG、BenchDebugを確認。
- 境界監査で通常/SILが同じDriveExecutionを使い、通常構築に生成関数を渡さないことを検査。ホスト専用ヘッダのARMコンパイル拒否はDriveSil.hppに移設。
- 実機モデル、駆動許可、物理安全、WCET・スタックは未検証。追加診断通信や実機操作は行わない。

通常・UBSan全体回帰（2bd35626ade347fc）PASS。最終追加2ケースはCTestビルドとUBSanバイナリで個別再実行し4/4 PASS（いずれも独立期限不成立でARM拒否の出力を確認）。最終ARM管理cleanBuild（eb05b6c1efb641a2）Debug/Release/BenchDebug、各46 App翻訳単位、0 error/0 warning。nmで通常ELFに`DriveExecution::step`、BenchDebugにはなし。全ELFに旧DriveCycle、initializeDriveSil、driveSilObservation、composeFixture、`MotorFramePlan::compose`、`MotorBench::requestPulse` なし。Debug text226988/data2828/bss13352 B、Release text79020/data2828/bss13352 B（#92基準+1832/-16 B）、BenchDebug text83812/data2748/bss14336 B。共通駆動経路のコードは通常ELFにも含まれるが、通常構築時の生成関数はnullであり非ゼロ出力入口は未接続。仕様527参照、CAN監査19/19、CubeIDE設定24/24、Doxygen全98ソースリンク、diff check合格。
