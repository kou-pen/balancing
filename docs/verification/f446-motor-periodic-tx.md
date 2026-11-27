# 通常運転とSILの周期送信管理共通化

基準 master bf215bf、実装 8e62989、試験 fdbe765。設計承認済み。BOARD-F446-001、BEHAV-001、PROTO-RS06-001、PROTO-C620-001。App新規0本（96ファイル、C++45、旧C0）。

## 責務

MotorCanLink.hpp/.cpp内のMotorPeriodicTxはCAN1 4対象、CAN2 1対象の周期送信枠を単一所有し、両バスの既存CanTxSchedulerを借用する。バス別世代、発行元とserial、予約・受付・終端・欠落の累計、受付時刻と送達鮮度を管理する。通常DriveRuntimeは新規観測周期・StopAll・ゼロ予約だけを管理。tests/sil/DriveCycleは状態機械の結果、4ms位相、人工モデルのフレーム生成、明示期限を管理し、同じ共通送信処理を呼ぶ。実機起動、ARM許可、非ゼロ生成の入口は追加しない。

RS06 Type3/4は既存MotorCanLinkとMotorServiceが管理し、周期送信共通化に移さない。ワイヤID、0xD000（SIL）/0xD001（通常停止）内部origin、4ms/2ms予約期限、既存SIL期限は不変。実機送達期限は設定しない。

## 世代・終端の扱い

新世代は旧未受付分を置換し、Invalid/StaleGenerationの場合は旧予約を保持。Schedulerは受付済み旧世代を終端まで保持する。共通層は受付時刻より古い終端、別バス・別origin・旧tokenを拒否。最新世代の正常終端だけが駆動鮮度を回復する。予約時から停止用ゼロと駆動指令を区別する。駆動監視はbeginActiveの明示期限のみ。期限ちょうどの終端も鮮度回復不可。

withdrawは未受付を破棄し、受付済みに中止を要求する。中止終端は診断累計に残すが駆動鮮度から切り離す。停止用ゼロの成功は通常駆動の証跡ではない。通常停止側は従来の外部故障通知を維持する。SILの上位運転判断とRobotSequence、fixtureOnlyは変更しない。受付と終端は物理停止を証明しない。

## 試験

既存CANイベント試験内にtestSharedMotorPeriodicTxを追加。直接の模擬CanPort/Router/Schedulerで3枠→4対象、ゼロと駆動の区別、誤バス・誤origin・受付前時刻、旧世代の終端、期限直前/一致、停止時の中止終端と停止枠解放を検証。既存の通常停止/SIL統合試験を両方保持し、IWDG・BenchDebug・ARM準備診断も回帰。

- 最終CTest 71/71（401aa757148640ac）。追加ケースは既存実行ファイル内のためCTest数は同じ。
- ARM管理cleanBuild（59226701502141ac）3構成、45 App翻訳単位、全構成0 errors/0 warnings。nmで通常ELFに`MotorPeriodicTx::queueZero`、BenchDebugにはなし。全構成にSIL DriveCycle、`MotorFramePlan::compose`、`MotorBench::requestPulse` なし。
- Debug text222228/data2828/bss13336 B、Release text77156/data2828/bss13336 B、BenchDebug text83732/data2748/bss14288 B。Releaseは基準よりtext+1048 B、bss+736 B（5対象の共通状態・診断領域）。BenchDebugのサイズは不変。実機WCET/スタックは未測定。

実機設定、CADモデル、機構安全、通常ARM・非ゼロ駆動接続、書込み・通電は未実施。送信管理の共通化のみを完成扱いとする。

最終通常・UBSan全件PASS（c6668d7dafe14c14）、CAN契約監査19/19、CubeIDE設定24/24、仕様507 API参照、Doxygen生成・全96ソースリンク・diff check合格。既存ARM準備診断文書のDoxygen自動リンク誤認をコード表記に修正。
