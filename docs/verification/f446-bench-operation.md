# F446 単軸試運転の明示操作接続

基準 master 26c03a5、実装 863f985、試験 67f5869。T-F446-11/12、BEHAV-001、BOARD-F446-001。App新規ファイル0（98ファイル、46 C++、旧C0）。

## 実装

MotorBench.hpp/.cppのpollに既定nullptrの操作関数を追加。旧送信終端・CAN受信・既存保護判定・停止確認の後、次の送信予約前に一度呼ぶ。C620の停止中ゼロ更新と新規操作の競合を回避する。f446_motor_bench_runtime.cppのserviceCanBenchで番号・時計を検査して操作を通す。欠落1回でも新規操作を通さず、既存6ms故障契約は変更しない。操作元はホスト試験のみ。開始と停止の同時操作は停止優先、拒否は再試行しない。既存requestPulse/cancelを利用する。

起動固定のモータID・出力上限・時間、0.5Nm/A・200ms、150ms、4/2ms、CAN wire、既存故障・停止・IWDG処理を変更しない。C ABIは操作なし固定、既定boardConfigは無効。物理入力・実機パラメータを設定しない。

## 検証

- CTest 79/79（7b3d400e155d4501）。既存76回帰＋RS06、C620、周期異常の操作接続3件。
- 起動前、停止未確認、未完了TX、NaN・上限超過を拒否。RS06 Type4送達後のReset確認、Type3送達後の新規Running確認、短時間出力と停止。C620は選択チャネルだけの模擬出力。停止優先、旧終端待ち、自動再始動なし、時間満了、欠落番号・重複番号、バス異常/RX欠落、非選択バス無送信を確認。従来の単体試験で鮮度・送達期限・有効化期限を検証。
- 実機操作、通電、物理停止、出力の安全認定、WCET/スタックは未実施。実機操作経路は未接続。

通常・UBSan全体回帰（85e7effb59704c68）PASS。ARM管理cleanBuild（af72b0fac18e4ebc）Debug/Release/BenchDebug各46 App翻訳単位、0 error/0 warning。nmでserviceCanBenchはBenchDebugのみ。全ELFに`MotorBench::requestPulse`、consumeOperation、旧DriveCycle、initializeDriveSil、`MotorFramePlan::compose` なし。Debug text226988/data2828/bss13352 B、Release text79020/data2828/bss13352 B（#94と同じ）、BenchDebug text83964/data2748/bss14336 B（text+152 B、RAM不変）。仕様530参照、CAN監査19/19、CubeIDE設定24/24、Doxygen全98ソースリンク、diff check合格。
