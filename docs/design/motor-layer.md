# モータ制御レイヤーと単軸試運転

**更新日：** 2026-11-27。対象仕様：[RS06](../specs/protocols/rs06_can1.md)、[C620](../specs/protocols/c620_can2.md)、[F446](../specs/boards/f446_control.md)、[安全・操作](../specs/behavior/robot_operation.md)。

## 通常運転と単軸試運転

通常運転はApplication / RobotSequence / MotorServiceが所有する。単軸試運転は独立したMotorBenchが所有し、実行中に両者を切り替えない。通信の符号化、機器の受信・送信完了管理、CANポートは共通化している。現在の通常起動は既定で非駆動観測。明示設定で停止専用送信に接続できるが、通常運転・試運転とも非ゼロ要求を実機起動へ接続していない。

| App内のファイル | 役割 |
|---|---|
| `communication/Rs06Codec.hpp/.cpp`、`C620Codec.hpp/.cpp` | CAN指令とフィードバックの符号化・復号 |
| `device/Rs06Motor.hpp/.cpp`、`C620Motor.hpp/.cpp` | 機器ごとの状態・鮮度・送信終端 |
| `communication/can/CanPort.hpp/.cpp` | CANレジスタ、メールボックス、IRQと共通DWT時計 |
| `communication/can/CanTxScheduler.hpp/.cpp`、`CanRouter.hpp/.cpp` | 送信調停と回収イベントの配送 |
| `bench/MotorBench.hpp/.cpp` | 単軸試運転状態、停止処理 |
| `bench/f446_motor_bench_runtime.cpp` | BenchDebug専用のCAN起動、周期処理、IRQ配送 |
| `App/Inc/balancing_f446_bench_bridge.h` | 生成Cコードと試運転ランタイムの境界 |

## 起動と停止確認

CubeIDEのBenchDebugは通常Debug/Releaseとは別のELFを生成する。モードはコンパイル時固定。通常Application、USB・IMUのApplication入力処理を起動しない。CAN1/2の生成初期化は残すが、HAL_CAN_Startと通知有効化は明示設定により選択した1バスのみ。CAN2の依存CAN1クロックを維持し、共有フィルタバンクを14で分ける。対象軸だけを通すマスクを設定する。

リポジトリの既定boardConfigは未検証・無効。CAN開始前に拒否され、生成mainはError_Handlerへ進み、通常運転にフォールバックしない。ホスト試験用の人工設定は製品へ混入しない。実機設定値や書込みを本PRで承認したことにはならない。

試運転ランタイムはTIM7の番号付き2msイベントでMotorBenchを処理する。RS06はType4停止の送達とその後の新規Type2 Resetを確認、C620は0x200全チャネルゼロを送る。受信・送信終端はCanPortで採取する。CAN IRQから状態機械や制御演算を実行せず、同一IRQをHALに重複処理させない。

## 明示操作の接続

番号付き主ループの `serviceCanBench` から、当該周期の一回限りの操作を `MotorBench::poll` に渡せる。受信・送信終端の回収と保護判定を終えた後、次の送信を予約する前に操作を処理する。C620は停止中もゼロを更新するため、この順序で旧ゼロの完了と新規報告を確認してから開始を判断する。

ホスト試験の入力元は開始と停止が同時なら停止を優先する。開始拒否を次周期へ自動繰越ししない。番号欠落・重複・逆行、時計不成立、FAULT中の新規操作は通さない。従来の停止・期限・IWDG処理は継続する。モータ選択は起動固定であり、運転中の軸変更は追加しない。

実機C入口は操作を渡さない。実機用のボタン、USB、CAN入力、モータID・出力設定は未接続・未確定。

## 仕様上の注意

RS06はCAN1拡張29bit、C620はCAN2標準11bit、各1 Mbps。RS06の角度±4π、速度±50 rad/s、トルク±36 N·m、C620の±16384＝±20 Aは通信表現範囲であり機械的な許容出力ではない。実機FW・主機ID・軸ID・電源・設定値の確認が必要。

既存MotorBenchには単軸パルス用の0.5 N·m/A・200 ms上限、RS06有効化150 ms、受信鮮度30/20 msがある。現行BenchDebugからパルス要求関数を呼ばない。実機通電・非ゼロ要求の接続は別承認。停止フレームの送達は物理停止を保証しない。独立した物理非常停止と支持治具、実機IWDGの確認は残件。

## 確認

- [単軸試運転の操作接続](../verification/f446-bench-operation.md)
- [単軸試運転のC++統合時の検証](../verification/f446-motor-bench-cpp.md)
- [BenchDebug起動・IRQ分離検証](../verification/f446-bench-startup.md)
- [モータ通信コーデック](../verification/f446-motor-codecs-cpp.md)
- [F446 TODO](../todo/f446.md)

旧C11の接続例はGit履歴（master 0cc9e7b以前）に残す。
