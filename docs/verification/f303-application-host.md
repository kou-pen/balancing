# F303 application論理統合とF446対向試験（2026-11-27）

対象：BOARD-F303-001（F303-01/05/06/07/09/10）、PROTO-PWR-001、Issue #102。CubeMX自動生成はユーザー指示で保留し、HALに依存しない範囲を実装する。

| ファイル | 責務 |
|---|---|
| [application.hpp](../../firmware/f303/balancing_f303/App/Inc/balancing/power/application.hpp) | 周期入力・出力適用関数・周期完了結果 |
| [application.cpp](../../firmware/f303/balancing_f303/App/Src/balancing/power/application.cpp) | Monitor→Can/Controller→BoardIO→出力適用・完了判定 |
| [PowerCan.cpp](../../firmware/f303/balancing_f303/App/Src/balancing/power/PowerCan.cpp) | 破棄した状態報告を次周期に再生成 |
| [test_f303_application.cpp](../../tests/test_f303_application.cpp) | 模擬基板を用いた統合試験、F446受信サービスとの接続 |

## 周期と出力の契約

1 kHzタイマ通知を呼出側が飽和可能な未消費数として取得する。0件は実行しない。複数通知・同一/逆行時刻は異常扱いとし、追いつくための連続実行を行わない。具体的な実時間ジッタ許容値や周期途中の期限検知は未確定。IRQで採取したADC/CAN/停止入力は不可分のスナップショットとして渡す。

Controllerが唯一の状態所有者。MonitorでADC評価、Canで要求集約、Controllerで状態遷移、BoardIOで論理出力を確認する。出力接続先が適用成功を返した正常周期のみCAN候補とIWDG更新許可を外に返す。正常なFAULT保持も周期完了として扱う。適用拒否時は接続先が両電源を遮断済みという契約とし、ControllerをFAULTへ移し、古いCAN候補を全破棄、次周期に最新状態を再生成する。適用先がない場合も完了しない。拒否後のLEDは論理表示であり、物理反映は次周期以降の責務。送信完了・実IWDG更新を意味しない。

物理停止が観測された場合の自動復帰、CAN OFF/ESTOPによる停止保持を区別する。停止原因が分類不能、出力未検証、周期異常、適用拒否は保守的にFAULTとする。これらは今回の論理実装上の判断であり、実機でのIRQラッチ取得・消費・解除の実装とは区別する。出力適用コールバックにはGPIO更新と最新ラッチ照合の不可分性、失敗時の遮断を要求する。適用後からCAN送信/IWDG更新までの競合も実機接続で防止する。

## 試験

人工設定で全4モジュールを接続し、通常・UBSanの統合試験PASS。CMake Debug全CTest 86/86 PASS（job 9f9c6ad26c334934）。全通常/UBSan回帰試験PASS（job fd5b0d629b524eee、CubeIDE設定25/25、CAN監査19/19）。仕様整合・Doxygen生成とF303固有ソースリンク・差分検査PASS。プリチャージ→重複導通→ON→過電流FAULT→明示ON復旧、F446実PowerBoardServiceへの状態報告、物理停止とCAN停止、複数通知、同一時刻、未処理遮断、ハード異常、未検証既定設定、出力適用失敗時のON報告・応答破棄、次周期のFAULT報告を検証。初回は試験内のF446/F303 PowerState型名が曖昧でコンパイル失敗し、F303型を明記して再試験した。

CubeIDE付属ARM GCC 13.3.1で application.cpp のCortex-M4 hard-floatオブジェクト生成もPASS。完全ELFリンク・実行は未実施。

## 残件

実GPIO、ADC/DMA、CAN、TIM、IRQ、IWDG、実機周期の計測、出力適用後の割り込み競合、主電源導通確認方法、回路定数は未検証。CubeMX生成と製品FWビルドは保留。PR #107のSDK/リンク試験は独立した承認待ちのため本ブランチには含めない。F446側の製品コード・CAN仕様は変更なし。T-F303-01〜06の完了条件は維持。
