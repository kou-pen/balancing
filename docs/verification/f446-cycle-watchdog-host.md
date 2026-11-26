# 制御周期の完了監視とIWDG更新境界

関連: BOARD-F446-001、T-F446-02/07/12。基準 master af834cc。既存6ms FAULT条件は変更しない。

| ファイル | 責務 |
|---|---|
| core/CycleTicket.hpp | TIM7番号・開始/終了の時刻付き証跡 |
| app/CycleHealthMonitor.hpp/.cpp | 主ループ開始・終了照合、番号周回、欠落/旧番号/重複/逆行/未完了診断 |
| device/WatchdogPort.hpp/.cpp | 明示検証済みコールバックへの新規証跡一回限りの更新要求 |
| app/application.cpp / balancing_app_bridge.h | CAN回収前に開始、診断・表示・観測消費まで戻った後に完了 |
| Core/Src/main.c USER CODE | TIM7 ISRの番号増分、pendingと番号の同時回収 |
| tests/test_f446_can1_runtime.cpp --watchdog | 単体と模擬CAN統合、FAULT中の正常停止処理 |
| tests/test_cpp_app.cpp | 単体Applicationの共通時計スタブ |

## 実行順

TIM7 ISR → pending + 32bit番号 → mainの短い排他区間で回収 → balancing_app_begin_control_tick → CAN回収 → balancing_app_control_tick → CycleHealthMonitor.complete → WatchdogPort.refresh（明示設定時のみ）。

最初に観測できた番号を基準とし、次から符号なし差分で欠落数を計算する。番号周回は許容し半周以上の差は旧番号とみなす。欠落または前回未完了を検出した周期には新規更新証跡を発行しない。次の正常周期で復帰可能。時計逆行は基準を巻き戻さず拒否。周期処理が終わっていない、観測なし、重複/逆行採取、旧証跡は更新なし。番号なしC ABIは従来の診断試験互換で更新なし。FAULTに遷移したかではなく正常な停止処理が完了したかで更新を決める。

## 実測（2026-11-26）

- CTest 47/47、job 78af934e39f24045、終了0。初回は既存Application単体試験に時計スタブが不足しリンク失敗したため修正し再実行。
- 通常/UBSan全件PASS、job f195379e5fe74b34、終了0。CAN静的監査19/19、CubeIDE設定22/22。
- STM32CubeIDE 1.19.0 Debug/Release cleanBuild、全54 Appソース、ARM ELF32、0エラー0警告、job 216f8082d58b4d68、終了0。Release text=64292、data=92、bss=11904 B。
- 既存SILの6ms欠落・送信異常・非常停止回帰を維持。

## 未完了

物理IWDGのHALドライバ・開始・リセット原因/LSI実測・約500ms期限の実測は別承認。既定プロファイルではコールバックなし、IWDGを開始しない。500Hz実機WCETは未計測。FaultRecordの再起動保持、物理非常停止、CAN ID、安全閾値は変更なし。
