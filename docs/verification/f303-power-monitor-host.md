# F303 PowerMonitorの換算・保護判定（2026-11-27）

対象仕様：BOARD-F303-001（F303-02/04/05/09）、Issue #102。実機ADCや出力には未接続。

| ファイル | 責務 |
|---|---|
| [PowerMonitor.hpp](../../firmware/f303/balancing_f303/App/Inc/balancing/power/PowerMonitor.hpp) | ADC取得単位・校正・保護閾値・公開API |
| [PowerMonitor.cpp](../../firmware/f303/balancing_f303/App/Src/balancing/power/PowerMonitor.cpp) | 換算、鮮度、状態別電圧と持続過電流 |
| [test_f303_power_monitor.cpp](../../tests/test_f303_power_monitor.cpp) | 人工観測・境界試験、実Controllerへの論理接続 |

## 実装境界

生値の有効範囲を確認後、チャネルごとに物理値=count×scale+offsetを計算する。非有限値、float範囲超過、バッテリー非正値・バス負値、未検証/不正設定、ADC無効、未来/逆行/期限切れ、同時刻の観測内容改変を無効扱いし、部分的な値は公開しない。観測時刻は元の取得時刻を保持する。

OFF/PRECHARGE/ON/FAULTの全状態でバッテリー上下限とバス上限を評価し、バス下限はONでのみ適用。電流は符号付き換算値の絶対値で評価する。OFF/FAULT中の過電流は投入禁止、PRECHARGE/ONでは連続した新規観測の継続時間が設定値に達した場合に異常判定する。閾値の等号は許容、継続期限・観測期限の等号は失効。観測の欠落が有効期限以上なら能動状態で保護不成立。状態がPRECHARGEからONへ変わるだけでは過電流保持時間をリセットしない。MonitorはFAULTをラッチせず、Controllerが状態・復旧を管理する。

符号・絶対値、時間継続の比較方法は論理実装上の選択。回路によって電流の観測可能方向・許容値が異なるため、実機に適用する値は後で別承認する。

## 検証

通常・UBSanのF303単体/結合試験PASS。CMake Debug全CTest 83/83 PASS（job 1241b521b3984c5a）。全通常/UBSan回帰試験PASS（job 49452c18f7654ef9、CubeIDE設定25/25、CAN監査19/19）。仕様整合・Doxygen生成・差分検査PASS（既知の日本語翻訳警告のみ）。校正の異常、上下限等号、負電流、過電流の継続・解除、ADC期限、時刻逆行・改変、物理値不正を確認。Monitorの結果を実PowerControllerへ渡し、自動プリチャージ→重複→ON→持続過電流によるFAULT→異常中のPOWER_ON拒否→正常化後の明示ONによる再充電→ADC欠測によるFAULTを確認した。

## 未確定

人工fixtureのADC範囲・係数・閾値・時間は実機値ではない。ADC分解能・基準電圧・回路定数・電流計測範囲・電圧/電流保護値・観測周期・実時間負荷、主電源成立手段・GPIO/IRQ/IWDG、ARMビルドは未検証。MonitorとControllerの観測期限を実機構成で整合させること。CAN故障ビットの新規割当は行わず、T-F303-01〜06の完了条件は変更しない。
