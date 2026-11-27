# F303 PowerControllerの論理境界試験（2026-11-27）

対象仕様：BOARD-F303-001、PROTO-PWR-001。Issue #102。対象はHAL非依存の初版であり、GPIO/CAN/ADC/IWDGには未接続。

| ファイル | 役割 |
|---|---|
| [PowerController.hpp](../../firmware/f303/balancing_f303/App/Inc/balancing/power/PowerController.hpp) | 起動固定設定、論理入力、状態・出力要求、公開API |
| [PowerController.cpp](../../firmware/f303/balancing_f303/App/Src/balancing/power/PowerController.cpp) | 周期処理、状態遷移、保持・期限判定 |
| [test_f303_power_controller.cpp](../../tests/test_f303_power_controller.cpp) | 人工の時刻・観測・操作イベントで境界を検証 |

## ホストで確認した条件

- 未検証/不正設定では投入不可。異常/未判別リセットはFAULT。
- 通常起動は観測成立後に充電を開始。許容差・保持時間の等号と、一度外れた場合の再計測。
- 同一観測を再利用して時間成立や主電源確認を捏造しない。観測期限・欠落・未来/逆行時刻・非有限値を拒否。
- 充電・重複の期限到達が完了条件より優先。重複区間中の電圧差拡大もFAULT。
- PRECHARGE充電/重複/ONの各段階でCAN ESTOP・ハード異常・ソフト異常・観測不成立・通常OFFを注入。
- FAULT保持、通常OFF/物理停止解除による意図の違い、明示ONでの再充電。
- 主電源成立確認がない状態はONと報告しない。

人工fixtureの値は回路定数ではない。単体試験は通常・UBSanともPASS。CMake Debugの全CTestは82/82 PASS（job 5a402583386444b8）。構成全体の通常/UBSanも全件PASS（job 8c3bab76af9648db、CubeIDE設定25/25、CAN監査19/19）。仕様整合・Doxygen生成・差分検査PASS（Doxygenの既知日本語翻訳警告のみ）。

## 実機未検証

物理主電源成立の確認手段・重複導通許容、ADC換算/状態別保護、GPIO/IRQ排他、リセット時遮断、CAN対向、1 kHz実時間、IWDG、ARMクロスビルドは対象外。F303 CubeMXプロジェクトは未作成。T-F303-01〜06の完了条件は維持する。
