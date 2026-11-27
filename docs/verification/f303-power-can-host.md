# F303 PowerCanとF446対向ホスト試験（2026-11-27）

対象：BOARD-F303-001（F303-01/06/07）、PROTO-PWR-001、Issue #102。実CAN/HAL未接続。

| ファイル | 責務 |
|---|---|
| [PowerCan.hpp](../../firmware/f303/balancing_f303/App/Inc/balancing/power/PowerCan.hpp) | 周期受信バッチと論理出力・送信候補 |
| [PowerCan.cpp](../../firmware/f303/balancing_f303/App/Src/balancing/power/PowerCan.cpp) | CAN要求集約・Controller実行・応答・状態報告 |
| [test_f303_power_can.cpp](../../tests/test_f303_power_can.cpp) | 優先順位、周期報告、F446の実受信サービスとの結合 |

## 処理境界

既存F446のPowerBoardCodec、ProtocolId、ProtocolPayload、CanTypesを同じソースから直接利用する。コピー・移動やCAN ID/payloadの変更なし。PowerControllerが電源状態を唯一所有し、PowerCanは一周期に一回だけ呼ぶ。

- 受信バッチは最大8フレーム（ホスト用の固定容量、実CAN負荷は未検証）。IRQ側がデータフレームのみを採取し、欠落を通知する。次周期に古いON要求を残さない。
- 同周期ではESTOP > OFF > ON。最初の同種要求を選択し、残りの同種要求はBusy、停止により抑止された要求はSafetyRejected。受信欠落/容量超過時に新規CAN ONを受理しない。既知の停止要求は処理し、CAN欠落だけで独立電源動作を中止しない。
- POWER_RESULTは要求元・番号・種別・結果・状態を既存Codecで返す。Acceptedは実行完了やONではない。
- POWER_STATUSは起動時・状態変化時・直前生成から100 ms経過時に生成し、連番はuint16で周回。遅延分をまとめて生成しない。未割当flags/faultBitsは0。物理ESTOPを観測した立下り（起動時停止を含む）で自機ESTOP候補を一回生成し、他機から受信したESTOPを反射しない。
- 候補は送信完了ではない。BoardIOでGPIO適用・遮断ラッチ照合後に実送信を行う責務が残る。古いON報告や応答候補を蓄積しない。重大異常のISR直接遮断と受信欠落時の割り込み保護は未接続。

## 試験

F446の実PowerBoardServiceにF303のPOWER_STATUS候補を渡してFAULT/PRECHARGE/ON、500 ms鮮度境界、FAULT通知を確認。F446/Pi要求の応答照合、ON/OFF同時受信の順序入替、CAN ESTOP優先、不正ID/DLC/IDE、自己反射、受信欠落、物理ESTOP、100 ms生成周期、連番周回を試験した。F303単体・F446論理対向は通常/UBSanともPASS。CMake Debug全CTest 84/84 PASS（job c4c37ccef562428e）。仕様整合・Doxygen生成とソースページ・差分検査PASS（既知の日本語翻訳警告のみ）。

## 未検証・実装上の選択

固定バッチ容量8、同種要求の先着選択、状態変化時の追加報告、物理ESTOP候補の立下り生成は今回の論理実装の選択。実CANのフィルタ、割り込みで停止要求を取りこぼさない設計、送信メールボックス・送信終端・バスオフ・時刻、実機の周期/負荷、Pi対向コード、ARMクロスビルドは未検証。ハード異常・物理導通確認とGPIO適用前の候補送信禁止をBoardIO/applicationで検証する。T-F303-05等の実機完了条件は維持する。
