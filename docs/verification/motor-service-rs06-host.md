# RS06 4軸の模擬CAN検証

対象はF446の `MotorService`、`Rs06Motor`、`Rs06Codec` と `RobotSequence` の結合です。根拠は [BEHAV-001](../specs/behavior/robot_operation.md)、[PROTO-RS06-001](../specs/protocols/rs06_can1.md) とMOTOR-197/198。基準コミットは `818efcc`、試験環境はooedo-vmです。

## 呼び出し順

1. 模擬CANから受け取った受付・終端イベントは `onRs06TxResult()` に渡す。受信フレームは実際の受信を記録した時刻と共に `onRs06Frame()` に渡す。
2. `rs06ArmFeedback(nowUs)` と `takeObservedFaults(nowUs)` を `RobotSequence::update()` に渡す。全体期限はARM受理時に固定する。
3. `applySequence()` でその周期の要求を受け、`pendingRs06Requests()` から空きメールボックスに投入できる候補を得る。トランスポートは受付結果をその場で記録する。完了・中止は同じトークンで戻す。

起動直後は4軸の停止送信が未完了。停止を要求した軸にType 3が受付済みの場合、旧送信が終端に達した後にType 4を送る。ほかの軸の停止は並行できる。停止完了の条件は各軸のType 4のTx完了で、モータ無効化の報告を別に管理する。

## 受入ケース

| 条件 | 期待結果 |
|---|---|
| HAL受付のみ、旧トークンのTx完了、同時刻以前の応答、形式不正・誤宛先 | 有効化未確認のまま待機。別軸の結果を流用しない |
| Type 3のTx完了後、該当軸で正常な運転モードのType 2を受信 | セッション内でその軸の有効化を確認。残りが1軸でも待機 |
| 3メールボックス満杯、送信中止後の再試行、期限ちょうど | 再試行時刻で期限を延長しない。全軸確認と期限到達が同時ならFAULT |
| 送信確定失敗、CAN1バスオフ、軸の故障報告 | 期限を待たずFAULT。1周期内で復旧した異常も初回の異常記録に含める |
| DISARM、FAULT、旧Type 3終端、Type 4再試行と古い完了 | 未完了軸を追跡して送信し、完了済み軸を再送しない。未完了中は次のARMを拒否 |
| 停止Tx完了、受信時刻の境界、30 ms到達 | 送信完了を軸の無効化報告と区別。Type 2受信時刻が停止Tx完了より後であることを確認。30 msで受信鮮度が失効 |
| 設定上のモータID重複・ゼロ | 送信禁止と異常報告 |

## 試験の前提

模擬CANの有効化・無効化確認にはType 2を採用し、Type 24は鮮度と故障監視に使う。RS06の状態報告にARMセッション番号はなく、新しいType 3のTx完了より後にバス上へ遅れて現れた旧報告は、時刻だけでは完全に識別できない。実CAN接続時にはType 2/24の出現条件、送信完了と受信時刻の取り方、メールボックス中止・バス復旧時の終端通知を照合する。ARM期限とアドレスの試験値はfixture専用。

## 実行結果

- `python3 tools/run_host_tests.py` — 通常・UBSanとも合格。
- CMake/CTest — Debug全14件、ReleaseのMotorService RS06対象試験が合格。
- `python3 tools/validate_specs.py` — 合格。仕様ID、文書リンク、API参照を照合。
- STM32CubeIDE 1.19.0 — Debug／Releaseの管理クロスビルドで新規C++ソースを含むARM ELFの生成を確認。両構成ともエラー・警告0件。
- Doxygen 1.9.1 — 既存の警告を比較するため一時コピーで警告のエラー扱いを解除。基準コミットから警告増加なし。`MotorService`・`Rs06Motor`と本ページのHTML生成を確認。
- 実機書き込み、実CAN通信、C620ゼロ電流送信との同時停止、実機駆動は未検証。
