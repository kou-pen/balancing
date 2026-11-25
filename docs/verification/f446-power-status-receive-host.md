# F446 CAN2 電源状態の受信境界（ホスト検証）

対象: PROTO-PWR-001 / BOARD-F446-001 / BEHAV-001、T-PROTO-05/11。
基準: master 8976cc53d6cf15f31dbc9969309b8edf1d88bb5a。

## 実装と責務

- `PowerBoardService.hpp/.cpp`: `acceptCanStatus` でバス・配送分類・Codec結果を検査し、既存の `acceptDecoded` に渡す。受信時刻と主ループ評価時刻は独立。flags/faultBits/sequenceは意味を確定せず保存する。
- `CanRouter`、`PowerBoardCodec`、500 msの鮮度と保留FAULTの意味は変更しない。
- 本番の配送先・フィルタ・ARM・電源操作に接続しない。試験用Sinkのみで既存のCanPortとCanRouterを通す。

## 受入検証

| 観点 | 試験内容 |
|---|---|
| 正常受信 | 独立した固定ID 0x0840BC03 とバイト列を模擬FIFOに入れ、CanRouter経由で状態・受信時刻・Big endian生値を照合 |
| 不正フレーム | 別バス、別分類、DLC、IDE、ID、列挙、予約領域で拒否し保持値を変更しない |
| 時刻 | 同時刻・逆行・未来・受理時点500ms経過を拒否、鮮度は採取時刻+500msちょうどで失効 |
| FAULT | 正常なFAULT受信後、次のONが来ても既存の保留通知が消えない |
| 結果応答 | POWER_RESULTは状態報告の代用にしない |

対象試験: `tests/test_f446_power_board_service.cpp` と `tests/test_f446_can_event_poll.cpp`。

## 実測結果（2026-11-25）

- CMake Debug ビルド成功、CTest 33/33 PASS（全件）。
- `python3 tools/run_host_tests.py` 通常・UBSan全件PASS（VM job `5db7f79afc1047a0`）、CAN監査19/19、CubeIDE設定22/22。
- `python3 tools/validate_specs.py`、`python3 tools/sync_cubeide_cpp.py`、`doxygen Doxyfile`、`git diff --check` PASS。Doxygenは既知の日本語翻訳警告のみ。

## 未検証・後続

連番の重複・再起動・周回の扱い、未定義の状態/故障ビット、CAN2欠落・bus-offの上位通知、F303旧FWとの互換性、実機フィルタ有効化と本番Applicationへの配送は別途検討する。物理電源供給・ARM許可を証明する試験ではない。ARMクロスビルド・STM32書込み・実CAN通信は未実施。
