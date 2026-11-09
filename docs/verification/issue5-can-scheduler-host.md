# Issue #5：CANスケジューラの仮想時刻ホスト検証

**日付:** 2026-11-09  
**環境:** Balancing Development VM（`/home/dev-admin/chatgpt-workspace/balancing`）  
**実装コミット:** `0a78f94`（初期実装）、`ec8402b`（不正初期化時の無効化ガード）。  
**作業ブランチ:** `feature/issue5-host-scheduler`（PR #16の作業ブランチ `refactor/f446-unified-firmware-tree` から派生）  
**実施範囲:** PC仮想時刻・模擬CANメールボックスのみ。実機書込み/駆動なし。

## 検証対象

- 新規C11 `App/Inc/balancing_tx_scheduler.h`／`App/Src/balancing_tx_scheduler.c`。
- 新規 `tests/test_tx_scheduler.c` を `tools/run_host_tests.py` の通常とUBSanの両方に追加。
- CMake `balancing_tx_scheduler_tests` をCTestへ追加。
- 既存 `application.cpp`、`main.c`、CAN HALアダプタ、安全状態機械は**変更していない**。

## 実行結果

| コマンド | 結果 |
|---|---|
| `python3 tools/run_host_tests.py` | **PASS**：制御コア、CAN codec、HAL模擬、C++、CANスケジューラ（通常＋UBSan）、CubeIDE設定回帰13件 |
| `cmake -S . -B build/host-cmake` | **PASS** |
| `cmake --build build/host-cmake -j 4` | **PASS** |
| `cmake --build build/host-cmake --target test` | **PASS**：CTest 5/5 |
| `python3 tools/validate_specs.py` | **PASS**：仕様ID・リンク・参照整合 |
| `python3 tools/sync_cubeide_cpp.py` | **PASS**：メタデータ |
| `python3 tools/dev_lint.py ruff-check` | **PASS** |
| `python3 tools/dev_lint.py ruff-format-check` | **PASS** |
| `doxygen Doxyfile` | **PASS**：既知の日本語翻訳警告あり |
| 新しい配置でのSTM32CubeIDE Debug/Releaseクロスビルド | **NOT RUN**（PR #16のブロッカー継続） |
| CAN1/2実機バス波形・CPU負荷・ISR競合 | **NOT RUN** |
| 実機書込み・送信・モータ駆動 | **NOT RUN** |

## ホストモデルの試験ケース

| 試験 | 判定 | 観測・条件 |
|---|---|---|
| 正常周期 | PASS | 0/2/4/6/8 msのCAN2要求5件、0/4/8 msのCAN1要求3×4件 |
| 3メールボックス | PASS | CAN1先頭3件のHAL受付後、4件目は送信完了で空きが出てから受付 |
| 受付≠完了 | PASS | `accepted()`でTX完了時刻は更新されず、`completed()`で初めて更新 |
| 待ち行列の非蓄積 | PASS | 次4 ms周期へ旧未受付要求を持ち越さない。旧世代がまだHAL占有中は新世代を保留 |
| 遅れたティック | PASS | 4 msのRS位相を維持し、過去周期の要求をまとめ送りしない |
| CAN2バックプレッシャー | PASS | 未受付要求は最新1件だけ。旧フレームが未完了なら新しい共通指令を重ねない |
| 受信正常・送信完了欠落 | PASS | 新規RXが継続してもC620送信完了が途絶えるとTX異常要求 |
| RS06各軸TX監視 | PASS | 一軸のTX完了欠落を検出。別軸の受信で隠れない |
| RS06受信断 | PASS | 仮想時刻30 msでRS軸RX異常要求 |
| C620受信断 | PASS | 仮想時刻20 msで車輪RX異常要求 |
| 制御6 ms停止 | PASS | 駆動許可中、ティック間隔がちょうど6 msでFAULT要求 |
| 時刻逆行 | PASS | 駆動許可中に逆行を検出しFAULT要求をラッチ |
| 通常DISARM | PASS | RS06トルク要求を停止し、CAN2をゼロ要求へ置換 |
| Bus-off | PASS | FAULT要求をラッチし、自動復帰しない |
| 不正な再初期化 | PASS | 以前有効だったスケジューラを再初期化失敗時に無効化し、新規送信要求を禁止 |

**ホスト故障注入専用パラメータ：** テスト基本設定でRS06 TX完了監視 `9,000 µs`、C620 TX完了監視 `6,000 µs`。別の試験で仮想条件に合わせて値を変更。**いずれも実機の許容遅延・安全閾値ではない。** 設計文書 [CAN送信スケジューラ](../design/can-scheduler.md) を参照。

## 実機を接続する前に残る項目

1. 実機CAN1のType1/Type3/Type4全送信者間の調停、緊急停止優先、受理済み旧フレームのキャンセル・Bus-off時の扱い。
2. HAL送信API成功からモデル`accepted()`記録までの割込み競合を防ぐ手順。CAN1/2それぞれのIRQ優先順位。
3. 実機計測に基づくRS06各軸/C620共通送信のタイムアウト値。現在は未決定。
4. `BModeMachine`の既存FAULT入力へ送信監視エラーを結合すること。物理非常停止の独立動作。
5. CubeIDEプロジェクトの新しいApp/Srcソース全件をコンパイルしてDebug/ReleaseのELFを確認。PR #16の`tools/verify_f446_build.py`がPASSするまで、過去の成果物で代用しない。
6. 実CAN 1 Mbpsの負荷、割込みレイテンシ、CPU使用率、TX完了タイムスタンプ、RX鮮度、PWM/制御周期の測定。

**結論：** Issue #5のホストで検証可能な要求モデルは作成し、仮想時刻による正常系・故障系の試験がPASS。MCUクロスビルドと実機の締切・安全性は証明していない。Issueは安全・実機検証未完につき自動クローズしない。
