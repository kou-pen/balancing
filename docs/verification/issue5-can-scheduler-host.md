# Issue #5：CANスケジューラの仮想時刻ホスト検証

**日付:** 2026-11-09  
**環境:** Balancing Development VM（`/home/dev-admin/chatgpt-workspace/balancing`）  
**実装コミット:** `0a78f94`（初期実装）、`ec8402b`（不正初期化時の無効化ガード）。  
**作業ブランチ:** `feature/issue5-host-scheduler`（2026-11-10にマージ済みPR #16の `master` を取り込み、ホスト用実装を独立PRで提出）  
**実施範囲:** PC仮想時刻・模擬CANメールボックスのみ。実機書込み/駆動なし。

## 検証対象

- 新規C11 `App/Inc/balancing/communication/balancing_tx_scheduler.h`／`App/Src/balancing/communication/balancing_tx_scheduler.c`。
- 新規 `tests/test_tx_scheduler.c` を `tools/run_host_tests.py` の通常とUBSanの両方に追加。
- CMake `balancing_tx_scheduler_tests` をCTestへ追加。
- 既存 `application.cpp`、`main.c`、CAN HALアダプタ、安全状態機械は**変更していない**。

## 実行結果

| コマンド | 結果 |
|---|---|
| `python3 tools/run_host_tests.py` | **PASS**：制御コア、CAN codec、HAL模擬、C++、CANスケジューラ（通常＋UBSan）、CubeIDE設定回帰16件 |
| `cmake -S . -B build/host-cmake` | **PASS** |
| `cmake --build build/host-cmake -j 4` | **PASS** |
| `cmake --build build/host-cmake --target test` | **PASS**：CTest 5/5 |
| `python3 tools/validate_specs.py` | **PASS**：仕様ID・リンク・参照整合 |
| `python3 tools/sync_cubeide_cpp.py` | **PASS**：メタデータ |
| `python3 tools/dev_lint.py ruff-check` | **PASS** |
| `python3 tools/dev_lint.py ruff-format-check` | **PASS** |
| `doxygen Doxyfile` | **PASS**：既知の日本語翻訳警告あり |
| 新しい6ソース構成でのSTM32CubeIDE Debug/Releaseクロスビルド | **NOT RUN**（PR #16の5ソース管理ビルドはPASS。本Issue追加の`balancing_tx_scheduler.c`はMakefile未再生成） |
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
| キャッシュされた要求の競合 | PASS | `next()`の拒否を無視して`accepted()`を直接呼んでも、CAN1の旧世代／CAN2の既占有を拒否 |
| 再ARM後の旧送信完了 | PASS | DISARM前のRS06 Tx完了が遅れても、再ARM後の正常なTx完了と誤認しない |
| CAN2旧世代の遅延完了 | PASS | 前の2 ms世代のC620共通指令完了では新世代のTX鮮度を更新せず、新世代の完了時のみ更新 |
| 同一時刻の制御ティック重複 | PASS | 同じ時刻・同じ駆動許可で2回呼んでもRS06/C620要求世代を増加させない |
| 同一時刻のDISARM優先 | PASS | 同時刻でも`true → false`はゼロ要求へ切替え、`false → true`の即時再ARMは無視 |
| 外部注入TX期限の境界 | PASS | RS06/C620の異なる注入期限で、期限直前は正常、期限ちょうどで独立したTX異常 |

**ホスト故障注入専用パラメータ：** テスト基本設定でRS06 TX完了監視 `9,000 µs`、C620 TX完了監視 `6,000 µs`。別の試験で仮想条件に合わせて値を変更。**いずれも実機の許容遅延・安全閾値ではない。** 設計文書 [CAN送信スケジューラ](../design/can-scheduler.md) を参照。

**ユーザーの設計判断（2026-11-09）：方針Aを採用。** RS06とC620のTX完了監視の**実機用タイムアウトは実測後にそれぞれ確定**する。ホストの閾値はすべて試験用であり、実機用のデフォルトとして設定しない。`b_sched_init`で値0を拒否する仕様を維持し、値が未確定の段階では実機送信へ接続しない。

### 敵対的レビューの再現手順

- テスト先行で`accepted()`へのキャッシュIntent直渡しを試したところ、**旧世代in-flightを無視した追加受付**が発生し、assertが失敗した。その後、`next()`と`accepted()`に同じバス占有ガードを追加してPASS。
- DISARM前にHAL受理したRS06指令を再ARM直後に完了させる時系列を追加。立下がりで世代を無効化し、旧完了を新セッションの正常Txとして記録しないことを確認。
- 同一`now_us`のティックを2回実行したところ、従来はC620世代が余分に進んでassertが失敗した。通常の重複ティックを無処理にしてPASS。ただし`true → false`の同時刻DISARMまで無視する別の欠陥をテストで検出し、DISARMだけは優先・同時刻再ARMは拒否する形へ修正した。
- 監視値を`4,000/8,000 µs`等に変更して期限の境界を試験し、RS06/C620で独立に閾値が効くことを確認。

これらは**実装の論理的な競合検出**であり、HAL ISRの真の並列性、実CANバスオフ、物理非常停止、メッセージ遅延の上限を測ったものではない。

## 実機を接続する前に残る項目

1. 実機CAN1のType1/Type3/Type4全送信者間の調停、緊急停止優先、受理済み旧フレームのキャンセル・Bus-off時の扱い。
2. HAL送信API成功からモデル`accepted()`記録までの割込み競合を防ぐ手順。CAN1/2それぞれのIRQ優先順位。
3. 実機計測に基づくRS06各軸/C620共通送信のタイムアウト値。現在は未決定。
4. `BModeMachine`の既存FAULT入力へ送信監視エラーを結合すること。物理非常停止の独立動作。
5. CubeIDEプロジェクトの新規`balancing_tx_scheduler.c`を含めた6ソース全件をコンパイルしてDebug/ReleaseのELFを確認。PR #16の5ソース成功は本Issueの6ソース成功を意味しない。`tools/verify_f446_build.py`は現時点で両構成NOT VERIFIED。
6. 実CAN 1 Mbpsの負荷、割込みレイテンシ、CPU使用率、TX完了タイムスタンプ、RX鮮度、PWM/制御周期の測定。

**結論：** Issue #5のホストで検証可能な要求モデルは作成し、仮想時刻による正常系・故障系の試験がPASS。MCUクロスビルドと実機の締切・安全性は証明していない。Issueは安全・実機検証未完につき自動クローズしない。

## 2026-11-10：ホスト実装の統合ゲートと構造移動の順序

- PR #16 は `master` にSquashマージ済み。既存コード一元配置の基準は確定した。
- Issue #5の**受入条件で指定された仮想時刻スケジューラの挙動・CAN1 3メールボックス・遅延時破棄・RX/TX独立監視・6 ms周期停止・時刻逆行のホスト検証はPASS**（通常・UBSan）。CMake/CTest 5/5、仕様監査、Ruff、DoxygenもPASS。
- **このPRの完了範囲は抽象送信要求モデルとホスト故障注入まで**。実機値・HAL割込み経路・Type3/Type4物理停止調停・モータ駆動は接続も認証もしていない。
- 新規C11ソースはまだMCUの生成Makefileに含まれないため、CubeIDE Debug/Release実リンクはこのコミットでは未確認。追加ソースを認識させるCubeIDE正規再生成が必要。
- **作業順序の変更（ユーザー指示）**：このホスト実装をPRで区切った後、C11/C++自作コードを`App/Inc/balancing`と`App/Src/balancing`へ**まとめて再配置**する。移動と同時に制御・通信ロジックの書き換えはしない。ヘッダ公開参照、CMake、テスト、IDE設定、設計文書を移動PRで更新。
- ハードウェアが使えないため、Issue #5そのものの実機安全検証をPASS扱いにせず、未実施チェックを残す。実機TXタイムアウト値は方針A（実測後に確定）を維持。

