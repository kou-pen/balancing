# C++17集中移植前のmaster基準検証（2026-11-12）

**目的：** 新規機能開発凍結の前に、旧C11＋既存C++骨組みの動作とビルドの基準を確定する。ここに記載するPASSはホスト・クロスビルドの範囲のみ。**実機動作と移植完了は未検証。**

## 固定した検証対象

| 項目 | 値 |
| --- | --- |
| リポジトリ | `kou-pen/balancing` |
| ブランチ | `master` |
| ソースコミット | `1eb308272bdfb10480308ac1b1548ab576eee4b3` |
| 既存統合PR | #24 LegPD/RollPD、#25 YawPI、#26 モータベンチ受信期限 |
| 実行日 | 2026-11-12 |
| 環境 | Ubuntu 22.04.5 LTS / x86_64、GCC 11.4.0、CMake 3.22.1、Python 3.10.12 |
| ARMクロスビルド | STM32CubeIDE 1.19.0公式ヘッドレス管理ビルド |
| 実行ディレクトリ | 隔離チェックアウト（元の開発用作業ツリーやmasterを変更しない） |
| ソース差分 | 実行後の`git status --porcelain`は空（追跡対象に変更なし） |

## 実行手順と実測結果

いずれも上記コミットのリポジトリルートで実行した。

| コマンド | 結果 | 検証範囲 |
| --- | --- | --- |
| `python3 tools/run_host_tests.py` | **PASS**（終了コード0） | C11コア・CAN codec・送信スケジューラ・HALモック・C++ ABI・六節リンク数学fixture・観測・操作・姿勢フィルタ・Issue #10結合・LegPD/RollPD・YawPI、それぞれ通常/UBSan。加えてCubeIDEメタデータ・CAN契約静的検査・Python単体試験20+14件PASS |
| `python3 tools/validate_specs.py` | **PASS** | 仕様書12件、TODO7件、Markdown57件、API仕様参照71件 |
| `cmake -S . -B build/host-cmake` | **PASS** | C11/C++17ホスト設定生成 |
| `cmake --build build/host-cmake` | **PASS** | ホスト静的ライブラリ・12テスト実行体 |
| `ctest --test-dir build/host-cmake --output-on-failure` | **PASS：12/12件** | CTest全件 |
| `python3 tools/regenerate_cubeide_f446.py` | **PASS：Debug/Release** | IDEのIOC/プロジェクトメタデータ照合と公式managed clean-build、全Appソースを含むARM ELFの存在/鮮度 |
| `tests/spec_gap_core_mode.c` の個別コンパイル・実行 | **既知仕様不一致の再現、終了コード1（意図的）** | MOTOR-151、MOTOR-198の診断。通常テストスイートへ混入させない |

CubeIDEビルドの末尾ログはDebugが`Build Finished. 0 errors, 0 warnings.`、Releaseも`Build Finished. 0 errors, 0 warnings.`。

| ARM ELF | text [B] | data [B] | bss [B] |
| --- | ---: | ---: | ---: |
| Debug | 15708 | 12 | 2820 |
| Release | 9216 | 12 | 2820 |

**注：** この実測は2026-11-12のmasterに限る。C++17移植PRを統合した後には必ず再実行する。ツールの完走だけでは配線、CAN実通信、制御周期の実測、物理E-STOP、ロボットの倒立を検証していない。

## 凍結前の既知安全仕様差分（Issue #4）

仕様正本：`docs/specs/behavior/robot_operation.md` / `docs/specs/boards/f446_control.md`。実装：`App/Src/balancing/control/controller/balancing_core.c`。

| 論点 | 現行実装と診断結果 | 仕様との関係 | 凍結前に必要な処置 |
| --- | --- | --- | --- |
| **MOTOR-151** | `arm_preconditions()` が `posture_ok` を要求し、他の全条件が正常でも偽ならARMを拒否 | 姿勢・脚長の*指定範囲そのもの*はARM条件にしないという仕様と不一致 | ARMの許可判定とBALANCING中の機構保護を混同せず分離して修正し、境界テストを実施。制御中の保護を不用意に除去しない |
| **MOTOR-198** | ARM後にRS06×4の新規有効化応答が来なければ、無期限に`arm_pending`を保持する（診断では2秒時点`mode=0 pending=1 disable=0`） | 有効化失敗時に全台無効化・FAULTが必要だが、失敗判定と通常運転向け期限は未確定 | 送信失敗・故障応答・応答タイムアウト等の判定者、検証済み期限、4台停止要求、FAULTラッチを具体設計し、故障注入テストで検証 |

`tests/spec_gap_core_mode.c` の実出力：

```text
MOTOR-151: SPEC MISMATCH: posture gate rejected otherwise ready ARM
MOTOR-198: DIAGNOSTIC only (timeout not specified): after 2s without replies, mode=0 pending=1 disable=0
```

`BMotorBench` の **150 ms有効化待機は単軸ベンチ専用**で、通常運転の有効化期限として根拠なく流用しない。2秒は診断時刻であり、仕様の期限でもない。

## 凍結および移植再開のゲート

1. **完了済み：** 統合masterのホスト通常/UBSan、仕様照合、CMake/CTest、CubeIDE Debug/Releaseの基準検証。
2. **未完了：** MOTOR-151/198の仕様差分の処置。安全動作変更なので、具体的なガード/期限・変更ファイル・試験案を提示して承認を得てから実装する。実装後はこの全検証セットを再実行する。
3. **その後：** 新規機能開発を凍結し、基準SHA/タグと移植元の不変条件を確定する。C11の新規機能追加（VMC統合・通常駆動CAN HAL経路など）は原則行わず、C++17移植完了後に直接開発する。
4. **移植期間：** 23項目の合意済み設計をもとに集中移植。PRは1機能ずつ、実装/テスト/Markdownは原則別コミット、masterへのマージはユーザー承認が必要。
5. **開発再開：** 移植済み機能の旧C11との回帰、ホスト/UBSan/CubeIDE、故障注入の完了と残る実機未検証項目の明示をゲートとする。

関連：[設計・移植ゲート Issue #27](https://github.com/kou-pen/balancing/issues/27)、[安全仕様差分 Issue #4](https://github.com/kou-pen/balancing/issues/4)、[既存状態機械監査](core-mode-contract-audit.md)。

**今回の文書PRは記録のみで、ソースの安全判定変更・新規実装・MCU書き込み・実機駆動を含まない。**
