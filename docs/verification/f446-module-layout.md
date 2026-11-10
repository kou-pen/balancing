# F446自作C11/C++ファイルの機能別一括配置（構造変更PR）

**記録日：** 2026-11-10  
**対象：** STM32CubeIDE F446プロジェクト firmware/f446/balancing_f446  
**依存：** ホストCANスケジューラPR #17（Issue #5）。旧配置のCubeIDE正式5ソースビルドはPR #16でPASS。  
**目的：** 実機検証ができない期間に、以降の制御開発で使用するソース構造を確定する。移動と同時に挙動を変更しない。

## 所有権の境界

- Core/、Drivers/、balancing_f446.iocはCubeMX/STの管理領域。C++化やファイル移動をしない。
- 自作C11・C++は原則App/Inc/balancing/（公開ヘッダー）とApp/Src/balancing/（実装）の**機能別ミラー階層**へ配置する。
- 例外：CubeMX生成のCore/Src/main.cが相対パスでincludeしているApp/Inc/balancing_app_bridge.hは互換性のため動かさない。
- App/Examples/documentation_example.hはDoxygenの記述例なのでそのまま維持する。
- core/は将来の共通型・時刻・単位・汎用数値演算用。用途が混在するutil/は作成しない。
- control/estimator/とcontrol/filter/は今後の分類。現時点で実装を持たないため、空ディレクトリ用のダミーファイルを登録しない。

## 一括移動したソース（対応するヘッダーはInc、実装はSrc）

| 新しい相対パス（App/Inc／App/Srcから） | 責務 | 注意 |
|---|---|---|
| balancing/app/balancing_motor_bench.{h,c} | 単軸試運転のユースケース | 通常運転へ接続しない |
| balancing/app/application.cpp | 非駆動C++入口 | App/Inc/balancing_app_bridge.hを利用 |
| balancing/control/controller/balancing_core.{h,c} | 現行C11制御・状態機械 | 内部機能分割は別作業。計算・安全判定は不変 |
| balancing/communication/balancing_motor_can.{h,c} | RS06/C620 CAN codec | CAN開始・送受信には未接続 |
| balancing/communication/balancing_tx_scheduler.{h,c} | Issue #5送信要求モデル | HAL非依存、タイムアウト実機値は未確定 |
| balancing/device/balancing_f446_motor_bench_port.{h,c} | STM32 HAL接続アダプタ | 実機向けコールバックは未接続 |

実行可能ソースは**Cが5件、C++が1件、合計6件**。移動した公開ヘッダーは5件。Cブリッジは旧位置へ残す。

## 調整した参照とビルド契約

- C言語・C++ソースとホスト試験は、balancing/communication/balancing_motor_can.h のように機能別パスを修飾してヘッダーを参照する。インクルード探索基準はApp/Incとする。
- C++入口の既存Cブリッジ参照は、移動後のapplication.cppからの相対パスを変更するのみ。**生成main.cは無変更**。
- CMakeLists.txt、tools/run_host_tests.py、tools/prepare_clangd.py、tools/sync_cubeide_cpp.pyのソースパス・探索を更新。
- tools/verify_f446_build.pyはApp/Srcを**再帰走査**し、各サブディレクトリのsubdir.mk、各相対パス付きobjects.list・.o、ARM ELF32・鮮度を確認。旧直下Makefileや古いELFを新配置のPASSにしない。
- tools/regenerate_cubeide_f446.pyは、各ネスト先のsubdir.mkを確認した上でビルド結果を判定。CubeIDE自身の生成・コンパイル完了ログが必要。
- 将来の追加C/C++、新規ネスト先、生成Makefileの欠落・オブジェクトの古さはtests/test_cubeide_cpp_metadata.pyの独立フィクスチャで検証。

## PC上で実施する検証

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j 4
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
python3 tools/dev_lint.py ruff-check
python3 tools/dev_lint.py ruff-format-check
doxygen Doxyfile
git diff --check
```

## CubeIDEの正式再生成・クロスビルド（別ゲート）

旧直下配置のビルド成果物を利用しない。デバイスへの書込みはしない。CubeIDEが起動可能な**通常VMターミナル**で次を実行する。

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 tools/regenerate_cubeide_f446.py
python3 tools/verify_f446_build.py
```

全6ソースのApp/Src/balancing/...以下のsubdir.mkとobjects.listが自動生成され、DebugとRelease両方で新しいARM ELF32が生成されることを確認する。

**現時点の判定を混同しないこと：** PR #16で確認された旧5ソースのCubeIDE managed-buildはPASSだが、**本PRの6ソース機能別配置のCubeIDE managed-buildは、上記コマンドの実行ログが得られるまでNOT VERIFIED**。Git管理外のMakefileを手修正して得たクロスリンクはCubeIDE本体の再生成成功ではない。

## 変更禁止事項・保留事項

- ファイル移動・ヘッダー参照以外の制御、CANプロトコル、ARM/FAULT判定、電流・トルク値、スケジューラ周期を変更しない。
- HAL割込みの実機競合、緊急停止時のCAN Type4送信順、独立の物理非常停止は未実装/未検証。
- 実機のCAN送信完了タイムアウトは、既定の**方針A（実測後に確定）**。PC用の故障注入値を本番値へ昇格させない。
- HAL起動・モータ駆動・実機書込みは行っていない。

**PR運用：** #17を先にレビュー・マージする。その後、機能別配置PRをmasterへリターゲットし、合否を確認してからマージする。自動マージしない。関連Issue #5はハードウェア未検証を残したまま維持する。
