# F446ソース配置統合と検証手順

**実施日:** 2026-11-09  
**対象:** `firmware/f446/balancing_f446/App`  
**変更種別:** 配置とビルド参照。モータ動作・安全閾値・CAN仕様の変更は行わない。

## 整理後の責務

`App/Inc`：制御コア・CAN codec・試運転アダプタの公開ヘッダーとC/C++ブリッジ。  
`App/Src`：HAL非依存C11制御・CAN符号化・単軸試運転ロジック、HALポート、C++アプリ。  
`App/Examples`：Doxygen例。  
`Core`と`Drivers`：CubeMX生成コードとHAL/CMSIS。

従来の`firmware/core/`、`firmware/f446/include,src/`、`firmware/examples/`のファイルを移動。C11の内容は元のGit blobのままで、CMake・ホストテスト・clangd・資料を新パスへ変更した。

## ビルド上の注意

CubeIDEの`App`は既存のsourcePathなので、移動したC11とHALポートは新たにMCUコンパイル対象となる。Debug/ReleaseのCコンパイラへ`../App/Inc`を追加し、`tools/sync_cubeide_cpp.py --fix`で復元可能にした。アプリ起動点`application.cpp`は変更していない。CAN起動、割込み接続、非ゼロ駆動、ARM処理は追加していない。

C11側で使用する `lround` / `lroundf` / `fminf` / `fmaxf` のため、Debug/ReleaseのC++リンカに `m`（libm）を明示設定した。`sync_cubeide_cpp.py --fix`でも復旧する。なお実際のクロスリンク成功は未確認。

移動前のCubeIDEクロスビルドPASSは**移動後の結果として使用しない**。新しいソースのコンパイル、重複定義、libmリンク、警告を必ず再確認する。

## 再現コマンド

```sh
python3 tools/run_host_tests.py
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake
ctest --test-dir build/host-cmake --output-on-failure
doxygen Doxyfile
```

CubeIDEのDebug/Releaseクロスビルドは[旧測定記録](f446-crossbuild-20261109.md)のヘッドレスコマンドを新規ワークスペースで再実行する。

| 項目 | 判定 |
|---|---|
| Git上のパス移動 | **PASS**：9ファイルの100%同一リネーム、旧パスの追跡ファイルなし |
| ホストC11/C++（通常・UBSan） | **PASS**：制御、CAN、HAL模擬、C++入口 |
| CubeIDE設定回帰テスト | **PASS**：8件 |
| CMake / CTest | **PASS**：CMakeビルド、CTest 4/4（`cmake --build build/host-cmake --target test`） |
| 仕様検証・CubeIDE設定監査 | **PASS**：`validate_specs.py`、`sync_cubeide_cpp.py` |
| Ruff / git diff --check | **PASS**：lint、format、空白差分 |
| Doxygen | **PASS**：既知の日本語翻訳警告のみ |
| 新配置のCubeIDE Debug/Releaseクロスビルド | **NOT RUN**：リモート実行ゲートがheadless-build.shの起動を拒否。コンパイル不合格を示すものではない |
| 実機書込み/通電/CAN駆動 | **NOT RUN** |

実機動作を新たに可能にしたことは意味しない。関連Issue #5の通信・スケジューラ検証と別管理する。

## 実行済み検証の記録（2026-11-09）

- 実行場所：Balancing Development VM（`/home/dev-admin/chatgpt-workspace/balancing`）
- 確認コミット：`f5f2b70b89080246540383829d76aa97c7f2eea6`
- `python3 tools/run_host_tests.py`：通常・UBSanの制御コア、モータCAN、HALスタブ、C++の全群PASS。CubeIDEメタデータ回帰8件PASS
- `cmake -S . -B build/host-cmake`、`cmake --build build/host-cmake -j 4`：PASS
- `cmake --build build/host-cmake --target test`：CTest 4/4 PASS
- `python3 tools/validate_specs.py`：仕様12件、TODO 7件、Markdown 39件、API参照40件、PASS
- `python3 tools/sync_cubeide_cpp.py`：設定整合性PASS
- `python3 tools/dev_lint.py ruff-check` と `ruff-format-check`：PASS
- `git diff --check origin/master...HEAD`：PASS
- `doxygen Doxyfile`：終了コード0。Doxygen日本語翻訳データの既知警告のみ

**クロスビルドの残件：** リモートMCPのコマンド実行ゲートでは、`/opt/st/stm32cubeide_1.19.0/headless-build.sh` を直接起動できなかった。CubeIDEによるコンパイル・リンクの成否は未判定。任意の代替コマンドでゲートを迂回せず、正規の実行経路でDebug/Release両方のELF生成と、`m`のリンク、警告・サイズを確認する。確認後にこの表へ実測値を追記する。
