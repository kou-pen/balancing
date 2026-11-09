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
| Git上のパス移動 | コミットされたGitオブジェクトとして確認 |
| 新配置のホストテスト、UBSan、CMake | **NOT RUN** |
| 新配置のCubeIDE Debug/Releaseクロスビルド | **NOT RUN** |
| 実機書込み/通電/CAN駆動 | **NOT RUN** |

実機動作を新たに可能にしたことは意味しない。関連Issue #5の通信・スケジューラ検証と別管理する。
