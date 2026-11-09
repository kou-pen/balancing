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

C11側で使用する `lround` / `lroundf` / `fminf` / `fmaxf` のため、Debug/ReleaseのC++リンカに `m`（libm）を明示設定した。`sync_cubeide_cpp.py --fix`でも復旧する。IDE同梱GNU Arm 13.3.rel1を使用し、移動後の5ソースを含む一時更新済みMakefileから**Debug/ReleaseのARMリンク成功**を確認した。ただし、リンク時の`--gc-sections`により未呼出しのC11関数は最終ELFから除去されるため、数学関数を実行する実機経路の動作検証ではない。

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

**再生成の必須条件:** 既存の`Debug/App/Src/subdir.mk`と`Release/App/Src/subdir.mk`は、配置変更前に生成されたものであり、`application.cpp`しか収録していない。古いMakefileをそのまま実行しても今回の構成を検証したことにはならない。CubeIDEへプロジェクトを再インポートし、`Refresh`と`Build`で生成Makefileを作り直す必要がある。生成した`Debug`/`Release`はバージョン管理しない。

ビルド後に、**5ファイルすべてがコンパイル・リンク対象であり、成果物が新しいこと**を次のコマンドで確認する。

```sh
python3 tools/verify_f446_build.py
```

この確認はビルドコマンド自体を実行せず、生成`subdir.mk`・`objects.list`・オブジェクトファイル・ARM ELF32形式・ソース／ビルド設定からの更新時刻を検査する。古い生成物では`NOT VERIFIED`と終了コード1を返す。**出力がPASSでも実機動作・タイミング・安全性の検証にはならない。**

| 項目 | 判定 |
|---|---|
| Git上のパス移動 | **PASS**：9ファイルの100%同一リネーム、旧パスの追跡ファイルなし |
| ホストC11/C++（通常・UBSan） | **PASS**：制御、CAN、HAL模擬、C++入口 |
| CubeIDE設定・生成物検証の回帰テスト | **PASS**：15件（新しいC/C++ソースの追加も検出） |
| CMake / CTest | **PASS**：CMakeビルド、CTest 4/4（`cmake --build build/host-cmake --target test`） |
| 仕様検証・CubeIDE設定監査 | **PASS**：`validate_specs.py`、`sync_cubeide_cpp.py` |
| Ruff / git diff --check | **PASS**：lint、format、空白差分 |
| Doxygen | **PASS**：既知の日本語翻訳警告のみ |
| 新配置のARM GNU Debug/Releaseクロスコンパイル・リンク | **PASS**：IDE同梱GNU Arm 13.3.rel1＋生成Makefileの対象リストを一時更新し、5ソース全件を含む両ELFを再作成（後述） |
| CubeIDE本体の管理ビルド・Makefile再生成 | **NOT RUN**：リモート実行許可ゲートがheadless-build.shの起動を拒否。IDEによる自動ソース登録の確認だけが残る |
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

**ヘッドレスIDE管理ビルドの残件：** リモートMCPの実行ゲートでは、`/opt/st/stm32cubeide_1.19.0/headless-build.sh` を直接起動できなかった。IDEが自動再生成するMakefileの内容は未判定。一方、IDE同梱GNU Armを許可済みの`make`経由で実行し、**全5ソースのDebug/Releaseコンパイル・リンクPASS**を確認した。検証用に更新したMakefileの詳細・警告・サイズは後述の記録を参照する。これをIDE managed-buildの成功と混同しない。

## 追加したビルド成果物ガード

`tools/verify_f446_build.py` により、CubeIDEの古い生成Makefile/ELFを誤って新構成のPASSと扱うことを防止する。現在のVMで実行すると、**Debug/Releaseとも期待どおりNOT VERIFIED（終了コード1）**：各生成Makefile・`objects.list`から4つのCソース/オブジェクトが欠落し、ELFも旧成果物であった。

スクリプト単体の模擬フィクスチャ試験（新しいARM ELF32、古い生成Makefile、古い.o、古いELF、非ARMバイナリ）を`tests/test_cubeide_cpp_metadata.py`に追加し、回帰テストは **13/13 PASS**。このPASSは検証スクリプト自体の試験結果であり、実機向けクロスビルド完了を意味しない。

## 追加検証：IDE同梱GNU ArmによるDebug/Releaseの全ソース・クロスリンク

**実施場所：** Balancing Development VM、STM32CubeIDE 1.19.0同梱 GNU Tools for STM32 13.3.rel1、2026-11-09。ブランチは`refactor/f446-unified-firmware-tree`。実機書込み・通電・モータ駆動は行わない。

### 実施方法と生成物の由来

1. 旧`Debug/App/Src/subdir.mk`、`Release/App/Src/subdir.mk`は`application.cpp`しか持たなかった。そのため**無変更のmake実行は検証にならない**。
2. 検証専用の未追跡ファイル`build/pr16_refresh_generated.py`を用い、**Git管理外の**`Debug`/`Release`の`App/Src/subdir.mk`と`objects.list`に次の4ソースと対応オブジェクトを一時追加した。
   - `balancing_core.c`
   - `balancing_motor_bench.c`
   - `balancing_motor_can.c`
   - `balancing_f446_motor_bench_port.c`
3. C側のコンパイルオプションにはCubeIDE生成済みのCore/Src設定を使用し、App/Incのインクルードパスを追加。C++には既存の生成ルールを利用。リンクでは既存Makefileのlibm指定を保持した。
4. IDE同梱ツールチェーンの`tools/bin`を`PATH`に指定し、両構成で`make -B -j 4 all`を実行した。**-B**はHAL・C++・新Cソースを含めて、旧オブジェクトの流用なしに再コンパイルしたことを意味する。
5. `python3 tools/verify_f446_build.py`で、両構成の全5ソース・オブジェクト・新しいARM ELF32をチェックした。現検証スクリプトは将来`App/Src`に増えるC/C++ソースも自動発見するため、今後の追加ファイルが旧ビルドから抜けるケースも検出できる。

実行したMakeの形（`TC_BIN`にはIDE同梱`.../gnu-tools-for-stm32.13.3.rel1.../tools/bin`の実パスを設定する）：

```sh
make -C firmware/f446/balancing_f446/Debug   -B -j 4 all "PATH=$TC_BIN:/usr/bin:/bin"
make -C firmware/f446/balancing_f446/Release -B -j 4 all "PATH=$TC_BIN:/usr/bin:/bin"
python3 tools/verify_f446_build.py
```

**注意：** この一時スクリプトで更新した`subdir.mk`はCubeIDE自身が再生成したファイルではない。**これによりGNU Armでの実クロスコンパイル・リンクは確認できたが、CubeIDEのmanaged-buildによるソース自動検出だけは未検証**である。検証スクリプトは成果物の内容と鮮度を検査するもので、誰がMakefileを生成したかまでは証明しない。未追跡の生成ファイルをGitへ含めない。

### 今回の実測値

| 項目 | Debug | Release |
|---|---:|---:|
| ARMコンパイル／リンク | PASS | PASS |
| 使用する全5ソースのビルドとリンク対象登録 | PASS | PASS |
| `make -B`時のコンパイル／リンクエラー | 0 | 0 |
| `make -B`時の警告 | 0 | 0 |
| `tools/verify_f446_build.py` | PASS | PASS |
| text [B] | 15,708 | 9,216 |
| data [B] | 12 | 12 |
| bss [B] | 2,820 | 2,820 |

**サイズの解釈：** 旧ビルドと最終ELFサイズが同じである。今回の移動済みC11/HAL関数は実行入口から呼ばれておらず、`--gc-sections`で最終ELFから除去されたと考えられる。これはソースのコンパイル／リンク成功を否定しないが、モータ制御やCAN出力が実機で動く根拠ではない。

### CubeIDE本体での最終確認方法（未実施）

CubeIDEで`firmware/f446/balancing_f446`をプロジェクトとして取り込み、プロジェクトのRefreshとCleanを実行する。Build Configurationを`Debug`に設定してBuildしたあと、`Release`へ切り替えて同様にBuildする。このとき`Debug/App/Src/subdir.mk`と`Release/App/Src/subdir.mk`が**CubeIDE自身によって5ソースを含む状態へ再生成**されるか確認する。最後に`python3 tools/verify_f446_build.py`を実行し、0警告・ELF・リンクサイズを記録する。

この手順の成功までは「CubeIDE管理ビルドPASS」とは記録しない。ただし、IDE同梱GNU ArmコンパイラによるDebug/Release実クロスリンクの阻害要因は今回の検証で見つからなかった。

