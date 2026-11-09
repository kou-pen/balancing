# VS Code: Ruff / clangd / clang-format の開発環境

**更新日:** 2026-11-09  
**対象:** Balancingリポジトリ全体（VMのワークスペースをVS Codeで開く想定）。GitHub Actionsは導入しない。

## 管理するファイル

| ファイル | 用途 |
|---|---|
| .vscode/settings.json | PythonのRuff保存時整形、C/C++のclangd診断、C++保存時整形（Cは手動整形） |
| .vscode/extensions.json | Ruff、Python、clangd、CMake Tools、EditorConfig拡張の推奨 |
| .vscode/tasks.json | Ruffチェック、clangd DB生成、C/C++ホストテスト、仕様検証 |
| ruff.toml | Python 3.10、100桁、E4/E7/E9/F/IのLintルール |
| .clang-format | C/C++17、4スペース、100桁、Allman括弧 |
| .clangd | clangdの索引とコンパイルデータベース位置 |
| tools/prepare_clangd.py | CMake DBにSTM32Cube生成Core用コンパイルコマンドを追記 |
| tools/dev_lint.py | Ruff CLIの確認・Lint・Format |

.vscodeの共有設定はGit管理対象。VS Code個人設定は追跡しない。

## VS Codeの初期設定

1. Balancingのルートディレクトリを**ワークスペースのルート**としてVS Codeで開く。STM32CubeIDEの内部balancing_f446だけを開くと、親ディレクトリの設定・clangd DBは適用されない。
2. VS Codeの拡張機能メニュー（Ctrl+Shift+X）からワークスペースの推奨拡張をインストールする。
3. clangd拡張を有効化。clangd実行バイナリがない場合、拡張側のダウンロード案内に従う。Microsoft C/C++拡張のIntelliSenseが同時起動して競合しないようにする。**このVMには現状clangd/clang-formatの実行バイナリは未導入。**
4. Ctrl+Shift+P → Tasks: Run Task → **clangd: include STM32 Core** を1回実行する。CMake DB生成→STM32用フラグ追記が行われ、build/compile_commands.jsonを作成する。
5. C/C++ファイルを開いてシンボル補完、ヘッダ参照を確認。**C++は保存時に整形**、CubeMXが自動生成するCは意図せぬ差分を防ぐため保存時整形を無効にし、必要なときだけ手動で整形する。PythonファイルではRuff診断と保存時整形が動く。

CMake DBはホスト用のC11／C++17テストコードを網羅し、STM32のCore/SrcおよびHALアダプタには解析用のCortex-M4フラグを付ける。**clangdで診断できることはCubeIDEのクロスビルドが成功することを意味しない**。STM32のコンパイル・書込みはCubeIDEで検証する。

## Ruff CLI

このVMではリポジトリ直下のローカル仮想環境にRuff CLIをインストール済み（実行ファイルのパスは .venv/local/bin/ruff）。仮想環境はGitから除外する。他環境では次のようにセットアップする。

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 -m venv .venv
.venv/bin/python -m pip install ruff

python3 tools/dev_lint.py ruff-check
python3 tools/dev_lint.py ruff-format-check
```

VS CodeのRuff拡張は内蔵Ruffを利用する設定なので、CLIが未インストールでもエディタ内の診断・整形機能は使える。CLI・拡張のバージョンに差が出る場合は同じ版に揃える。

Pythonの保存時にはRuffの整形と**安全な修正・import整理**を実行する。CLIで明示的に修正する場合だけ次のコマンドを使う。

```sh
python3 tools/dev_lint.py ruff-fix
python3 tools/dev_lint.py ruff-format
```

## clangd用DBの再作成

CMakeのソース追加やCubeMX再生成でC言語ファイルが変わったら、VS Codeのclangd: include STM32 Coreタスクを再実行する。CLIからは下記。

```sh
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
python3 tools/prepare_clangd.py
```

コンパイルDBは生成物なのでbuild/とともにGit管理しない。生成元CMake設定と追加スクリプトを管理する。

**注意:** STM32Cube生成物のCore/とDrivers/はCubeMXが管理し、手動編集はUSER CODEブロックの範囲に限定する。clang-formatの保存時整形がUSER CODE外の自動生成部分に作用するおそれがあるため、**生成コードの整形は無効**にし、利用者が管理する`firmware/f446/balancing_f446/App/`を整形する。

## VS Codeタスク

Ctrl+Shift+P → Tasks: Run Task から実行。

- Ruff: check、Ruff: format check
- clangd: include STM32 Core（CMake DBとSTM32解析フラグ）
- Host: C/C++ tests
- Docs: validate
- CubeIDE: metadata check

## CI

GitHub Actionsとセルフホストランナーは今回**導入しない**。検証はローカルタスクとして準備し、将来のCIに転用できるようにする。