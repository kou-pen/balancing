# GitHubリポジトリ運用（準備段階）

**更新日:** 2026-11-09  
**方針:** balancing の単一リポジトリ（モノレポ）。ローカルGitはmasterブランチで初期化・初回コミット済み。originは https://github.com/kou-pen/balancing.git に登録済み。GitHubへのpushはVMからの認証設定待ち。GitHub ActionsとセルフホストCIは導入しない。

## リポジトリの範囲

- firmware/core/：STM32Cubeに依存しないC11制御・CAN層
- firmware/f446/：制御基板のSTM32CubeIDEプロジェクトとC++アプリケーション
- firmware/f303/：電源基板の開発領域（現時点では未完成）
- docs/：仕様・設計決定・TODO・試運転手順
- tests/：ホスト側の単体・回帰テスト
- tools/：検証・CubeIDE再生成後の整合チェック
- hardware/、simulation/：将来、KiCadや制御シミュレーションを同じリポジトリで管理するための推奨配置先（現時点で移動や空ディレクトリ作成はしない）

**正本:** 基板や通信仕様とファームウェアは、可能なら同じ変更単位で更新する。現段階ではAGENTS.mdとdocs/todo/のルールを優先する。

## Gitに含めるファイル

CubeIDEの .ioc、.project、.cproject、.mxproject、Core、App、Drivers、リンカスクリプト、C/C++コード、テスト、Markdown、Mermaid用JavaScript、KiCad設計データ等は管理する。共有VS Code設定の settings.json、tasks.json、extensions.json、および clangd、clang-format、Ruffの設定ファイルも管理する。VS Codeの個人設定とclangd DBの生成物は除外する。[VS Codeツール設定](editor.md)参照。

Drivers/は現在のSTM32Cube F4ライブラリと同じ版で再ビルドできるよう、当面同梱する（将来、依存ライブラリ管理を変更する場合は別途検討する）。

## Gitに含めないファイル

- build/、各種Debug/／Release/、コンパイル生成物、Doxygen HTML出力
- STM32CubeIDEの.metadata/、.settings/、個人設定・一時ファイル
- Pythonキャッシュ・仮想環境
- backups/（過去のローカルZIPなど。**削除はしない**）
- 個人情報・接続トークン・秘密鍵・.envファイル

詳細はルートの.gitignoreと.gitattributesを参照。**.gitignoreは既に追跡されたファイルを削除・秘匿するものではない**ため、最初の登録前に中身を点検する。

## 初回GitHub公開までの手順

現在のVMでは、このディレクトリをローカルGitリポジトリとして扱う。登録対象179ファイルを確認し、初回コミットを作成済み。**GitHubへのpushはまだ未実施**（GitHubプラグインの認証はVMのgitコマンドには共有されない）。

```sh
cd /home/dev-admin/chatgpt-workspace/balancing

# 対象一覧と除外確認
git status --short --untracked-files=all
git status --short --ignored
git check-ignore -v firmware/f446/.metadata/.ide.log build/docs/html/index.html
git check-ignore -v firmware/f446/balancing_f446/balancing_f446.ioc || true
git check-ignore -v firmware/f446/balancing_f446/.cproject || true

# ローカルでの回帰試験
python3 tools/validate_specs.py
python3 tools/run_host_tests.py
python3 tools/sync_cubeide_cpp.py
```

初回コミット時に以下の確認を実施した。初回コミットは作成済みのため、以下のコマンドは再実行不要。

```sh
git add .
git diff --cached --stat
git diff --cached --check
git status --short
git commit -m "Initial balancing firmware and design docs"
```

GitHub側にはbalancingという空リポジトリを作成（初回作成時のREADME/.gitignore/Licenseの自動追加は**避ける**）。リポジトリの公開範囲はそのとき判断する。

```sh
git remote -v
# VMにGitHub用のSSH鍵またはHTTPS用のGit認証を設定した後で実行する
# git push -u origin master
```

originは https://github.com/kou-pen/balancing.git を登録済み。リポジトリはPublicであり、pushすると公開される。GitHub側にコードが未反映の間は、コミットはVMのローカルにのみ存在する。

## 実装とブランチ

- master：少なくともホストテストが通った版を置く。実機確認済みという意味にはしない。
- 機能開発：feature/f446-can、feature/lqr-simなど短命なブランチで進める。
- 仕様・コード・TODOは可能な限り一緒に更新する。
- ファームウェアのCubeMX生成ファイルはUSER CODE以外を直接書き換えず、再生成後にpython3 tools/sync_cubeide_cpp.pyで整合を検査。
- リポジトリ全体のgit reset --hard、git clean -fd、履歴のforce-pushなど破壊的操作を自動化しない。

## CIについて

**今はGitHub Actionsもセルフホストランナーも導入しない。** 既存のローカルテストを維持し、CIが必要になった段階でセルフホストを含めて別途設計する。CI用のワークフロー／認証情報は今回作成しない。

## 大容量データ

STEP/メッシュ/動画/測定ログなどが増えた際はGit LFSか外部ストレージを検討する。現在はLFS設定を強制しない。GitHubの通常Gitでは大容量ファイルの追加前に制限を確認すること。

## 公開前のチェック

公開対象に機密情報・認証情報・学内限定の回路情報・購入関連の個人情報がないか確認。公開先が未決定なら、まずGitHubの**Private**リポジトリで開始するのが安全。