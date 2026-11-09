# GitHubリポジトリ運用・初回登録履歴

**更新日:** 2026-11-09  
**方針:** balancing の単一リポジトリ（モノレポ）。ローカルGitはmasterブランチで初期化・初回コミット済み。originは git@github.com:kou-pen/balancing.git に登録済み。masterへの初回pushは完了。GitHub ActionsとセルフホストCIは導入しない。

## リポジトリの範囲

- firmware/f446/balancing_f446/App/：C11制御・CAN層、HAL接続、C++アプリを含むユーザー管理コード
- firmware/f446/balancing_f446/Core/・Drivers/：CubeMX生成コードとHAL/CMSIS
- firmware/f303/：電源基板の開発領域（現時点では未完成）
- docs/：仕様・設計決定・TODO・試運転手順
- tests/：ホスト側の単体・回帰テスト
- tools/：検証・CubeIDE再生成後の整合チェック
- hardware/、simulation/：将来、KiCadや制御シミュレーションを同じリポジトリで管理するための推奨配置先（現時点で移動や空ディレクトリ作成はしない）

**正本:** 基板や通信仕様とファームウェアは、可能なら同じ変更単位で更新する。技術要求はAGENTS.mdとdocs/specs/のルールを優先する。GitHub運用はCONTRIBUTING.mdとgithub-governance.mdを参照する。

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

## 初回GitHub登録時の確認履歴

現在のVMでは、このディレクトリをローカルGitリポジトリとして扱う。登録対象179ファイルを確認し、初回コミットを作成済み。**初回コミット `70c82fe` はGitHubの `master` へpush済み**。

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

GitHub側にはbalancingというリポジトリを作成し、初回pushを実施した。

```sh
git remote -v
# 初回pushは完了。以後は新規ブランチからPRを作成する。
```

originは git@github.com:kou-pen/balancing.git を登録済み。リポジトリはPublicであり、pushした変更は一般公開される。

## 実装とブランチ

- master：少なくともホストテストが通った版を置く。実機確認済みという意味にはしない。
- 機能開発：feat/f446-can、feat/lqr-simなど短命なブランチで進める。1 PRにつき1ブランチ。
- 仕様・コード・TODOは可能な限り一緒に更新する。
- ファームウェアのCubeMX生成ファイルはUSER CODE以外を直接書き換えず、再生成後にpython3 tools/sync_cubeide_cpp.pyで整合を検査。
- リポジトリ全体のgit reset --hard、git clean -fd、履歴のforce-pushなど破壊的操作を自動化しない。

## GitHubの運用ルール

Issue・PRの命名、ラベル、AIの権限、承認条件は `CONTRIBUTING.md`、`docs/development/github-governance.md`、`docs/development/github-setup.md`を参照する。

## CIについて

**今はGitHub Actionsもセルフホストランナーも導入しない。** 既存のローカルテストを維持し、CIが必要になった段階でセルフホストを含めて別途設計する。CI用のワークフロー／認証情報は今回作成しない。

## 大容量データ

STEP/メッシュ/動画/測定ログなどが増えた際はGit LFSか外部ストレージを検討する。現在はLFS設定を強制しない。GitHubの通常Gitでは大容量ファイルの追加前に制限を確認すること。

## 公開前のチェック

公開対象に機密情報・認証情報・学内限定の回路情報・購入関連の個人情報がないか確認。公開先が未決定なら、まずGitHubの**Private**リポジトリで開始するのが安全。