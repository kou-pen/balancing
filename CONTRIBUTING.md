# balancing 開発・GitHub運用

このリポジトリはユーザーとAIで共同開発する。基準ブランチは `master`。
規約の正本はこの文書、AI固有の制約は `AGENTS.md`、技術仕様は `docs/specs/` とする。
運用合意の詳細は [GitHub運用規約](docs/development/github-governance.md) を参照する。

## Issue
- 目的単位で作成：機能追加、バグ、調査など。軽微な文書修正はIssue不要。
- 着手前に既存Issueを検索し、重複を避ける。大きな作業は親子・関連Issueに分ける。
- 1 Issueに複数PRを認める。子作業が終了しても元の受入条件を無断で縮小しない。
- ラベルは `type:*` を原則1つ、`area:*` を1つ以上、`priority:*` を原則1つ。
- 進捗はOpen/Closedを基本に、作業中のみ `status:in-progress`、停止時に `status:blocked` を使う。
- AIはIssue作成・更新・通常整理を自律実行可。P0/P1への変更、ユーザー指定の完了条件変更は承認制。
- AIは受入条件を満たした通常Issueをクローズ可。安全・実機検証のIssueは人間の確認が必要。削除は禁止。
- 実装Issueと実機検証Issueは分離し、後者を未完了のまま消さない。

## ブランチ・PR・マージ
- 1 PRに対して1短命ブランチを `master` から作る。`develop` や分野別長期ブランチは置かない。
- 例：`feat/f446-can`、`fix/imu-timeout`、`refactor/control-core`、`test/rs06-codec`、`docs/protocol`。
- 制御、通信、ファームウェア、基板・電源、安全機構、シミュレーションロジックの変更はPR必須。
- 誤字、コメントなどの軽微な変更のみ `master` へ直接コミット可。
- AIはPR作成・レビュー・修正を行うが、`master` へのマージはユーザーの明示的な承認を要する。
- マージ方式は **Squash and merge**。マージ後の作業ブランチは削除する。
- PRタイトルと最終Squashコミットは `type(scope): 内容` のConventional Commits形式。Issueは自然文。
- `feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf` を使用。途中の作業コミットは自由。
- PR本文の `Closes #番号` は、**そのPRでIssueの全受入条件が完了する場合のみ**使用する。
  部分実装なら `Refs #番号` とし、誤った自動クローズを防止する。

## マージ前検証と安全
- ホスト試験PASSでマージ可能。実機検証未実施を隠さず、別Issueで追跡する。
- 安全機構の変更、モータ自動有効化、電源・保護機能の重要変更には事前承認と個別のマージ判断が必要。
- `python3 tools/validate_specs.py`、`python3 tools/run_host_tests.py`、
  `python3 tools/sync_cubeide_cpp.py`、該当テストやlintを実施し、結果をPRに記載する。
- 試験状態は `PASS / FAIL / NOT RUN` を区別。未実施のテストをPASSにしない。
- AIは標準レビューに加えSTM32、CAN、制御、回路、シミュレーションの対象領域別チェックを実施。
  指摘の重大度は `Blocker / Major / Minor / Suggestion`。
- Blocker・Majorの未解決事項は原則マージしない。通常修正はAI自律、重要設計変更は事前承認。

## 並行開発・引き継ぎ
- 独立したIssueのみ原則2〜3件まで並行。別ブランチ・別作業環境/worktreeを使う。
- Github Issue・PRを進捗の正本とし、ローカル状態はGit管理外の `local/handoff/` に補助記録する。
- 中断時はIssue/PRに変更内容、試験、未解決事項、次の手順、ブランチ、コミットを記載する。
- 週1回、停滞Issue・PRを棚卸しする。7日無更新の作業中Issue/レビュー待ちPRを確認し、
  14日停滞時は理由を整理する。停滞期間だけでは自動クローズしない。

## 記録とリリース
- Issue：課題・調査、PR：変更・レビュー・試験、`docs/specs/`：正式仕様。
- 重要な設計決定は `docs/decisions/` のADR、継続参照する検証は `docs/verification/` に記録する。
  大容量波形・動画・bagは外部で管理し、参照先とコミットIDを記録する。
- 実機検証済みの節目でGitタグとGitHub Releaseを作り、基板Revision、試験条件、
  既知の制限、復旧方法を記載する。実機未検証の `master` を安定版とは呼ばない。
- GitHub Actions/自動CIは導入しない。force-push、`master` 削除、無断マージを禁止する。
