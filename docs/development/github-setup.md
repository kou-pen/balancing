# GitHub側の設定と導入確認

対象： [kou-pen/balancing](https://github.com/kou-pen/balancing) / `master`。

**この文書の手順が存在することはGitHubの設定適用を意味しない。**
確認済みの状態と設定の要否を分離する。

## 設定反映状況（2026-11-09確認）

- GitHub API照合：.github/labels.json の18ラベルが名称・色・説明まで一致。既定ラベル10件は残置。
- GitHubのリポジトリ設定：Squash mergeのみ有効、Merge commit / Rebase merge / auto-mergeは無効。
- GitHub Ruleset protect-master（ID: 26837140）はActive、対象はmaster、ルールは削除禁止とforce-push禁止。PR必須・必須CIは設定しない。GitHub APIでもmasterが保護対象であることを確認。
- **Automatically delete head branches は2026-11-09にユーザーがGitHubのWeb設定画面でONを確認**。接続APIからこの値を独立検証できないため、ユーザー確認として記録する。
- Issue・PRテンプレートはmasterへ配置済み。実際の作成画面の目視確認は未実施。
- .github/workflows/ は未配置。GitHub Actionsは導入しない。

## Labels
定義の正本は `.github/labels.json`。type（原則1）、area（複数可）、priority（原則1）、補助statusを作成する。
GitHubの Settings → Issues → Labels （または `/labels`）でラベルを作成する。
既定ラベル（bug、documentation、enhancementなど）は運用上は使用しない。不要なら整理する。
未導入時に存在しないラベルをIssueに割り当てたと主張しない。

GitHub CLIを認証済みの管理端末に導入した場合、下記のスクリプトで定義を反映できる。
`python3 tools/github_labels.py --dry-run` はGitHubへ書き込まず内容だけ表示する。
`python3 tools/github_labels.py --apply` は `gh` によるラベル作成・更新を行う。旧ラベルは自動削除しない。

## Pull Requests
[Settings → General](https://github.com/kou-pen/balancing/settings) の Pull Requests:
- Allow squash merging：**オン**
- Allow merge commits：**オフ**
- Allow rebase merging：**オフ**
- Automatically delete head branches：**オン**
- Allow auto-merge：当面**オフ**（人間の承認が必須）

`master` がデフォルトブランチであることを確認する。

## Rulesets または Branch protection
[Settings → Rules → Rulesets](https://github.com/kou-pen/balancing/settings/rules) で `master` に適用するルールを作る。
- Block force pushes：**オン**
- Restrict deletions：**オン**
- Require a pull request before merging：**オフ**（軽微な直接コミットを許可するため）
- Required approvals：設定しない（AIレビューは正式なGitHubユーザー承認と同一ではない）
- Required status checks：設定しない（CI未導入）
- Bypass権限：可能なら管理者を含めて保護対象にし、意図しない例外を作らない

公開リポジトリでもプラン・所有者・設定で保護機能の適用可否が異なる場合がある。
作成後に実際の有効状態を確認する。

## 作業フロー
1. 通常作業はAIがIssueを検索し、作業ブランチで実装、ホスト検証。
2. PRを開き、AIレビュー・テスト・安全項目を本文に記録。
3. ユーザーが明示的にマージを承認。
4. Squash and mergeでmasterに統合し、作業ブランチ削除。
5. 週1回Issue・PRを棚卸し。自動クローズなし。
6. 実機検証済み節目はGitタグ＋Release。実機未検証を安定版と表現しない。

## 導入時の確認事項
- [x] 上記18ラベルがGitHub上に存在し、色と説明も一致する
- [x] Squashのみ許可、他マージ方式無効。ブランチ自動削除はユーザーがWebでONを確認
- [x] `master` にForce push・削除禁止のActive Rulesetを設定（破壊的な実試験は行わない）
- [x] Issue/PRテンプレートを`master`に配置（作成画面の目視確認は未実施）
- [x] AIレビューとユーザーのマージ承認を運用規約で区別
- [x] GitHub Actionsワークフローが作成されていない
