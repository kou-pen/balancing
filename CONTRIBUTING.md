# 開発・PRの進め方

基準ブランチは `master` です。通常の変更はブランチを作成してPRでレビューします。**最終マージにはユーザーの明示的な承認が必要です。**

## 開発の流れ

| 段階 | やること |
|---|---|
| 課題を確認 | 既存Issueを検索。目的・仕様ID・受入条件を確認 |
| 実装 | 原則1PRにつき1目的。CubeMX生成コードと自作コードの境界を維持 |
| テスト | 変更対象のホスト試験と仕様・メタデータ検査を実施 |
| レビュー | 変更内容、試験結果、未検証事項をPRへ記録 |
| マージ | ユーザー承認後にSquash merge。完了条件が残るIssueは閉じない |

詳しい合意内容は [GitHub運用規約（23項目）](docs/development/github-governance.md) にあります。AIの追加制約は [AGENTS.md](AGENTS.md) が正本です。

## 変更の種類

| 対象 | 扱い |
|---|---|
| 制御、通信、FW、基板、安全、シミュレーションロジック | PR必須 |
| 誤字などの軽微な文章修正 | 直接修正も可 |
| 安全閾値、モータ自動有効化、電源・保護機構、実機書込み | **変更・実行前の承認が必要** |
| 仕様未確定のモータ設定や実測値 | 推測で決めない |

ブランチは短命にし、名前は `feat/`、`fix/`、`refactor/`、`docs/` などを使用します。PR・SquashコミットはConventional Commits形式です。複数PRで対応するIssueには、受入条件をすべて満たすまでは `Refs #N` を使います。

## コミットの分け方

同じPR内でも、**ソース実装・テスト・Markdownは別コミット**にします。原則「実装 → 回帰テスト → ドキュメント」の順で、各コミットの目的が分かる名前を付けます。重要な計算や安全判定の設計意図は、ヘッダーだけでなく **`.c`/`.cpp` の処理箇所にも**記載します。詳細は [AI開発規則](AGENTS.md) を参照してください。

## 最低限の検証

```sh
python3 tools/validate_specs.py
python3 tools/run_host_tests.py
python3 tools/sync_cubeide_cpp.py
```

必要に応じてCMake/CTest、Ruff、Doxygen、CubeIDE管理ビルドも実行します。**ホストPASSは実機PASSではありません。** 未実施なら `NOT RUN` と記録します。

## 記録先

| 情報 | 正本 |
|---|---|
| Issueの受入条件・進捗、PRのレビューと試験 | GitHub |
| 技術仕様・確定した安全条件 | [仕様ID索引](docs/spec-index.md)以下の各仕様書 |
| 長期参照する試験証拠 | [検証記録](docs/verification/index.md) |
| 実装の読み方・設計の解説 | [ドキュメント案内](docs/index.md) |

リリースはGitタグとGitHub Releasesで管理し、実機未検証の `master` を安定版と呼びません。GitHub Actionsは導入していません。force-pushと `master` の削除は禁止です。