# Balancing — 二輪脚ロボット

**設計基準日：** 2026-10-27（仕様・TODO最終更新：2026-11-09）  
**状態：** MOTOR-146〜224の設計を仕様へ反映。C++17アプリケーション骨組みとC/C++ブリッジを追加しホスト検証済み。**RS06/C620のCAN符号化・復号と単軸短時間試運転、Issue #5の周期要求モデルをホスト試験済み**。PR #16のF446 HAL接続ソースはクロスコンパイル済みだが、HAL送受信への実行時接続は未実装。基板への書込み・実機回転・F446/F303統合・倒立は未実施。

## まず読む文書

1. [実装ロードマップ](docs/todo/roadmap.md) — P0固定脚長→P1可変脚長→P2ジャンプ、未完了の優先順位。
2. [ジャンプ制御（後続）](docs/specs/behavior/jump.md) — 5段階のジャンプ仕様（初期実装の対象外）。
3. [図で見る制御システム](docs/diagrams.md) — データフロー、ARM、制御周期、状態遷移、非常停止。
4. [CubeIDEのC++開発と自動生成](docs/design/cubeide-cpp.md) — main.cを残し、C++制御アプリを再生成の影響から分離。
5. [モータCAN制御層と単軸試運転](docs/design/motor-layer.md) — RS06/C620のC11符号化、HAL接続、試運転時の制限と手順。
6. [データ構造と所有権](docs/design/data-model.md) — 構造体、単位、変更権限、コードを増やしすぎない規約。
7. [実機非依存の制御コア](docs/software-core.md) — C11の責務分離、ARM判定、LQRとホスト試験。
8. [離散線形シミュレーション](docs/design/linear-simulation.md) — 2 ms・6状態の外部行列モデルとCSV出力。
9. [CAN ID・データ・単位の総覧](docs/can-id-table.md) — CAN1／CAN2の通信一覧。
10. [仕様ID索引](docs/spec-index.md) — 12件の仕様書への入口。
11. [ソフトウェア構成](docs/architecture.md) — タスク周期、責務分離と保護。
12. [AI開発規則](AGENTS.md) — 最小変更、テスト、仕様と実装の対応。
13. [結合・実機検証TODO](docs/todo/verification.md) — 未実装項目と完了条件。
14. [GitHub運用・初回登録手順](docs/development/git.md) — モノレポの管理対象、除外・ローカル検証、公開前確認。
15. [VS Code：Ruff・clangd](docs/development/editor.md) — 保存時整形・補完とローカル検査、STM32の解析用DB。
## ディレクトリ

| 配置先 | 内容 |
|---|---|
| `docs/specs/boards/` | F446／F303基板 |
| `docs/specs/protocols/` | CAN1／CAN2／USB／I2Cの通信契約 |
| `docs/specs/control/` | 状態推定、LQR、VMC |
| `docs/specs/behavior/` | ロボット操作と状態遷移 |
| `docs/todo/` | P0/P1/P2の未実装・未検証事項（`roadmap.md`が入口） |
| `firmware/f446/balancing_f446/` | F446のCubeIDEプロジェクト。自作ソースを`App/{Inc,Src}/balancing/`へ機能別に配置。PR #16の旧5ソースCubeIDEビルドPASS。新6ソース配置のCubeIDE管理ビルドと実機書込みは未検証。F303用FWは未実装 |
| `firmware/f446/balancing_f446/App/Inc/balancing/`・`App/Src/balancing/` | 自作C11/C++の公開ヘッダーと実装を機能別にミラー配置。詳しくは[配置検証](docs/verification/f446-module-layout.md) |\n| `firmware/f446/balancing_f446/App/Examples/` | Doxygenコメントの書式例 |
| `Doxyfile` | HTMLリファレンス生成設定 |
| .gitignore・.gitattributes | Gitの除外設定と改行コード統一。GitHub Actionsは当面使用しない |
| .vscode・.clangd・ruff.toml | Ruff・clangd・clang-formatの共有エディタ設定 |

## 検証と公開

以下をプロジェクトのルートで実行する。

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 tools/validate_specs.py
python3 tools/run_host_tests.py
doxygen Doxyfile
```

生成物：`build/docs/html/index.html`。同一LAN内の公開先：`http://192.168.73.129:8769/`。

## リポジトリ管理

`balancing`を1つのGitHubリポジトリにする方針。ファームウェア、今後追加するKiCad回路図・PCB、シミュレーション、文書をディレクトリで管理する。初回登録前の確認は[GitHub運用・初回登録手順](docs/development/git.md)を参照。**GitHub Actions・セルフホストCIは今回は設定しない**。

## 管理方針

- 定義済みの内容と実機確認待ちの内容を区別する。未確定のCAN IDや通信値を作り上げない。
- 初期は**固定脚長の基本バランシング**（MOTOR-213）。可変脚長・ジャンプ・自動起立・EKF・スリップ推定は後続。MOTOR-095とMOTOR-132の起立シーケンスは保留。
- MOTOR-151により**初期姿勢・脚長範囲はARM許可ゲートとしない**。ただし機械的可動限界と危険な指令への保護は維持する。
- `../balancing-spec/`は以前の資料として保存。**このディレクトリを以後の編集上の正本**とする。
- 一部のVM版文書は過去のv0.2資料から要約・再構成している。完全な原文一致は保証しない。

## 開発・GitHub運用

- `CONTRIBUTING.md`：Issue、PR、ブランチ、テスト、レビューの手順
- `docs/development/github-governance.md`：合意した23項目
- `docs/development/github-setup.md`：ラベル・Squash merge・master保護
