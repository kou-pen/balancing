# AI用の文書ルータ

このファイルは**AIエージェント専用の索引と編集契約**です。人間向け説明の代替ではなく、仕様・TODO・実装・検証記録の正本を再定義しません。ここに進捗の固定コピーは置かず、作業開始時にGitHub Issue/PRとGitの現行状態を確認します。

```yaml
schema: balancing.ai-doc-router/v1
audience: agent_only
entrypoint: AGENTS.md
authority:
  requirements: docs/specs/
  spec_id_registry: docs/spec-index.md
  protocol_payloads: docs/specs/protocols/
  can_summary_only: docs/can-id-table.md
  acceptance_criteria: docs/todo/
  decisions: docs/development/github-governance.md
  code_behavior: firmware/f446/balancing_f446/App/
  evidence: docs/verification/
  live_work_state: GitHub Issues and Pull Requests
routing:
  architecture:
    - docs/architecture.md
    - docs/diagrams.md
    - docs/specs/boards/f446_control.md
  control:
    - docs/specs/control/
    - docs/specs/behavior/robot_operation.md
    - docs/software-core.md
    - docs/verification/core-mode-contract-audit.md
    - docs/design/leg-kinematics.md
    - docs/verification/issue7-kinematics-host.md
  can:
    - docs/specs/protocols/
    - docs/design/motor-layer.md
    - docs/design/can-scheduler.md
    - docs/verification/issue5-can-scheduler-host.md
    - docs/verification/can-contract-audit.md
  stm32:
    - docs/design/cubeide-cpp.md
    - docs/verification/f446-module-layout.md
  roadmap:
    - docs/todo/roadmap.md
    - docs/todo/f446.md
    - docs/todo/control.md
    - docs/todo/verification.md
  github:
    - CONTRIBUTING.md
    - docs/development/github-governance.md
validation:
  spec_links: python3 tools/validate_specs.py
  can_contract: python3 tools/check_can_contract.py
  host_and_ubsan: python3 tools/run_host_tests.py
  cubeide_metadata: python3 tools/sync_cubeide_cpp.py
  host_cmake: cmake -S . -B build/host-cmake
  host_ctest: cmake --build build/host-cmake --target test
  documentation: doxygen Doxyfile
  cubeide_artifacts: python3 tools/verify_f446_build.py
status_vocabulary:
  - PASS
  - FAIL
  - NOT_RUN
  - NOT_VERIFIED
```

## 文書編集の機械向け契約

| class |対象 | 編集規則 |
|---|---|---|
| human_overview | README.md、docs/index.md、docs/architecture.md、docs/todo/roadmap.md、docs/verification/index.md | 結論を最初に。図か表で関係を示し、詳細はリンク。大量の箇条書きや検証ログを貼らない |
| human_reference | docs/design/、docs/diagrams.md、CONTRIBUTING.md | 参照できる見出し、表、短い段落。専門情報を省略せず、全体像から段階的に開示 |
| canonical_contract | docs/specs/、docs/todo/、docs/development/github-governance.md | 完了条件と安全・設計の意味を保持。見た目の改善と仕様変更を同一視しない |
| audit_evidence | docs/verification/（READMEを除く） | コミット、環境、コマンド、実測値、合否、未検証事項を残す。長くても証拠を削らない |
| agent_only | AGENTS.md、docs/ai/ | 短い識別子・キー・参照先・検証コマンドを優先。正本から重要な規則を無断でコピー・改変しない |

文書修正時は、リンク・Doxygen・仕様ID整合を検証します。仕様の変更がない文章整理では、制御ロジック、閾値、受入条件、GitHub Issueの状態を変更しません。