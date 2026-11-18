# RobotSequenceのホスト検証

**実行日** 2026-11-18、ooedo-vm、GCC/G++ 11.4.0。基準コミットは `c9f005b`。対象仕様は [BEHAV-001](../specs/behavior/robot_operation.md) と [BOARD-F446-001](../specs/boards/f446_control.md)。関連する決定はMOTOR-151・197・198です。

## 検証した契約

| 受入条件 | ホスト試験 |
|---|---|
| 検証済み期限と駆動前提 | 期限未検証・ゼロ・加算範囲外・停止未完了を拒否。初期姿勢の診断範囲は状態機械の入力と独立させ、機構安全が未確認の間は4軸確認済みでも `ARMING` で非ゼロ駆動を保留 |
| 現在のセッションの全4軸 | 旧セッションの確認・失敗を現在のセッションに流用しない。3軸確認で待機し、4軸と機構安全を期限前に確認して `BALANCING` へ遷移 |
| 絶対期限 | 送信待ちで期限を延長しない。4軸確認や手動DISARMと同時でも、期限ちょうど・期限後は `FAULT` と停止要求 |
| 停止とFAULT | 安全異常のない手動DISARMは `DISARMED`。必須条件消失・確定故障・時計逆行は同じ評価で `FAULT` と `StopAll`。運転中の機構安全消失と上位からの周期異常も停止 |
| 異常記録と回復 | 主原因は非常停止を優先。同時異常と後続異常を別フラグに保持。明示解除しても同周期のARMを処理せず、次のARMで新セッションを発行 |

仮想時計とRS06の有効化結果を入力しています。タイムアウトの試験値はホスト専用です。Type 3の実Tx完了と後続応答の照合、Type 4の軸別送信、待機指令の無効化は、MotorServiceとApplicationの接続後に検証します。

## 実行結果

- `python3 tools/run_host_tests.py` — PASS。RobotSequenceの通常・UBSanと既存のホスト回帰。
- `cmake -S . -B build/robot-sequence-debug -DCMAKE_BUILD_TYPE=Debug` と `cmake --build build/robot-sequence-debug -j 4` — PASS。
- `ctest --test-dir build/robot-sequence-debug --output-on-failure` — PASS、13/13件。RobotSequenceの対象テストはReleaseでもPASS。
- `python3 tools/validate_specs.py` — PASS。仕様ID、文書リンク、API参照を照合。
- Doxygen 1.9.8 — PASS。RobotSequenceクラスと本ページの生成HTMLを確認。
- F446クロスビルドと実機試験 — NOT RUN。今回の検証はホスト上の状態機械とモック入力を対象とする。
