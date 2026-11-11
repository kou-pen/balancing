# 検証記録の案内

**PASSは、その条件で実際に検査した範囲だけを意味します。** ホスト試験、ARMクロスビルド、実機動作を同じ「完成」として扱いません。

## どの結果を見ればよいか

| 目的 | 記録 | 確認できた範囲 |
|---|---|---|
| F446の11ソース構成 | [Issue #9脚長・Rollホスト検証](issue9-leg-pd-host.md) | Debug/Releaseで新しい脚長PDソースを含むARMクロスビルドPASS（実機未実施） |
| F446の直前10ソース構成 | [Issue #10ホスト・ARM検証](issue10-observation-host.md) | PR #23時点の10ソース、実機未実施 |
| CAN周期と送信調停 | [Issue #5ホスト試験](issue5-can-scheduler-host.md) | 仮想時刻・3メールボックス・故障注入 |
| 単軸試運転の受信期限 | [CANポート集約](motorbench-rx-owner.md) | ベンチの重複期限除去とポート境界の故障注入 |
| CAN仕様の整合とID重複 | [Issue #6 CAN静的照合](can-contract-audit.md) | RS06/C620の既知フレーム、F303/Piの未確定項目 |
| ARM・FAULT判定の差分 | [状態機械の監査](core-mode-contract-audit.md) | ホスト回帰PASSと既知の仕様不一致 |
| 脚運動学の数値演算 | [Issue #7数学fixture](issue7-kinematics-host.md) | 解析ヤコビアンと有限差分、実機CAD未検証 |
| 脚長保持・Roll差動 | [Issue #9ホスト検証](issue9-leg-pd-host.md) | ARM前ラッチ、PDの単位・符号、異常入力拒否 |
| センサ・操作入力・IMUフィルタ | [Issue #10ホスト検証](issue10-observation-host.md) | C620折返し、IMU静止バイアス、操作元・通信断復帰 |
| 過去のCubeIDE配置 | [5ソース配置の検証](f446-source-layout.md) | PR #16時点の構成と再生成結果 |
| 初期のF446クロスビルド | [2026-11-09の記録](f446-crossbuild-20261109.md) | 統合前のC++骨組み |

新しい構成の判断には、一番上の**現行11ソース版**の検証記録を使います。過去のビルド記録は当時の事実として残し、現在の検証結果へ読み替えません。

## 検証状態の区別

| 表記 | 意味 |
|---|---|
| PASS | 記載された環境と対象で試験成功 |
| FAIL | 試験実行後に失敗 |
| NOT RUN / NOT VERIFIED | 未実施、または合否に必要な証拠がない |

## 現時点の境界

制御コア・CAN通信・送信要求モデル、運動学fixture、脚長PD/Roll PDの**ホスト試験**、F446の**11ソースCubeIDE Debug/Release管理ビルド**は2026-11-11にPASSしました。一方、実機書込み、CAN実通信、トルク出力、実時間性能、独立非常停止、実機での倒立は未検証です。

次の試験を計画するときは [実機検証TODO](../todo/verification.md) の受入条件を参照し、作業上の最新状態はGitHub Issue/PRで確認します。
