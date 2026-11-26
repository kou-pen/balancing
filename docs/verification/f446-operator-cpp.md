# 操作入力のC++統合

基準 master c931001。BEHAV-001 / PROTO-REMOTE-001 / PROTO-PS5-001。旧C11操作モデル・変化率制限をRemoteControlServiceへ統合し、二重の状態モデルを解消する。Appのファイル数107→105、Cソース8→7。

| ファイル | 変更 |
|---|---|
| app/RemoteControlService.hpp/.cpp | 操作型・private状態・設定検証・中立復帰・変化率・CAN受信を単一所有 |
| app/balancing_operator_input.h/.c | 廃止 |
| control/controller/balancing_core.h/.c | 操作専用のb_rate_limitを移設。他の数値演算と旧状態機械は今回不変 |
| ControlCycle / ApplicationObservation / DualSenseService / Can2ServiceLink / application | C++型と明示enum classへ更新。CAN ID、USB形式、物理設定は不変 |
| tests/test_operator_input.cpp | 旧Cの全7試験を公開C++ APIへ移行、浮動小数点オーバーフローを追加 |
| tests/test_f446_remote_control_service.cpp | 旧C比較器を廃止し直接期待値、現行RobotSequenceとの非自動ARM確認 |
| tests/test_issue10_flow.c | 旧C観測・姿勢試験は維持、操作とモードの連結確認は現行C++試験へ移行 |

## 保存した動作
設定未検証で無効。切替はDISARMEDのみ。受信の時刻重複・過去/未来・非有限値・未選択入力を拒否。期限ちょうどは鮮度有効、1us超過は無効。中立は2件以上の新規報告と継続時間で確認、欠落時は中立確認をリセット。周期時刻が重複しても二重加速しない。逆行時は結果なし。ARMING/DISARMED/FAULTの目標は即ゼロ、BALANCINGのリンク断では変化率制限でゼロへ戻す。旧演算のfloat積・差分・飽和・非有限値拒否を保持する。実機閾値や自動ARM条件を追加しない。

旧Cの直接API（b_operator_*、b_rate_limit）は廃止し、型名はC++向けに変更した。フィールド名は無関係な差分を避けるため保持。過去の検証記録は当時の事実。実機・通信・周期実測は未実施。

## 検証
CTest 50/50 PASS（5655d8a01f1f4ea1）。
通常/UBSan全件PASS（a1d0e344953c4595）、CubeIDE Debug/Release全49 Appソース、0 errors/0 warnings（7447597ce0354c84）。Release text=69656, data=2828, bss=12216 B。CAN監査19/19、CubeIDE設定22/22、仕様465参照・Doxygen全sourceリンク・diff check PASS。両ARM ELFに旧操作C関数/SIL/試運転のシンボルなし。
