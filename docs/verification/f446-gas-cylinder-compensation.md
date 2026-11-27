# F446 ガスシリンダ補償インターフェース

基準 master b3f8dd2、CTRL-LQR-001 / MOTOR-160〜162,207 / T-CTRL-09 / Issue #9。

## 実装

既存のBalanceControl.hpp/.cppのみに製品ソース変更を限定。GasCylinderCompensationをBalanceControlConfig末尾に追加（未装着、未検証、関数nullが既定）。設定は構築時コピー。装着かつ未検証/関数なしならinitializeを拒否する。未装着はコールバックを実行しない。

computeで同一周期ControlSnapshotを読み取り専用で外部純粋数学関数へ渡す。左A/B・右A/Bの機構軸補助トルク[N m]を全4軸出力し、VMCの出力から一回差し引く。欠落をNaNで検出、非有限値と差引後のfloat範囲外を拒否。失敗時reset/nullopt、既存ControlCycleがExternalFaultとStopAllを当該周期に適用する。MotorCommandの変換・個別制限、CAN ID、診断フレーム、実機駆動許可は変更しない。

## ホスト試験

test_f446_sil_control_cycle.cppで未装着互換、装着時の正負補助トルク、車輪不変、既存のRS06軸別制限、未検証/関数なし、部分出力、不正数値、差引オーバーフロー、計算成功後の次周期失敗と停止、FAULT保持を確認。人工値のみ使用。CTest 79/79（962a8c508be14b3d）。通常/UBSan全件PASS（dbd41bf92d994d06）、CubeIDE設定25/25・CAN契約19/19。CubeIDE管理clean-build Debug/Release/BenchDebug全構成0 error/0 warning、ARM ELFモード分離PASS（78d3c9e222824423）。仕様参照532件、Doxygen生成（既知の日本語翻訳警告）、diff check合格。

## 残件

実機のガスシリンダ力曲線・取付点・関節座標・機構可動域から補助トルクを求めるモデルは未実装。モデルに追加の観測が必要かはCAD確認後に決める。コールバックの純粋性・実時間上限は外部モデルの契約であり、このIF単体では保証しない。実機書込み、通電、駆動、物理特性測定は未実施。
