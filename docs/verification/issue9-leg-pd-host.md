# Issue #9：固定脚長PD・Roll PDホスト検証

**対象：** 2026-11-11、`feature/issue9-leg-pd-roll-host`。仕様 CTRL-LQR-001、MOTOR-159/160/163/204〜209。

## 自動検証結果

| 実行 | 結果 | 限定事項 |
|---|---|---|
| `python3 tools/run_host_tests.py` | PASS（C11通常・UBSanを含む） | ゲイン・脚長・力は数学fixture |
| `cmake -S . -B build/host-cmake && cmake --build build/host-cmake --parallel 4` | PASS | ホストのみ |
| `cmake --build build/host-cmake --target test` | 11/11 PASS | 物理モデルの妥当性は評価しない |
| `python3 tools/validate_specs.py` | PASS | 仕様ID・参照・CAN一覧 |
| `python3 tools/regenerate_cubeide_f446.py` | Debug/Release PASS | CubeIDE管理クロスビルドのみ、実機動作ではない |

追加 `tests/test_leg_pd.c` は、測定時の左右差を維持するARM前ラッチ、目標一致時のPD段差ゼロ、実伸縮速度のみのD項、Roll左右逆符号、未検証ゲイン・NaN・Inf・非正脚長・オーバーフロー時のゼロ出力を確認します。既存のホスト回帰も再実行しています。

## 適用してはいけない値

試験中の脚長0.28〜0.32 m、Kp=100 N/m、Kd=10 N s/m、Roll Kp=20 N/rad、Roll Kd=2 N s/rad、重力補償5 Nは全て**人工fixture**です。実機の設定として使いません。

## NOT RUN / NOT VERIFIED

実六節リンクの幾何・支持力方向、左右軸符号、BNO055取付Roll座標、RS06×4の独立トルククリップ、実ガスシリンダ、状態機械のARM/FAULT結合、物理500 Hz/250 Hz、実機CAN・モータ駆動は未検証。従ってこのPRだけでIssue #9の受入条件を満たしたとは扱いません。
