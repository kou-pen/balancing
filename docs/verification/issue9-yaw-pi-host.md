# Issue #9：Yaw PIホスト検証

**実行日：** 2026-11-11。ブランチ `feature/issue9-yaw-pi-host`。仕様 CTRL-LQR-001／MOTOR-201/202。

| 試験 | 結果 |
|---|---|
| `python3 tools/run_host_tests.py` | PASS（既存ホスト回帰＋Yaw通常・UBSan） |
| `cmake -S . -B build/host-cmake` とビルド | PASS |
| `cmake --build build/host-cmake --target test` | 11/11 PASS |
| `python3 tools/regenerate_cubeide_f446.py` | CubeIDE Debug/Release管理ビルド両方PASS |
| `python3 tools/validate_specs.py` | PASS |

`tests/test_yaw_pi.c` は、積分の初回値・上限、左右モータの倒立共通トルク優先、残容量0での出力0、飽和悪化方向の積分停止、指令反転時の積分解消、DISARM/FAULT相当の非BALANCING時リセット、ジャイロ鮮度欠落、未検証ゲイン、NaN、無効な時間間隔を検証します。

車輪上限4.5 N m、倒立合計8 N m、Yaw差動余力0.5 N m等の数値は**人工fixture**です。制御ゲインも架空であり、機体への実装値として使いません。

**NOT RUN：** 実BNO055 Yaw角速度符号・鮮度、DualSense操作系、実機K_300とのLQR配分、左右M3508電流上限、実機500 Hz、CAN送受信、モータ駆動、物理非常停止。状態機械との実配線も未統合です。Issue #9はOpenを維持します。

PR #24の脚長PD/Roll PDとは独立した検証であり、両方を統合したSILを実施したとは主張しません。
