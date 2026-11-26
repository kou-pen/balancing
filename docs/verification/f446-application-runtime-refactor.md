# Application実行境界の分離

対象: BOARD-F446-001 / BEHAV-001 / T-F446-07〜10。基準 master 49ea4ef。目的はApplicationとCANランタイムから試験ビルドの条件を除去すること。既存の制御アルゴリズム、駆動許可条件、CAN ID、周期、停止判定は変更しない。

| ファイル | 変更 |
|---|---|
| App/Inc/balancing/app/DriveRuntime.hpp | 共通周期実行、設定、CAN所有者結合、終端通知の契約 |
| App/Src/balancing/app/DriveRuntime.cpp | 非駆動のControlCycleのみを呼ぶF446実装 |
| App/Src/balancing/app/application.cpp | 観測・操作・実行・診断・表示の順を共通化 |
| App/Inc/balancing/app/ApplicationObservation.hpp | 試験固有設定と結果を分離 |
| App/Src/balancing/device/f446_can1_runtime.cpp | 共通の結合・終端通知 |
| tests/sil/DriveCycle.hpp/.cpp | 既存の模擬CAN駆動を移動。ARM/Thumbでの誤コンパイルは明示エラー |
| tests/sil/DriveRuntime.cpp / DriveSil.hpp | 人工設定、模擬駆動の所有、試験結果の取得 |
| CMakeLists.txt / tools/run_host_tests.py | ターゲットごとに実装をリンク。SILマクロを削除 |
| tests/test_drive_runtime_boundary.py | App側への試験型・試験用マクロ再混入の静的回帰検査 |

## 検証状況

当初VMの ping、hosts、run_command が内部エラーを返したため、GitHubのGitデータAPIで独立コミットを作成。その後VM接続が復旧し、専用worktreeに取得して以下を実行した。masterと既存#72/#73は変更しない。

受入条件:
- Application.cpp / ApplicationObservation.hpp / f446_can1_runtime.cpp にSIL専用プリプロセッサ条件なし。
- python3 tests/test_drive_runtime_boundary.py、CMake/CTest、python3 tools/run_host_tests.py（通常/UBSan）全件合格。
- CubeIDE 1.19.0 Debug/Release cleanBuild、python3 tools/verify_f446_build.py、ARM ELFと生成objects.listにSIL実装が存在しないことを確認。
- python3 tools/validate_specs.py、Doxygen、git diff --check。
- #72/#73をマージする場合は共通Application変更を統合し、両機能を含めて全件再検証する。

## 実測結果（2026-11-26）

| 検証 | 結果 |
|---|---|
| ソース境界 | PASS、App全体からSILマクロと型を排除 |
| CMake / CTest | 44/44 PASS、job 2cd0b63b96a5477f、終了0 |
| ホスト通常 / UBSan | 全件PASS、job e521cfed6f9c45a3、終了0 |
| CAN監査 / CubeIDE設定 | 19/19 / 22/22 PASS |
| CubeIDE 1.19.0 Debug / Release cleanBuild | 全47 Appソース、ARM ELF32、0エラー・0警告、job 7982b0dad8084950、終了0 |
| ARM ELF / objects.list | 両構成で試験用型・関数シンボル、DriveCycleオブジェクト、tests/silソース不在 |
| 仕様 / Doxygen / diff | PASS（既知の日本語翻訳警告のみ） |

Release text=61308 B、data=92 B、bss=11240 B。実機駆動は未実施。未マージ#72/#73との統合は別ローカル検証ブランチで扱う。
