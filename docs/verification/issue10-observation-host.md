# Issue #10：センサ・操作入力のホスト検証

**対象：** CTRL-EST-001、BEHAV-001、PROTO-C620-001、PROTO-REMOTE-001、T-CTRL-04/05、T-F446-05/06。  
**結果：** ホストの数学・時刻管理・入力復旧テストはPASS。実機BNO055・DualSense・CAN_REMOTE・モータとの結合は未実施。

## 検証済みの処理

| 処理 | 確認した条件 | 判定 |
|---|---|---|
| 500 Hz向けスナップショット | コピー後も値と元の測定時刻が固定。再読出しで時刻更新しない | PASS |
| C620車輪位置・速度 | 実CAN codecから復号、8192カウントの折返し、左右符号、20 ms期限 | PASS |
| IMU受信時刻 | 未証明の新規入力、重複、過去、未来、NaN/Infを拒否。50 ms期限 | PASS |
| USB/CAN操作元 | DISARMEDのみ切替、未選択ソースの指令を拒否。自動切替なし | PASS |
| 操作入力喪失・復帰 | 速度・Yaw目標をゼロへ減速。複数の新規ニュートラル入力を確認後に操作だけ再開 | PASS |
| 自動ARM禁止 | 操作入力からARM/CLEAR_FAULTイベントを生成しない。DISARMED/FAULTの目標は0 | PASS |
| BNO055数学フィルタ | 明示的な静止確認、ジャイロバイアス平均、取付軸の右手系、相補補正と角度±π境界 | PASS |
| IMUの異常・外乱 | 重力範囲外はジャイロのみ。新規時刻の異常、長い計測欠落、未校正を拒否 | PASS |
| 連結された模擬センサ・操作入力 | IMU測定→固定コピー→姿勢推定、C620の別周期更新、操作断とDISARMED状態機械 | PASS |

## 実行結果（2026-11-10、開発VM）

| チェック | 結果 |
|---|---|
| 通常・UBSanのC11ホスト試験 | PASS |
| CMake/CTest | **10/10 PASS** |
| CubeIDE Debug / Release | **10ソースのARMクロスビルドPASS**（両方0エラー・0警告） |
| CubeIDE生成Makefile・オブジェクト・ARM ELF32 | PASS |
| 仕様ID・文書リンク、Ruff、Doxygen | PASS（Doxygenの既知の翻訳警告のみ） |
| 実機I2C/USB/CAN、物理500 Hzと安全停止 | **NOT RUN** |

前回のPR #22では、観測スナップショットまでの **8ソース・CTest 7/7 PASS** を確認済みです。今回のホストモックと数学フィルタの2ソース、連結試験を追加して、10ソース・CTest 10/10となりました。ビルドログはGit管理外の `build/cubeide-managed-pr16/{debug,release}.log` にあります。

## 再現コマンド

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
python3 tools/regenerate_cubeide_f446.py
python3 tools/verify_f446_build.py
python3 tools/validate_specs.py
doxygen Doxyfile
```

## 未確定事項と安全境界

| 未確定の項目 | 必要な確認 |
|---|---|
| BNO055のI2C/ACCGYRO新規測定判定、取付方向、静止判定 | 実センサと実装する受信アダプター |
| DualSenseのHID report・ボタン番号・実デッドゾーン | 実デバイスのレポート解析 |
| CAN_REMOTEのID/DLC・ペイロード・送信元認証 | 既存の上位機器ソースとCAN仕様 |
| 車輪半径・減速比・左右の符号 | CADと実機 |
| RS06×4・F303電源を含む全観測ストア | 実ドライバと時刻付きデータ更新 |
| IRQとの共有・操作元切替の実シーケンス・物理500 Hz | 実機/HAL結合後の試験 |

姿勢フィルタの角度予測は、ジャイロ機体X/Y軸の角速度をEuler Roll/Pitch微分に近似したものです。大姿勢やYawとの結合が強い運動へはこのまま適用せず、実機への統合前に運動学の妥当性を検証します。

**ホストで使用した数値は人工fixtureです。** 例として車輪半径0.1 m、ロータ対車輪比10:1、操作入力期限10 ms、ニュートラル継続4 ms、ジャイロ相補係数0.9を使用しました。これらを実機の制御・安全値へ転記しません。

[観測スナップショット](../design/observation-snapshot.md) · [操作入力](../design/operator-input.md) · [姿勢フィルタ](../design/attitude-filter.md)

**Issue #10のホスト検証部分は進展しましたが、実機HAL/USB/I2C統合と物理校正を残してOpenを維持します。**
