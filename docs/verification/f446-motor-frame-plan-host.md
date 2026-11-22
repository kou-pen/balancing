# 6軸CANフレーム構築・2バス優先順位（SIL）

**対象:** PROTO-RS06-001 / PROTO-C620-001 / BOARD-F446-001 / BEHAV-001、Issue #5/#11/#27。  
**ベース:** PR #48統合後の `master` `d176de3`。ハードウェアには接続しない。

## コードを読む順番

1. `MotorFramePlan::compose()` — `MotorCommandPreview` から既存C11 `b_rs06_torque_frame()`（4件）と `b_c620_current_frame()`（1件）を使いCANフレーム値を作る。RS06の宛先番号は4軸分を構築時に固定し、ゼロ・重複を拒否する。実機IDは補完しない。
2. `test_f446_motor_frame_plan.cpp` — 正常なType 1の4軸のID・量子値・Kp/Kd=0、C620 `0x200` の左右電流と未搭載ID3/4のゼロ、停止・ARM中の出力抑止、後続軸の符号化失敗時に部分出力がないことを確認する。
3. `test_f446_can_event_poll.cpp` の `testMotorFramePlanCan1Can2StopPriority()` — 上記の5件を模擬CAN1/2 `CanTxScheduler` に入れ、停止の優先、旧周期フレームの破棄、停止完了後の旧トルク不復活、CAN2のゼロ電流への世代切替を確認する。

## 設計上の境界

`MotorFramePreview` は **fixtureOnly=true固定**。送信API、ARMセッション変更、HAL割込み呼出し、モータサービスの駆動許可は持たない。CANデータが正しく符号化できたことは、モータを実機で駆動できることを意味しない。

| 状態機械の出力 | RS06 Type 1 | C620 0x200 | RS06 Type 4 |
|---|---|---|---|
| `Balancing/DriveAllowed` かつ有効な数学プレビュー | 4軸分を作る | 左右の制限済み電流、ID3/4はゼロ | この層では作らない |
| `Disarmed/StopAll`、`Fault/StopAll` | 作らない | 全チャンネルゼロ | 既存 `MotorCanLink` が管理 |
| `Arming/EnableRs06` | 作らない | 全チャンネルゼロ | 既存 `MotorCanLink` が管理 |
| 不正な駆動要求・設定・符号化失敗 | フレーム集合を空にする | フレーム集合を空にする | 既存 `MotorCanLink` が管理 |

停止は専用の登録枠とCAN送信終端を持ち、既存 `MotorCanLink::submitPending()` が一元管理する。このクラスに停止キューを新設して二重管理しない。`CanTxScheduler::enqueueStop()` は**未受付の**旧周期Type 1を破棄する。一度HWメールボックスが受付済みのType 1は、ソフトウェア上では取り消せないため終端まで追跡する（実機の停止遅延は未検証）。

RS06の通常Type 1は4 ms毎（250 Hz）、C620の共通0x200は2 ms毎（500 Hz）。`MotorFramePlan` は周期タイマー・世代番号・期限を保持しない。今周期に何を符号化するかだけを担当し、送信周期・期限切れ・停止優先・受付済みトークン管理は既存の `CanTxScheduler` に残す。

## 実施したテスト

- 既存C11コーデックと完全一致する5フレームのID、DLC、符号化値、RS06のKp/Kd=0、未搭載C620 ID3/4のゼロ
- `Disarmed/StopAll`、`Arming/EnableRs06`、`Fault/StopAll` でRS06 Type 1を一件も生成しない
- 偽のDriveAllowed、ARMセッション番号ゼロ、数学プレビューの無効化、4軸IDの重複・ゼロ、後続4番目のRS06失敗、C620の不正電流で**部分フレームなし**
- CAN1の未受付Type 1がSTOPで消え、Type 4が優先され、STOP送信完了後に古いType 1が再び待機列へ現れない
- CAN2の周期生成番号を1→2に切り替え、非ゼロ0x200のTx終端回収後はゼロ電流0x200を登録する

### 実行記録

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
doxygen Doxyfile
```

- CMake/CTest：**22/22 PASS**。通常ホスト・UBSan：**PASS**（ホスト試験の全件合格とUBSan終了コード0）。
- 実 STM32CubeIDE Debug/Releaseクロスビルド、ChromiumでのMermaid SVG描画：**未実施**（設定監査・生成HTMLとは別の検証）。
- 実機CAN1/2、メールボックス順序、実通信負荷、実モータID・電流・トルク限界・非常停止：**未検証**。

**禁止している実機変更:** `Application::controlTick()`、本番の `MotorService::applySequence()` と `MotorCanLink::submitPending()`、CAN1/2 HAL起動、非ゼロ指令のスケジューラ接続、実機安全閾値、電源・機構CADは変更しない。
