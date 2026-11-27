# F446 Application通常運転のSIL統合

対象: BOARD-F446-001 / BEHAV-001 / CTRL-LQR-001 / PROTO-RS06-001 / PROTO-C620-001。
TODO: T-F446-03/04/08〜11、T-CTRL-11。基準: master `8629cf1ebf3bc66246f7963375bc3b6969fe9ec5`（#70統合後）。

## 実装

| ファイル | 役割 |
|---|---|
| DriveCycle.hpp/.cpp | 既存制御周期と4軸有効化、指令計画、両CAN送信調停を結合。最新周期への置換、軸別送信完了期限、停止時抑止 |
| ControlCycle.hpp/.cpp | 今周期の推定位置・実測dtと操作速度から位置基準の候補を作る。停止速度0では現在位置を使用。従来の明示参照も維持 |
| ApplicationObservation.hpp / application.cpp | ホストSILプロファイルと周期結果。CAN2操作/電源、既存状態機械、DriveCycleと診断を接続 |
| f446_can1_runtime.cpp / MotorService.hpp | 実際の模擬CAN所有者と照合済み終端の配送。軸IDはMotorServiceの設定を唯一の参照先とする |
| CanTxScheduler.hpp/.cpp | 旧周期の未受付分を破棄し、受付済み周期へ中止要求。終端未確認の同軸停止は待ち、他軸の停止は独立 |

## ホスト試験

`tests/test_f446_can1_runtime.cpp` の新しい10モードは同一のApplication・HALレジスタ模型・CanRouter/CanPortで実行する。実機CAD、電流、トルク、通信期限ではなく人工fixture。

| モード | 観測した受入結果 |
|---|---|
| --drive-sil | 起動の4停止/ゼロ、選択元のARM、3+1軸Type3完了後の新規RX、遷移周期ゼロ、通常6周期にRS06 12/C620 6フレーム、通信断→減速・現在位置基準→中立復帰、旧指令中止・4軸停止/ゼロ |
| --drive-sil-timeout | C620のRXが正常でも送信完了喪失でFAULT |
| --drive-sil-rs-timeout | C620完了・RXが正常でもRS06送信完了喪失でFAULT |
| --drive-sil-stale | 置換後の旧C620正常完了は最新送信鮮度を更新しない |
| --drive-sil-tx-error | 確定送信エラーで出力抑止、次周期FAULT |
| --drive-sil-estop | CAN2非常停止が通常指令に優先 |
| --drive-sil-gap | 新規CAN観測を保ったまま6msの制御欠落で停止 |
| --drive-sil-rx-loss | 車輪報告の連続性欠損で推定不成立・FAULT |
| --drive-sil-invalid | 1軸の換算未検証でARM拒否 |
| --drive-sil-invalid-model | LQR未検証でARM拒否 |

`tests/test_f446_can_event_poll.cpp` は旧周期の中止要求・別軸停止の先行・同軸の終端待ち・旧未受付分の非復活を追加。既存CAN2背景送信と周期送信の優先関係も回帰する。

## 実測結果（2026-11-26）

| 検証 | 結果 |
|---|---|
| CMake / CTest | 44/44 PASS（追加10モード） |
| 通常ホスト / UBSan | 全件 PASS、job 9e1ccead9d2e4bfd、終了0 |
| CAN契約監査 / CubeIDE設定 | 19/19、22/22 PASS |
| CubeIDE 1.19.0 Debug / Release | 全47 Appソース、ARM ELF32、両方0 errors/0 warnings、job fff645f701fd44ee、終了0 |
| ARMシンボル確認 | DriveCycle.oとELFにDriveCycle、initializeDriveSil、bindDriveSilRuntime、driveSilTerminalのシンボルなし |
| 仕様 / Doxygen / 差分 | PASS（Doxygenの既知日本語翻訳警告のみ） |

メモリ使用量 [B]: Debug text=175108/data=92/bss=11240、Release text=61004/data=92/bss=11240。

再現: `cmake -S . -B build/drive-cycle -DCMAKE_BUILD_TYPE=Debug`、`cmake --build build/drive-cycle -j 6`、`ctest --test-dir build/drive-cycle --output-on-failure`、`python3 tools/run_host_tests.py`。ARMはCubeIDE headless cleanBuild Debug/Releaseと `python3 tools/verify_f446_build.py`、arm-none-eabi-nm -Cで上記シンボル不在を確認。VMログ: `build/cubeide-headless-drive-cycle/{Debug,Release}.log`。仕様検査・Doxygen・差分チェックも実行。

## 限界

- `BALANCING_HOST_SIL` と非ARM/Thumb条件の両方でのみ通常運転結合をコンパイル。ARMの既定経路は非駆動、fixtureOnlyは解除しない。
- 送信中止要求は物理的停止を保証しない。対応軸の旧指令終端が返らない場合はその軸の停止送信が待機する。実機の中止・復旧挙動、独立電源遮断、IWDGは別途検証が必要。
- 送信/予約異常を周期確定後に検出した場合は出力を直ちに停止方向へ抑止し、FAULTの状態遷移は次のControlCycleに委ねる。状態機械を二重更新しない。
- 位置基準の更新則は候補であり実機ダイナミクス・K_300による性能検証なし。実際の500Hz処理時間・CAN負荷、USB実入力、実機の安全閾値、CLEAR_FAULTの実機許可、物理モデルと通常駆動許可は未実施。


## 後続の構造整理

上記は #71 時点の検証記録。実装配置とビルド選択を変更する後続作業は [Application実行境界の分離](f446-application-runtime-refactor.md) を参照。過去のPASSを新構成のPASSとして扱わない。

## 2026-11-27：IMU / F303通信途絶の独立統合試験

基準 master d88735f、Issue #11、T-VERIFY-07。製品ソース・設定・閾値は変更せず、既存test_f446_can1_runtime.cppの模擬IRQ/CAN/送信終端/通常運転SILに2ケース追加した。

| 試験 | 入力の停止 | 評価結果 |
|---|---|---|
| --drive-sil-imu-expiry | 最後の姿勢採取時刻を固定、50 msまで新規姿勢なし | 期限前はBALANCING、期限ちょうどはImuUnavailable・InterlockLost・FAULT |
| --drive-sil-power-expiry | POWER_STATUSだけを停止、最後のCAN採取時刻から500 ms | 期限前はBALANCING、期限ちょうどは電源ready/fresh=false、推定成立のままInterlockLost・FAULT |

両ケースとも他のセンサ・モータRX/TX終端・操作・制御周期を維持。期限到達時に新規非ゼロ指令なし、RS06 Type4 4軸とC620 0x200ゼロ1件を模擬送信終端まで確認。復旧後もFAULTを保持し自動ARMしない。電源報告は周期の20us前に採取されるため最後の周期だけ20us短縮して期限ちょうどに合わせた。IMUは姿勢供給境界の模擬であり、50ms鮮度評価を模擬側で実装しており、BNO055実機の取得/新規性検証ではない（実装の鮮度判定は別のIMU単体試験で確認）。電源期限は実製品PowerBoardServiceを経由。

CTest 81/81（bac38e5e08b54de2）、通常/UBSan全件PASS（786ec183a7c64625）、CubeIDE設定25/25、CAN監査19/19。仕様参照532件、Doxygen生成（既知の日本語翻訳警告）・diff check合格。新規2ケースをCMakeと通常/UBSanホスト実行器へ登録。製品ソース変更なしのためARMビルドは再実行していない。物理通信・停止・WCETは未実施。実機用パラメータと運転許可を変更しない。
