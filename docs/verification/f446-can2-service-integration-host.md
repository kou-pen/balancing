# F446 CAN2通信の非駆動統合

対象: PROTO-PWR-001 / PROTO-REMOTE-001 / BOARD-F446-001 / BEHAV-001、T-PROTO-05/06/09/11、T-F446-05/06。
基準: master `fa82ad38595c035cc4d7d6ce542d91817c541958`（#69統合後）。

## 実装した機能

| 変更箇所 | 役割 |
|---|---|
| Can2ServiceLink.hpp/.cpp | 既存サービスの参照、CAN2配送、独立停止入力、一周期の操作応答、期限・購読・設定エラー・電源応答照合 |
| Can2ServiceLinkTx.cpp | FAULT 3部分の符号化、終端で進捗、最新診断配信、期限付き応答と模擬電源要求 |
| RemoteControlService.hpp/.cpp | CAN motion復号、選択元・時刻・上限の既存判定、CAN欠落で旧入力と中立確認を無効化 |
| PowerBoardService.hpp/.cpp | #68の受信入口を利用。CAN欠落で電源readyを取り消し、最終値と保留FAULTは維持 |
| ApplicationObservation.hpp / application.cpp | 非駆動設定、操作・電源・FAULTの周期接続、状態と有効観測を診断フレームへ |
| f446_can1_runtime.cpp / balancing_f446_can1_bridge.h | 明示設定時のみ自作12 IDを含むフィルタ、実IRQ回収・受信配送・CAN2背景送信終端 |
| CanTxScheduler.hpp/.cpp | 背景は最大1受付枠。停止・通常・未受付周期が優先。背景の終端は周期投入の必須条件にしない |

## 試験の範囲

- `tests/test_f446_can1_runtime.cpp --can2-services`: 独立した固定ID・payloadでHAL模擬IRQ→Router→Application→RobotSequence→CAN2送信。設定なしの有効化拒否、7バンクの照合値、電源ONと採取時刻、Pi ARM権限、操作元切替、未検証モデルでのARM拒否、E-STOP最優先、FAULT 3部分と終端、解除不許可。
- `tests/test_f446_can_event_poll.cpp`: 実Router/CanPort/Schedulerとサービスのホストfixtureで、別バス、不正・旧入力、復帰中立、模擬ARM/DISARM/CLEAR_FAULT結果、電源要求・応答照合・送信失敗、欠落時の無効化、応答満杯時の停止優先、設定エラー・購読期限、最新診断への置換、FAULT失敗時再試行と3部分共通ID、C620周期と停止枠の優先を確認。
- 既定C ABI起動は旧受信専用動作のまま。試験値は人工設定で、現物の期限・制限・容量を表さない。

## 実測結果（2026-11-26）

| 検証 | 結果 |
|---|---|
| CMake Debug / CTest | 34/34 PASS |
| 通常ホスト・UBSan | 全件PASS（job eac8f827a61e427e、終了0） |
| CAN契約監査・CubeIDE設定回帰 | 19/19、22/22 PASS |
| CubeIDE 1.19.0 Debug / Release cleanBuild | 全46 Appソース、ARM ELF32、0エラー/0警告（job ea097a7b28984ad8、終了0） |
| 仕様・Doxygen・差分 | PASS（Doxygen既知の日本語翻訳警告のみ） |

ARM ELFのメモリ量 [B]:

| 構成 | text | data | bss |
|---|---:|---:|---:|
| Debug | 173476 | 92 | 11216 |
| Release | 60764 | 92 | 11216 |

コマンド: `cmake -S . -B build/can2-integration -DCMAKE_BUILD_TYPE=Debug`、`cmake --build build/can2-integration -j 6`、`ctest --test-dir build/can2-integration --output-on-failure`、`python3 tools/run_host_tests.py`、`python3 tools/validate_specs.py`、`doxygen Doxyfile`、`git diff --check`。ARMはCubeIDEの `-cleanBuild balancing_f446/Debug` と `-cleanBuild balancing_f446/Release`、`python3 tools/verify_f446_build.py`。ログ: VMの `balancing-input-integration/build/cubeide-headless-can2-integration/{Debug,Release}.log`。

## 未検証・残件

- 実機C ABIからは自作フィルタと背景送信を有効化しない。電源要求は単体模擬リンクのみ。実機操作、通常モータ送信、安全閾値、CubeMX生成コードは非変更。
- 未割当の状態/有効性ビットは意味を追加せず0。車輪角速度・周期超過累計のデータ源は未接続、fixtureOnlyの脚/車体状態はApplicationから送らない。設定番号別の値、状態遷移の診断IDは未決定。
- CAN IDは認証ではない。運動指令の生成時刻・再送重複、電源連番・再起動、応答番号周回、同時採取のTX/RX先後はワイヤだけでは証明できない。
- CLEAR_FAULT許可の実機判断、操作目標から位置基準・制御目標への接続、C620/RS06通常運転の制御経路は後続。バス上の実時間遅延、電源FW/Pi互換性、実機通信は未検証。
