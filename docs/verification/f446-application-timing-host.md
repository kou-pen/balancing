# F446 Applicationの共通時計・周期診断

仕様：BOARD-F446-001。対象：T-F446-03/05/07、Issue #11/#27。PR #50統合後のmaster（`d719ca6`）を基準とする。

## 処理順

既存のmainはTIM7の要求を受け、主ループでCAN1イベント回収→Application周期処理の順に呼ぶ。生成コードは変更しない。

- `f446_can1_runtime.cpp`：既存DWTの64bit積算時計で、イベント回収後の評価時刻を`BF446Can1Snapshot.sampled_at_us`へ保存する。最後のRX時刻とは区別する。採取前は`sampled=false`。
- `Can1Observation.h`：CAN1監視コピーのC契約をHAL起動用ヘッダーから分離する。ApplicationはHALの型に依存しない。
- `application.cpp`：周期ごとに監視コピーを取得し、前回の正常な採取時刻との差をµsで記録する。初回、取得不能後の初回は時刻のみ有効。重複・逆行では公開する時刻・差分を無効とし、正常な基準を巻き戻さない。取得不能時は差分基準を破棄する。
- `balancing_app_timing_snapshot()`：主ループから診断コピーを取得する。カウンタは処理回数であり、2msの実測を保証しない。

## 受入試験

| 試験 | 確認内容 |
|---|---|
| Applicationの単体試験 | 起動前・採取前・時刻0の初回・2ms/6ms間隔、重複・逆行、取得不能と復帰、再初期化 |
| CAN1ランタイムとの結合 | 既存HALスタブを用い、RXと周期評価が共通時計を使用すること、DWT周回、RXのない2ms区間、IRQマスク復元、全送信メールボックスが非要求であること |
| 既存回帰 | CMake/CTest 22件、通常ホスト・UBSan |

DWTの32bitカウンタは少なくとも1周未満の間隔でサンプリングする必要がある。168MHzでは約25.6秒で周回する。複数周をまたぐ停止、デバッガによるCPU停止、実機クロック・周期ジッタはこの試験で保証しない。周期値は回収完了時刻間隔であり、タイマISRの発生間隔ではない。

## 非変更範囲

ControlCycleの実機呼出、CAN2の起動、IMUの新規測定、RS06軸校正、ARM、モータ送信は未接続。6msの診断値を観測できても、この段階では新たなFAULT判定を作らない。Core/Drivers、既存安全閾値、実機設定を変更しない。

## 実行記録

- CMake configure/build、CTest：**22/22 PASS**。
- `python3 tools/run_host_tests.py`：**PASS**（通常・UBSan）。
- `python3 tools/validate_specs.py`：**PASS**（仕様12件、TODO7件、API参照221件）。`python3 tools/sync_cubeide_cpp.py`：**PASS**（設定監査）。`doxygen Doxyfile`：**PASS**（既知の日本語翻訳警告のみ）、新ヘッダーとdeviceフォルダリンク、Mermaid HTML17ブロックを確認。ブラウザSVG描画は未実施。
- ARM Debug/Releaseクロスビルド、実機測定：未実施。
