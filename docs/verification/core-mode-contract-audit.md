# 制御コア状態機械の仕様差分監査

**検証日:** 2026-11-09  
**対象:** `firmware/f446/balancing_f446/App/Inc/balancing_core.h`、`firmware/f446/balancing_f446/App/Src/balancing_core.c`、`tests/test_core.c`  
**仕様:** `BEHAV-001`、`CTRL-LQR-001`、`CTRL-EST-001`。関連Issue #4。  
**範囲:** C11純粋ロジックとホストテストのみ。STM32書込み・CAN実通信・モータ駆動は実施していない。

## 仕様対照表

| 項目 | 現行コアの状態 | 判定 | 残作業 |
|---|---|---|---|
| MOTOR-150 明示的ARM | `arm_edge` がなければ有効化を要求しない | コアPASS | 実HIDは未接続 |
| MOTOR-151 姿勢範囲によるARM拒否禁止 | `arm_preconditions` が `posture_ok` を要求し、これだけがfalseでもARMを拒否 | **仕様不一致（実行再現済み）** | 安全仕様の変更承認後、範囲ゲート削除と回帰検証 |
| MOTOR-159 現在脚長で目標初期化 | コアに脚長入力・目標状態がない | **未実装** | 状態推定/脚長制御統合の際に実装。未確定原点は使用しない |
| MOTOR-153/195/196 DISARM・ゼロ電流 | `b_mode_step` はRS06無効化要求を出すが、M3508の500 Hzゼロ電流継続送信はコア出力/MCUへ未統合 | **部分実装** | CANアダプタ統合後に送信観測。ゼロ指令だけを安全停止とみなさない |
| MOTOR-154/155 FAULT解除後に別ARM | `clear_fault_edge && clear_authorized` でDISARMEDに戻り、同周期のARMは認めない | **コアPASS** | 実HIDのCLEAR_FAULTマッピングは未検証 |
| MOTOR-157 E-STOP独立性 | ソフトのESTOP入力ではFAULT・RS停止要求・電源OFF要求を出す | **部分実装** | 独立の物理遮断経路は基板で別途試験 |
| MOTOR-189〜194 送信監視 | モータCANベンチの模擬送信完了は試験済み。通常状態機械や実HALの送信監視は未統合 | **未統合** | Issue #5とF446のCAN HAL接続へ |
| MOTOR-197 四軸新規応答 | 4台それぞれの送信証跡とその後の新規応答がそろうまで駆動しない | **コアPASS** | 実モータのmode/fault解釈は未検証 |
| MOTOR-198 有効化失敗時の全軸停止・FAULT | 前提条件喪失では待機をキャンセルしてRS停止を要求するが、FAULTではなくDISARMEDのまま。応答が来ない場合に待機期限がない | **仕様不一致・未決定事項あり** | 失敗判定/タイムアウトとFAULT遷移の責務・閾値を決める承認が必要 |
| スケジューラ停止/時計逆行 | BALANCING中の経過6 ms以上と時計逆行でFAULT | **コアPASS** | 実500 Hz計測・スケジューラ統合は未検証 |

**注意:** `BMotorBench` は通常の `BModeMachine` とは独立した単軸ベンチ試運転用の状態機械であり、ベンチ側にある150 ms有効化タイムアウトを通常運転のタイムアウトとして採用したことにはならない。

## 実際の仕様差分再現

以下は**意図的に非ゼロで終了する診断プローブ**であり、通常のホストテストには追加しない。安全仕様の動作変更は行わず、現行の観測を示す。

```sh
mkdir -p build/host
gcc -std=c11 -Wall -Wextra -Werror -pedantic \
  -Ifirmware/f446/balancing_f446/App/Inc \
  firmware/f446/balancing_f446/App/Src/balancing_core.c tests/spec_gap_core_mode.c \
  -lm -o build/host/spec_gap_core_mode
build/host/spec_gap_core_mode
```

2026-11-09実測:

```text
MOTOR-151: SPEC MISMATCH: posture gate rejected otherwise ready ARM
MOTOR-198: DIAGNOSTIC only (timeout not specified): after 2s without replies, mode=0 pending=1 disable=0
```

MOTOR-151の判定は仕様との差分を検出し、プログラム終了コードは1。MOTOR-198は2秒時点の状態を観測しただけであり、**2秒という期限を要求する契約ではない**。無期限待機を防ぐ具体的な状態遷移の設計決定が必要。

## 追加した回帰テスト

`tests/test_core.c` に以下を追加し、コアの既存の安全側挙動が変わらないことを検証:

1. FAULT中にCLEAR_FAULTとARMが同時に入力されても再ARMしない。CLEAR許可、DISARMED、独立した新しいARMイベントの順。
2. RS06 Type3指令の未来時刻・同時刻・4軸未満の応答では駆動しない。
3. BALANCING中のスケジューラ異常は6 ms境界でFAULTとなる。DISARMED中の長い周期間隔は故障扱いしない。NULL入力は停止要求とFAULTになる。

実行コマンド:

```sh
python3 tools/run_host_tests.py
python3 tools/validate_specs.py
```

通常ビルドおよびUBSanはPASS。**これは既に実装されている正しい動作の回帰確認であり、上表の未実装/仕様不一致を解消したことは意味しない。**

## 変更承認が必要なもの

- `arm_preconditions` の姿勢範囲ゲート除去（MOTOR-151）。機械的限界の保護は制御中に維持し、ARM可否との関係を別途検証する。
- RS06有効化応答の不成立・故障報告・タイムアウトによるFAULT遷移（MOTOR-198）。上位アダプタとの責務分担、失敗時の全4軸停止要求、復帰動作を含めて設計する。

**既存の安全動作の意味が変わるため、これらの機能変更はこの監査PRには含めない。** 新規Issueは増やさず、同じIssue #4の継続項目として扱う。
