# 単軸試運転：RX期限をCANポートへ集約

実施日：2026-11-11。対象：`refactor/motorbench-rx-timeout-at-port`。仕様：BOARD-F446-001、PROTO-RS06-001、PROTO-C620-001。

## 変更した境界

`balancing_f446_motor_bench_port.c` が対象CAN ID・形式の新規受信時刻を保持し、RS06 30 ms / C620 20 ms の受信期限を一度判定します。結果の `feedback_current` をベンチに渡します。値の出典は既存仕様で、時間は変更していません。

`balancing_motor_bench.c` はフィードバック期限の定数も時刻差分計算も持ちません。RS06の有効化指令送信完了→新規正常応答の順序は維持し、別用途の150 ms有効化待ちと最大200 msのパルス期限も維持します。

C620は要求時と出力ティック時の両方で**ポートの同じ期限判定関数**を使用します。RS06は4 ms送信ティックでポートから現在の通信状態を渡します。期限超過は従来どおり、C620を0 Aにし、RS06にType4停止を要求します。これはCAN到達・物理停止の証明ではありません。

## 実行結果

| コマンド | 結果 |
|---|---|
| `python3 tools/run_host_tests.py` | PASS、通常とUBSan両方 |
| `cmake -S . -B build/host-cmake`、`cmake --build build/host-cmake --parallel 4` | PASS |
| `cmake --build build/host-cmake --target test` | **10/10 PASS** |
| `python3 tools/regenerate_cubeide_f446.py` | **Debug / Release PASS** |
| `python3 tools/validate_specs.py` | PASS |

`tests/test_motor_hal_port.c` に、対象外CAN IDによる期限の誤延長禁止、受信なしの開始拒否、C620 20 ms境界と超過後ゼロ電流、RS06 30 ms期限超過でのType4停止要求を追加しました。ベンチ単体のホスト試験ではポートが返す通信状態を直接注入します。

## 未検証・次の整理範囲

実機CAN配線、IWDG、RS06実フィードバック、モータが停止できたか、実機での送信遅延・受信欠落は未検証です。

通常倒立制御は**このベンチとは別経路**です。現時点の`balancing_core.c`と`balancing_tx_scheduler.c`には、通常制御モック向けの重複したRS06/C620受信鮮度期限が残っています。これらは実機の入口がまだ未接続なので、実経路の監視がなくなる状態で一方を消さず、通常制御のCAN受信I/Fを統合する段階で一箇所へ集約します。送信完了期限は受信鮮度とは別に維持します。
