# モータ制御レイヤーと単軸試運転

**更新日：** 2026-11-09  
**状態：** CANフレーム生成・受信解析・短時間試運転およびHALアダプタの**模擬ホスト試験は合格**。CubeIDEの非駆動C++骨組みはSTM32F446向けクロスビルドPASS（2026-11-09）。**HALモータCANアダプタのMCUビルド/実接続、CAN配線、書込み、実機回転は未実施。**

対象仕様：[RS06](../specs/protocols/rs06_can1.md)、[C620](../specs/protocols/c620_can2.md)、[F446](../specs/boards/f446_control.md)、[安全・操作](../specs/behavior/robot_operation.md)。

## ファイルと責務

| ファイル | 役割 |
|---|---|
| `firmware/core/include/balancing_motor_can.h` | CANフレーム／フィードバックの名前付き型、公開API |
| `firmware/core/src/balancing_motor_can.c` | RS06 Type 1/2/3/4、C620 0x200/0x201〜204の符号化・復号 |
| `firmware/core/include/balancing_motor_bench.h` | 短時間・単軸試運転用の最小状態管理API |
| `firmware/core/src/balancing_motor_bench.c` | 有効化応答確認、送信・受信鮮度、試運転時間制限、停止要求 |
| `firmware/f446/include/balancing_f446_motor_bench_port.h` | F446/HAL向けの接続API |
| `firmware/f446/src/balancing_f446_motor_bench_port.c` | bxCAN1/2送受信と送信完了メールボックスの連携 |
| `tests/test_motor_can.c` | RS06/C620既知フレーム、入力拒否、故障・タイムアウトのホスト試験 |

**正式な制御層との接続点：** 500 HzのLQR/VMCでモータ軸の物理トルク・車輪電流を求め、駆動許可・上限を確定させてから`b_rs06_torque_frame`／`b_c620_current_frame`へ渡す。安全状態機械自体は既存`BModeMachine`を使う。**試運転専用`BMotorBench`を通常バランシングの状態機械に組み込まない。**

## 仕様上の注意

- RS06：CAN1、拡張29bit、1 Mbit/s。Type 3は有効化、Type 4の全0データは通常停止。Type 1ではKp=Kd=0としてフィードフォワードトルクのみ使用する。
- RS06のType 1トルクはCAN IDのbit23〜8に16bitの線形換算で格納。メーカー資料の版による速度やKp上限の差異があるため、**実機のRS06ファームウェアの換算値を確認してから使用**する。現在の実装はType 2角度±4π、角速度±50 rad/s、トルク±36 N·mの資料に合わせたもの。上限36 N·mは安全な出力値という意味ではない。
- C620：CAN2、標準11bit、0x200で最大4台分。M3508のロータ角0〜8191、速度rpm。指令値±16384が±20 Aに対応する。ただし実機の許容電流上限とは別。
- RS06の対象ID・主機ID・C620の対象ID、モータFW版、電源条件、低トルク／低電流試験上限は**未確定**。起動時に推測で埋めない。
- 開発用`BMotorBench`は同時に複数モータへ非ゼロ指令を出さない。RS06はType3送信完了後の新規正常報告（mode=2、fault=0）を確認してからトルク指令を開始。C620は20 ms以内の正常形式フィードバック確認を要求する。
- 試運転コードには**RS06最大0.5 N·m、C620最大0.5 A、最大200 ms**の保守的なハード上限を設ける。これらは機構安全を保証する数値ではなく、実機で確認する上限はさらに低く設定できる。RS06フィードバック30 ms切れ、C620フィードバック20 ms切れ、RS06有効化待ち150 ms経過時に出力を止める。運転中の異常では再起動要求を自動で発行しない。
- 試運転の終了時はRS06停止Type4、C620の0 Aを送信する。ただしCANバスオフ・MCU停止・無通電では送達できない。**これだけでは安全停止ではない**。独立した物理非常停止を必須とする。

## HAL実装へ接続する条件

実際のSTM32CubeIDEプロジェクトは firmware/f446/balancing_f446/ にあり、IOC・main.c・スタートアップコードとHALを含む。2026-11-09にCubeIDE 1.19.0のヘッドレスF446クロスビルドで非駆動C++アプリ骨組みのELF生成はPASS。**ただし、モータCANアダプタとC11モータCAN層は現在のMCUビルド対象に含まれない。** CAN1/2の開始・受信フィルタ・RX/TX割り込み連携も未接続。モータを駆動できる完成ファームウェアではなく、実機へは書き込まない。

CubeMX等で、F446のCAN1とCAN2を1 Mbit/sに設定し、CAN1拡張ID（RS06）、CAN2標準ID（C620）のフィルタ、CAN Rx FIFO0、Tx mailbox complete IRQ、2 msの時刻基準、IWDG、物理非常停止入力を準備する。CAN2使用時にはCAN1側のクロックとフィルタバンク分割に注意。ピンは基板回路図と照合する。

### 主ループの接続例（概念・そのままでは書込み不可）

```c
/* CANフィルタ、クロック、HAL_CAN_Start、Rx/Tx割り込み設定は別に行う。 */
BF446MotorBenchPort motor_port;
BMotorBenchConfig cfg = {
    .rs06_motor_id = /* 実機で確認した値 */,
    .rs06_host_id = /* 実機で確認した値 */,
    .c620_motor_id = /* 実機で確認した値 */,
    .rs06_limit_nm = /* 実機の安全な微小トルク */,
    .c620_limit_a = /* 実機の安全な微小電流 */,
    .pulse_duration_ms = /* 例えば100 ms以下から開始 */
};
b_f446_motor_port_init(&motor_port, &hcan1, &hcan2, &cfg);

/* 500 Hzのメインループから毎回呼ぶ（時刻は単調増加us）。 */
b_f446_motor_port_tick_2ms(&motor_port, now_us);

/* CAN割り込みから、該当ポートへ中継する。 */
b_f446_motor_port_on_rx0(&motor_port, hcan, now_us);
b_f446_motor_port_on_tx_done(&motor_port, hcan, mailbox_index, now_us);

/* 必ずモータ固定・車輪浮上・非常停止の確認後、明示的操作で1回だけ呼ぶ。 */
b_f446_motor_port_pulse_rs06(&motor_port, test_torque_nm, now_us);
/* C620は別試験。 */
b_f446_motor_port_pulse_c620(&motor_port, test_current_a, now_us);

/* 中止操作／非常停止検出時。別途ハードウェアの電源遮断も必要。 */
b_f446_motor_port_cancel(&motor_port);
```

**注意：** 上記は呼び出し順を示す接続図であり、未定義の実測値をそのまま貼り付けてコンパイルできるコードではない。実際のCAN HAL callbackではmailbox0/1/2の正しい番号を渡し、**HAL送信受付時ではなく送信完了時**に`on_tx_done`を呼ぶ。時刻`now_us`はHAL_GetTick()のミリ秒値をそのまま代入しない。

### HAL callbackの接続例

```c
/* 500 Hz処理とCAN callbackが共通に参照するインスタンス。 */
extern BF446MotorBenchPort motor_port;
extern uint64_t monotonic_us(void); /* ボード固有の単調増加us時計 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    b_f446_motor_port_on_rx0(&motor_port, hcan, monotonic_us());
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    b_f446_motor_port_on_tx_done(&motor_port, hcan, 0u, monotonic_us());
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    b_f446_motor_port_on_tx_done(&motor_port, hcan, 1u, monotonic_us());
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    b_f446_motor_port_on_tx_done(&motor_port, hcan, 2u, monotonic_us());
}
```

実際のCubeMXプロジェクトに既存HAL callbackがある場合は、**二重定義せずその中から上記関数を呼ぶ**。CAN2のゼロ指令を送信し続けるにはCAN2のTX mailbox完了通知も必要。送信遅延が継続しTX割り込みが欠落すると出力の安全停止を保証できないため、Bus-off／IWDG／独立した物理非常停止を必ず結合する。

## 実機検証の順序

1. 基板のピン割当、CAN終端120 Ω、共有GND、RS06主機ID/全ID、C620 ID、モータFW版、24 V供給、非常停止の独立動作を確認する。
2. 車輪・脚が空転可能な治具で、**非ゼロ出力なし**のCAN受信を先に確認。RS06のType2応答とC620の0x201/0x202のID・温度・RPMを記録する。
3. RS06一台だけType3→新規Type2正常応答→短い低トルクType1→Type4停止。可動限界やガスシリンダの反力に注意。機構に取り付けた状態では出力方向が未検証のため、最初は極力単体で試す。
4. C620一台でID1/ID2の符号と極小電流の短時間パルスを確認。DISARMED相当では0 Aを500 Hzで継続する。
5. 各台の受信断・送信障害を試験し、停止指令が届く場合と届かない場合を区別して記録する。条件を満たしてから4台RS06・2台M3508の同期出力層へ進む。

## 検証コマンド

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake
python3 tools/validate_specs.py
```

今回の状態：**RS06/C620のホスト試験合格、CubeIDEの非駆動C++アプリ骨組みはF446向けクロスビルドPASS**。ただしモータCAN層のMCUビルド・実HAL接続、書込み、実機回転は未検証（Issue #3）。

**関連TODO：** [通信](../todo/protocols.md)、[F446](../todo/f446.md)、[検証](../todo/verification.md)。