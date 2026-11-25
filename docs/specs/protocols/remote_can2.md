# CAN2：Raspberry Piからの操作・診断通信仕様

**仕様ID:** PROTO-REMOTE-001  
**更新日:** 2026-11-25  
**状態:** 新規v1の設計承認済み。コーデック・実機接続・周期候補の検証は未実施。

PC＝Raspberry Pi（単一ノード）。共通ID構成とデータ表記は[CAN ID総覧](../../can-id-table.md)、非常停止は[電源通信](power_can2.md)に一元化する。学ロボ2025は資料の分類・表記の参考であり、ワイヤ形式の互換性を要求しない。

## Register IDの詳細

### 操作（Data type = 0x20）

| Register ID | name | Priority | CAN ID（拡張） | 型 / DLC | r/w | 方向 | 概要 |
|---|---|---:|---|---|:---:|---|---|
| 0x001 | MOTION_COMMAND | 2 | 0x0880C401 | float32×2 / 8 | W | Pi→F446 | 前後速度[m/s]、Yaw角速度[rad/s] |
| 0x002 | ARM | 2 | 0x0880C402 | uint16 / 2 | W | Pi→F446 | 要求番号、明示ARM |
| 0x003 | DISARM | 1 | 0x0480C403 | uint16 / 2 | W | Pi→F446 | 要求番号、停止要求 |
| 0x004 | CLEAR_FAULT | 2 | 0x0880C404 | uint16 / 2 | W | Pi→F446 | 要求番号、原因解消後の明示解除 |
| 0x005 | SELECT_SOURCE | 2 | 0x0880C405 | uint16,uint8 / 3 | W | Pi→F446 | 要求番号、操作元（0 USB / 1 CAN） |
| 0x006 | REMOTE_RESULT | 2 | 0x08804C06 | 下記 / 8 | R | F446→Pi | 操作結果 |

MOTION_COMMANDのDATA[0:3]は前後速度、DATA[4:7]はYaw角速度。REMOTE_RESULTはbyte0–1要求番号、2–3要求Register ID、4結果、5ロボット状態（0 DISARMED / 1 ARMING / 2 BALANCING / 3 FAULT）、6–7予約0。結果は[共通定義](../../can-id-table.md)。応答は操作受理の結果であり駆動成立を保証しない。

標準操作元はUSB_DUALSENSE。双方からの明示切替はDISARMED中のみ。旧入力を破棄し、新規中立確認後に操作を受理。自動切替なし。ARM・運動指令は選択元のみ、DISARM・非常停止は選択元によらない。CLEAR_FAULTは原因解消時の明示操作。ARMの安全条件は[動作仕様](../behavior/robot_operation.md)のまま。
運動指令20 ms、受信期限100 msは試験候補。別ハートビート・指令連番なし。正しい新規受信のローカル時刻で鮮度判定し、受信途絶で既存のゼロへの減速・復旧中立確認。生成時刻・重複・操作セッション所属をワイヤ上で識別できないため、古い指令のキュー蓄積は禁止。

### 診断・モニタ（Data type = 0x30）

| Register ID | name | Priority | CAN ID（拡張） | 型 / DLC | r/w | 方向 | 周期 | 内容（DATA順） |
|---|---|---:|---|---|:---:|---|---|---|
| 0x001 | ROBOT_STATUS | 3 | 0x0CC04C01 | u8,u8,u16,u16,u16 / 8 | R | F446→Pi | 100 ms | 状態、操作元、状態フラグ、現在故障、観測有効性 |
| 0x002 | ATTITUDE_ANGLE | 3 | 0x0CC04C02 | float32×2 / 8 | R | F446→Pi | 100/20 ms | Pitch・Roll角[rad] |
| 0x003 | ATTITUDE_RATE | 3 | 0x0CC04C03 | float32×2 / 8 | R | F446→Pi | 100/20 ms | Pitch・Roll角速度[rad/s] |
| 0x004 | YAW_RATE | 3 | 0x0CC04C04 | float32 / 4 | R | F446→Pi | 100/20 ms | Yaw角速度[rad/s] |
| 0x005 | CHASSIS_STATE | 3 | 0x0CC04C05 | float32×2 / 8 | R | F446→Pi | 100/20 ms | 前後位置[m]、前後速度[m/s] |
| 0x006 | WHEEL_STATE | 3 | 0x0CC04C06 | float32×2 / 8 | R | F446→Pi | 100/20 ms | 左・右車輪角速度[rad/s] |
| 0x007 | LEG_LEFT_STATE | 3 | 0x0CC04C07 | float32×2 / 8 | R | F446→Pi | 100/20 ms | 左脚長[m]・脚角度[rad] |
| 0x008 | LEG_RIGHT_STATE | 3 | 0x0CC04C08 | float32×2 / 8 | R | F446→Pi | 100/20 ms | 右脚長[m]・脚角度[rad] |
| 0x009 | CONTROL_TARGET | 3 | 0x0CC04C09 | float32×2 / 8 | R | F446→Pi | 100/20 ms | 前後速度目標[m/s]・Yaw角速度目標[rad/s] |
| 0x00A | CONTROL_TIMING | 3 | 0x0CC04C0A | uint32×2 / 8 | R | F446→Pi | 100 ms | 実測周期[us]、周期超過累計（取得可否確認） |
| 0x020 | FAULT_1 | 1 | 0x04C04C20 | u16,u32,u16 / 8 | R | F446→Pi | イベント | イベント番号、イベント時刻[ms]、主原因 |
| 0x021 | FAULT_2 | 1 | 0x04C04C21 | u16,u16,u16,u16 / 8 | R | F446→Pi | イベント | イベント番号、発生時異常、追加異常、予約 |
| 0x022 | FAULT_3 | 1 | 0x04C04C22 | u16,u32,u16 / 8 | R | F446→Pi | イベント | イベント番号、元FAULT発生時刻[ms]、予約 |
| 0x080 | STREAM_SUBSCRIBE | 3 | 0x0CC0C480 | uint16×4 / 8 | W | Pi→F446 | イベント | 対象Register ID、周期[ms]、購読期限[ms]、要求番号 |
| 0x081 | STREAM_UNSUBSCRIBE | 3 | 0x0CC0C481 | uint16×2 / 4 | W | Pi→F446 | イベント | 対象Register ID、要求番号 |
| 0x082 | STREAM_RESULT | 3 | 0x0CC04C82 | u16,u16,u8,3B / 8 | R | F446→Pi | 応答 | 要求番号、要求Register ID、結果、予約3byte |

u8/u16/u32は符号なし整数。各フィールドはDATA[0]から隙間なしに順次配置。周期値は候補であり実機計測により確定する。ROBOT_STATUSの状態フラグ・有効性ビットは未決定。無効観測は通常物理量の送信を抑止し、Piは受信期限切れを表示する。通常テレメトリの時刻はPi側の受信時刻であり同時観測の保証ではない。
基本配信100 ms＋選択された詳細配信20 ms。購読期限1000 msは候補。購読更新が途絶すれば通常へ戻す。停止・モータ通信を優先し、周期データは最新値に置換。軸別詳細・状態遷移の追加IDは未確定。

FAULT_1～3は同じ内部イベント番号の下位16bitで関連づける。32bit msはMCU側時刻の下位32bit。元FAULTの時刻と追加異常のイベント時刻は区別する。PCの再構成期限・番号周回・再起動時の取り扱いは未確定。各故障ビットは既存FaultFlagsに対応：
| bit | 意味 |
|---:|---|
| 0 | EmergencyStop |
| 1 | BusOff |
| 2 | MotorFault |
| 3 | ExternalFault |
| 4 | EnableFailed |
| 5 | MechanismUnsafe |
| 6 | InterlockLost |
| 7 | ClockRegression |
| 8 | ControlTiming |
| 9 | ArmTimeout |
| 10–15 | 予約 |

主原因は単一ビット。3パートの送信完了はPC受信や物理停止の証明ではない。

### 設定読み出し（Data type = 0x40）

| Register ID | name | Priority | CAN ID（拡張） | 型 / DLC | r/w | 方向 | 概要（DATA順） |
|---|---|---:|---|---|:---:|---|---|
| 0x001 | PARAM_READ | 3 | 0x0D00C401 | uint16×2 / 4 | RQ | Pi→F446 | パラメータ番号、要求番号 |
| 0x002 | PARAM_VALUE | 3 | 0x0D004C02 | u16,u16,4B / 8 | R | F446→Pi | パラメータ番号、要求番号、値 |
| 0x003 | PARAM_ERROR | 3 | 0x0D004C03 | u16,u16,u8,u8 / 6 | R | F446→Pi | パラメータ番号、要求番号、結果、予約 |

番号別の型・単位・値の有効性は未確定。PCからの設定書き込みは行わない。変更はソース・起動設定のレビュー、再ビルド、書き込みによる。

作業番号：T-PROTO-06、T-PROTO-09、T-PROTO-11。
