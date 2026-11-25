# CAN ID・データ内容・単位の総覧

**更新日：2026-11-25。** CAN1とCAN2は別バス。**数値未確定のセルを設計上の確定値と見なさないこと。** 詳細仕様および送受信時の判定は各リンク先が正本。

## CAN1：RobStride 06（29bit拡張、1 Mbit/s）

| 対象 | IDの形式 | 方向 | データ内容 | 単位 | 詳細 |
|---|---|---|---|---|---|
| RS06モータ1〜4（左A・左B・右A・右B） | Type 1、宛先ID=1〜4、bit23〜8はトルク | F446→モータ | CAN ID内トルク、DATA角度・角速度・Kp・Kd | N·m、rad、rad/s、ゲイン | [RobStride通信](specs/protocols/rs06_can1.md) |
| 同上 | Type 2、送信元モータIDはbit15〜8 | モータ→F446 | 角度・角速度・実トルク・温度、故障・モード | rad、rad/s、N·m、℃ | 同上 |
| 同上 | Type 3 | F446→モータ | 有効化 | 単位なし | 同上 |
| 同上 | Type 4 | F446→モータ | 停止（故障解除ビットに注意） | 単位なし | 同上 |
| 同上 | Type 24 | 双方向 | 100 Hz定期報告の切替・状態報告 | 単位なし／状態のSI量 | 同上 |

**注意：** RS06の各Typeは29bit拡張IDの上位5bitに格納する種別であって、`0x01`や`0x18`がCAN ID全体ではない。RS06の主機IDと量子化範囲の一部は実機照合待ち。

## CAN2：車輪・電源・上位通信（1 Mbit/s）

| CAN ID | 形式 | 方向 | 対象・内容 | 単位 | 状態 |
|---|---|---|---|---|---|
| **`0x200`** | 標準11bit、DLC8 | F446→C620 | 1〜4番の電流指令。左=DATA[0:1]、右=DATA[2:3]、残り0 | 約0.0012207 A/LSB | **メーカー資料で確認** |
| **`0x201`** | 標準11bit、DLC8 | 左C620→F446 | 角度[0:1]、rpm[2:3]、実電流[4:5]、温度[6] | カウント（rad換算）、rpm、電流生値（A換算検証待ち）、℃ | **メーカー資料で確認** |
| **`0x202`** | 標準11bit、DLC8 | 右C620→F446 | 上記と同じ | 同上 | **メーカー資料で確認** |
| `0x1FF` | 標準11bit、DLC8 | F446→C620 | 5〜8番用グループ | A相当 | **本機未使用** |

**C620は標準11bit、RS06は拡張29bit。** 物理量の単位が同じでも、フレーム配置、値域、機器内の減速比は異なる。

## CAN2：自作プロトコル v1のID構成（設計承認済み）

| bit | 名称 | 意味 |
|---|---|---|
| 28:26 | Priority level | 0非常停止、1停止/FAULT、2電源/操作、3診断/読出し、4–7予約 |
| 25:18 | Data type | 0x01共通、0x10電源、0x20操作、0x30診断、0x40読出し |
| 17:14 | Source ID | 1 F446、2 F303、3 Raspberry Pi |
| 13:10 | Destination ID | 上記機器番号、0xF全体宛て |
| 9:0 | Register ID | データ種別内のメッセージ番号 |

ID = (priority << 26) | (type << 18) | (src << 14) | (dst << 10) | reg。PC=Raspberry Pi。Classic CAN・1 Mbps・拡張29bit・DLC 0–8、Big endian、IEEE754 float32、SI単位、予約0。u8/u16/u32は符号なし整数、R=読取り/通知、W=指令、RQ=読み出し要求（RTRではない）。構造体の生コピー禁止。個別メッセージの送信元・宛先・DLCは許可リストで検証。送信元IDは認証ではない。

### C610/C620予約領域

標準CAN ID 0x1FF–0x208（全8軸の指令・報告）および0x700（一斉ID設定用とされるため予防予約）は自作通信で使用禁止。拡張IDの数値そのもの・上位11bit（ID >> 18）の双方がこの範囲に入る場合も禁止。メーカー既存モータ通信経路は変更しない。C610/C620混在時はモータ番号を機種横断で一意にする。0x700の全FWでの挙動は未確認であり実機送信しない。

### 結果コード

| 値 | 名称 | 概要 |
|---|---|---|
| 0 | ACCEPTED | 要求受理（実行完了ではない） |
| 1 | ALREADY_IN_STATE | 既に要求状態 |
| 2 | BUSY | 処理中 |
| 3 | SAFETY_REJECTED | 安全条件不成立 |
| 4 | INVALID_REQUEST | 不正な要求 |
| 5 | UNAUTHORIZED | 許可外送信元 |

数値コードはv1草案の候補を採用する設計値。要求番号は応答対応用。宛先毎1件を基本とし自動再送・セッション発行を行わない。応答が得られなければ結果不明として状態を確認し、新規明示操作にする。ARM内部のRS06セッションは別であり維持する。

### 個別メッセージ一覧

- [電源・非常停止（共通、0x01/0x10）](specs/protocols/power_can2.md)
- [操作・診断・設定読出し（0x20/0x30/0x40）](specs/protocols/remote_can2.md)

設計参考：[学ロボ2025 data_protocol.md](https://github.com/NITTC-GakuRobo/Gakurobo2025_protocol/blob/23801878d40837cd203e6dd362931320f34781e8/data_protocol.md)（Data type別の表記のみ、ワイヤ互換性なし）。承認履歴：[Issue #27](https://github.com/kou-pen/balancing/issues/27#issuecomment-7157121128)、[0x700追加予約](https://github.com/kou-pen/balancing/issues/27#issuecomment-7157184975)。

## 仕様確定の優先順

1. 使用する機器の正規説明書・ファームウェア版
2. 配置された実機のCANログとバイトデコード
3. 現在の基板・既存実装のCANフレーム定義
4. 本仕様書の暫定値（不一致があれば修正）

F303/Pi通信は承認された新規v1の設計値。既存FWとの互換性・実CAN挙動は未検証。

## 参照資料

- [DJI C620説明書](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf)
- [RobStride RS06説明書（2025-11版）](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS06/RS06User%20Manual251112.pdf)
