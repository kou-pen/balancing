# 仕様ID登録簿

このファイルは索引であり、仕様本文を重複記載しない。IDはファイル名を変更しても固定し、他の機能に再利用しない。

| 仕様ID | 正本 | 関連する設計決定 |
|---|---|---|
| BOARD-F446-001 | [F446メイン基板](specs/boards/f446_control.md) | 500 Hz、CAN、IWDG、ARMの保護 |
| BOARD-F303-001 | [F303電源基板](specs/boards/f303_power.md) | 12S→24 V、非常停止、電源監視 |
| PROTO-RS06-001 | [RS06 CAN1](specs/protocols/rs06_can1.md) | MOTOR-103／106／108／109／111 |
| PROTO-C620-001 | [C620 CAN2](specs/protocols/c620_can2.md) | MOTOR-121／142／143／145 |
| PROTO-PWR-001 | [F303電源CAN](specs/protocols/power_can2.md) | MOTOR-089／092 |
| PROTO-REMOTE-001 | [Pi遠隔CAN](specs/protocols/remote_can2.md) | CAN_REMOTE |
| PROTO-PS5-001 | [DualSense USB](specs/protocols/ps5_usb.md) | USB_DUALSENSE |
| PROTO-IMU-001 | [BNO055 I2C](specs/protocols/bno055_i2c.md) | MOTOR-138／139／140 |
| CTRL-EST-001 | [状態推定・6節リンク](specs/control/state_estimation.md) | MOTOR-125／137／138 |
| CTRL-LQR-001 | [LQR／VMCとPID](specs/control/balance_vmc.md) | MOTOR-097／118／121／123／142 |
| BEHAV-001 | [ロボットの動作](specs/behavior/robot_operation.md) | MOTOR-146〜159／183〜196／197〜198 |
| BEHAV-JUMP-001 | [ジャンプ制御（後続）](specs/behavior/jump.md) | MOTOR-168〜187 |

**関連する決定：** CTRL-LQR-001にはMOTOR-160〜167／199〜224、CTRL-EST-001にはMOTOR-200／207〜211を反映。

**通信の早見表：** [CAN ID・データ内容・単位の総覧](can-id-table.md)。

**構成・起動・状態の図：** [制御システム図](diagrams.md)。

## 仕様とコードの対応

公開APIの宣言には`@spec{CTRL-LQR-001}`などを記述し、設計上の根拠と対応づける。仕様IDは原則として**文書単位**に付与し、細かな補助IDを無闇に増やさない。

## 検証状態の扱い

「設計で採択済み」と「実装済み」と「実機検証済み」は区別する。未検証の数値、CANフレームのバイト形式や換算係数を確定扱いしない。

## 保留の決定事項

- MOTOR-095：再起動後のFAULT状態復元。
- MOTOR-132：起き上がり・立ち上がりシーケンス（脚長目標の初期値はMOTOR-159で現在値に確定）。