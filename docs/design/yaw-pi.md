# Yaw角速度PI（ホスト演算）

要求正本：[倒立・Yaw制御仕様](../specs/control/balance_vmc.md)（CTRL-LQR-001／MOTOR-201/202）。実装は `App/Inc/balancing/control/controller/balancing_yaw_pi.h` と `App/Src` の同名C11ソースです。

## 入力から車輪配分まで

操作層が加減速制限したYaw目標角速度と、BNO055から得た新鮮なYaw角速度の差 `e [rad/s]` を取ります。出力は `ΔT=Kp·e+I` [N m]、積分器は `I_next=clip(I+Ki·e·dt,±I_limit)` です。前後車輪のLQR共通トルクを先に確保した余力 `available_delta_nm` の範囲でのみ差動を出します。

積分候補がYaw余力の飽和をさらに悪化させる場合は前回積分を保持し、誤差反転などの積分解消方向は許可します。操縦モードがBALANCING以外なら直ちに出力と積分をゼロに戻します。ジャイロ鮮度喪失は上流の安全状態機械で判定し、本モジュールでは重複判定しません。未検証ゲイン・不正値は失敗として積分を破棄します。ARMセッション開始時は `b_yaw_pi_reset` を明示的に呼びます。

## 実機への結合条件

`available_delta_nm` は左右車輪トルク上限と倒立LQRの共通トルクから算出する**差動の残容量**です。最終的な左右配分は既存 `b_wheel_allocate` が行います。Yaw PIが車輪トルク上限を決定するわけではありません。

角速度目標0は方角保持を意味しません。CAN/ARM/車輪モータ指令はこのモジュール内で実行しません。未測定のジャイロ符号、実機ゲイン、最大電流、タイミングは設定しておらず、実機未検証では `verified=false` のままにします。

[ホスト検証と未完了事項](../verification/issue9-yaw-pi-host.md)
