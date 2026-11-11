# 固定脚長・Roll支持力（ホスト演算）

実装：`App/Inc/balancing/control/controller/balancing_leg_pd.h` と対応する `App/Src`。要求の正本は [制御仕様](../specs/control/balance_vmc.md)（CTRL-LQR-001）。

## 処理の境界

呼出し側が安全状態機械で左右RS06の鮮度を確認した後、DISARMED中の同一周期スナップショットから `b_leg_pd_capture` で基準を取得します。関数内では鮮度フラグを重複して確認しません。平均脚長 `L0=(LL+LR)/2` と初期左右差 `ΔL=(LL-LR)/2` を保存し、`Lref,left=L0+ΔL`、`Lref,right=L0-ΔL` として保持します。両脚の初期長を同じ値へ強制しないため、ARM直後の脚長PDの段差を抑えます。

`b_leg_pd_compute` は左右それぞれ `Fleg=Kp(Lref-L)-Kd·dL+Fgravity` を計算し、`Froll=-Kp_roll·roll-Kd_roll·roll_rate` を左へ加算、右から減算します。D項は目標の差分ではなく実測した伸縮速度のみを使います。

出力は**VMCへ渡す抽象支持力 [N]**です。RS06への電流指令、モータ上限、実CAN送信を行いません。ゲイン・重力補償とRoll符号は外部検証が必要で、`verified=false` では計算を拒否してゼロ出力にします。可動範囲や左右力の符号はCADと実機で未確定のため推測して固定していません。

## 検証と未完了

[Issue #9のホスト検証](../verification/issue9-leg-pd-host.md)で通常・UBSan、初期化、左右差動、異常値を検査します。Yaw PI、六節リンクに基づく実機VMC、ガスシリンダ補償、各RS06の最終独立クリップ、状態機械とのARM連結、実機でのゲイン調整は別作業です。
