# 実機非依存の制御コア（現行C++17）

現在、自作Appに旧C11の制御関数は残っていません。HAL非依存の制御数学と状態判断は以下のクラスに集約されています。

| ファイル | 入力 → 出力 |
|---|---|
| control/filter/ImuAttitudeSource.hpp/.cpp | 加速度・ジャイロ → 姿勢 |
| control/estimator/LegKinematics.hpp/.cpp | モータ角 → 閉リンク幾何・ヤコビアン、仮想力 → 関節Nm |
| control/estimator/WheelOdometry.hpp/.cpp | 車輪累積角度・RPM → 位置・速度 |
| control/estimator/StateEstimator.hpp/.cpp | IMUと6軸 → ControlSnapshot |
| control/controller/BalanceControl.hpp/.cpp | 制御状態・目標 → LQR、脚長/Roll PD、Yaw PI、VMCと機構Nm |
| control/controller/MotorCommand.hpp/.cpp | 機構Nm → RS06 Nm・C620 Aと軸別制限 |
| app/RobotSequence.hpp/.cpp | 状態・保護条件・操作 → 状態と駆動要求 |
| app/ControlCycle.hpp/.cpp | 上記を一周期に結合（CAN/HAL送信なし） |

LQRの6状態は仮想脚角・角速度、車体位置・速度、Pitch角・角速度です。機構側の力・トルクとモータ側の電流を区別します。脚長基準はARM時に捕捉し、Yawは倒立優先の余力で配分します。通常の数学飽和だけをFAULTにしません。

式と詳細は [倒立・VMC仕様](specs/control/balance_vmc.md)、[状態推定仕様](specs/control/state_estimation.md)、[脚運動学](design/leg-kinematics.md)、[脚長・Roll](design/leg-pd-roll.md)、[Yaw](design/yaw-pi.md)、[姿勢](design/attitude-filter.md)。コードの所在は [フォルダ別案内](design/public-api.md)。

実機のCAD・原点・極性・ゲイン・変換・上限は未確定です。人工モデルの計算結果はSIL用であり、実機非ゼロ出力には接続されていません。

旧C11実装の説明・過去の差分は [整理前の全文（Git履歴）](https://github.com/kou-pen/balancing/blob/cb3abeda89acdbcb08fd9890735f2b8c95ed9d83/docs/software-core.md) と [C++統合検証](verification/f446-all-cpp.md) に残しています。
