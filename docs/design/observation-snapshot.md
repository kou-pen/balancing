# 観測スナップショット（現行C++17）

500 Hz制御とセンサ受信頻度は別です。同じデータを読み返しても測定時刻は更新しません。

| 入力 | 保持する場所 | 周期で読む場所 |
|---|---|---|
| RS06の角度・速度・状態 | Rs06Motor × 4 | MotorService.feedback |
| C620の角度・RPM・連続性 | C620Motor × 2 | MotorService.feedback → WheelOdometry |
| BNO055 | Bno055、ImuAttitudeSource | imuAttitude(nowUs) |
| F303電源 | PowerBoardService | snapshot(nowUs) |
| 操作入力 | DualSenseService、RemoteControlService | Applicationの操作イベントと目標 |

CANはIRQで時刻付きイベントを記録し、`f446_can1_runtime.cpp` のCanRuntime.serviceが両バスを回収します。回収終了時の共通時刻でMotorObservationSnapshotを生成します。ApplicationはこのコピーとIMU・電源・操作を使ってControlTickInputを作ります。

C620の連続性が失われた場合、古い角度を積算し続けません。車輪原点の再設定は明示操作であり、ARM/DISARMのたびに自動実行しません。受信した時刻・回収番号・制御開始番号を別々に扱います。

`resetApplicationWheelReference()` は起動時の明示許可、DISARMED、周期外、未消費操作・FAULTなし、左右校正・受信設定の成立を要求します。両CANの未配送イベント、FIFO0、異常、受付済みTXが残れば、内容を捨てず操作を拒否します。両輪と推定器の原点世代を照合して同期更新し、以前の観測・推定を無効化します。次の左右の新規受信を待ちます。採取境界はローカル時刻であり機器側の送信時刻・物理同時性は保証しません。通常ビルドの既定設定は無効、操作ボタンやCANコマンドは未接続です。

| 判定対象 | 所有者 |
|---|---|
| IMU 50ms・RS06 30ms・C620 20ms・電源 500msの鮮度 | 各機器サービス（期限ちょうどは無効） |
| 6軸と姿勢を統合できるか | StateEstimator |
| 運転状態と保護条件 | RobotSequence |
| 制御開始・完了の照合 | CycleHealthMonitor |

旧C観測ストアは廃止済み。 [データ所有権](data-model.md) ／ [周期図](../diagrams.md) ／ [C++統合の記録](../verification/f446-legacy-observation-retirement.md)。旧設計の詳細は [整理前の全文（Git履歴）](https://github.com/kou-pen/balancing/blob/cb3abeda89acdbcb08fd9890735f2b8c95ed9d83/docs/design/observation-snapshot.md)。
