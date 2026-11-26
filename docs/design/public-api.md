# F446 公開API設計（C++17）

2026-11-17の設計決定。対象は `balancing_f446/App` の公開APIとその呼び出し順です。[旧C11制御コア](../software-core.md) の挙動と試験を移行時の比較基準にします。公開型・関数はC++17向けに設計します。既存の状態機械との差は [仕様差分監査](../verification/core-mode-contract-audit.md) を参照してください。

- 要求仕様：[基板（BOARD-F446-001）](../specs/boards/f446_control.md)／[動作（BEHAV-001）](../specs/behavior/robot_operation.md)／[状態推定（CTRL-EST-001）](../specs/control/state_estimation.md)／[制御（CTRL-LQR-001）](../specs/control/balance_vmc.md)
- 関連作業：[公開API・移行 #27](https://github.com/kou-pen/balancing/issues/27)／[MOTOR-151・198 #4](https://github.com/kou-pen/balancing/issues/4)／[CAN送信調停 #5](https://github.com/kou-pen/balancing/issues/5)
- [データフロー、ARM、周期処理、停止とPC診断の図](../diagrams.md)

## 所有者と依存関係

| 公開境界 | 入力・出力 | 責務 |
|---|---|---|
| `RobotSystem` | HAL初期化後の `init()`、各コンポーネントへの参照 | 長寿命オブジェクトを静的に組み立てる。必須依存は構築時に参照で注入する |
| `Application` | タイマー通知、操作・センサー・CANイベント | メインループで500 Hz周期を実行し、今周期の入力と実測周期間隔を固定する。最後に6軸指令を一括確定する |
| `CanPort`、`CanTxScheduler` | CANフレーム、送信受付結果、Tx完了、バス異常 | CAN1・CAN2に各1組。ポートがHALとイベントを扱い、スケジューラが送信機会と優先順位を管理する |
| Codec、`CanRouter` | バス・ID・フレーム、受信時刻 | Routerが配送先を決め、Codecが形式検証と符号化・復号を行う。電源基板・PC操作の形式は各通信仕様を参照 |
| `MotorService` | 毎周期の `DesiredMotorState` とARMセッション番号、6軸の `MotorTargets` | 4台のRS06と2台のC620を所有し、フィードバックと安全判断を集約する。非ゼロ送信を防護し、各CANの送信調停につなぐ |
| `RobotSequence` | 操作イベント、必須条件、現在の機器・推定状態 | ロボット全体の状態、ARM期限、セッション番号、FAULT記録を持ち、毎周期 `DesiredMotorState` を返す |
| `StateEstimator`、`LegKinematics` | 取得済みIMU・6軸フィードバック、機構入力軸、周期間隔 | 片脚の運動学と機体状態の推定を行い、今周期の `ControlSnapshot` を返す |
| `BalanceControl`、`MotorCommand` | `ControlSnapshot` と `BalanceReference`、機構トルクと変換設定 | 制御器が機構トルクを計算し、変換器が検証済み設定で各モータの指令に換算する |
| `DiagnosticsService` | FAULT等のイベント、最新のステータス | CAN経由でPCへ配信する。停止要求と通常駆動の制御経路から独立して診断を管理する |

C++オブジェクトの構築後、CubeMX／HALの周辺初期化が完了してからデバイスを起動します。必須I/Oの初期化に失敗した場合は駆動を禁止します。実機I/Oを差し替える境界では参照による依存注入を使います。CAN受信やタイマーの割り込みはイベント・実行要求の記録までとし、周期内の推定・制御・診断配信はメインループで進めます。

## `RobotSequence` とFAULT記録

`RobotSequence` は `DISARMED`、`ARMING`、`BALANCING`、`FAULT` を公開します。状態分岐は `switch` に明示し、基底の `StateMachine<State>` は状態保持と遷移の記録を担います。サービスは各機器の判定結果を報告し、ロボット全体のFAULT遷移は `RobotSequence` が決めます。

- ARM要求を受理したときにセッション番号を発行し、各周期で `MotorService` に渡す。過去のセッションの送信完了や応答を、現在の有効化判定に使用しない。
- `DesiredMotorState` は毎周期の要求。ARMING中の有効化、通常駆動、停止の要求を扱い、非ゼロ出力には機構安全条件の検証・成立が別途必要。
- 状態遷移、全体のARM絶対期限、同時異常の優先順位は [動作仕様](../specs/behavior/robot_operation.md) を参照。

`FaultRecord` は現在ラッチ中の記録と、解除済みの直近1件の診断履歴を分けます。発生時に優先順位で選んだ主原因と同時検出した `triggerFlags` を保存します。ラッチ中に後から検出した異常は `subsequentFlags` に記録します。両フラグは固定幅のビットフラグで、後続異常が主原因や `triggerFlags` を上書きすることはありません。明示的な解除時に現在の記録を直近の診断履歴へ移します。

## `MotorService`、モータ個体、指令変換

`MotorService` は6軸分の `MotorTargets` を一括設定し、6軸分の `MotorFeedbackSnapshot` を一括取得します。周期内の制御と安全判定は取得した値を共有します。受信鮮度は該当する機器の境界で評価し、上位はその判定を使います。

`Rs06Motor` は各軸の有効化・停止、フィードバックの状態と鮮度を持ちます。詳細な `Rs06Phase` は内部に置き、外部には集約に必要な安全判断の結果を公開します。フレーム形式の検証と復号は `Rs06Codec` が行います。`C620Motor` は独立した具象クラスで、2軸の共通電流フレームは `MotorService` の責務です。単軸試運転と通常制御は同じ下層を使う別の駆動経路です。

| 型・処理 | 契約 |
|---|---|
| `MechanismTorques` | 関節4軸と車輪2軸の機構側の要求トルク [N·m]。制御計算の出力であり、モータ軸への換算前の値 |
| `MotorCommand` | 機構側トルクをRS06のモータ軸トルク、C620の電流指令へ変換する。単位・有限値・検証済み設定を確認し、軸別の限界を適用する |
| `MotorCommandConfig` | RS06用とC620用の専用設定型を使い、6軸それぞれの変換係数・符号・上限を保持。起動時に読み込んで固定し、PCからは参照のみ |
| `MotorCommandResult` | `CommandStatus` と、6軸の要求値・制限適用後の値・飽和情報を返す。妥当な飽和は制限後の値を使用可能。無効値や未検証設定による失敗は非ゼロ指令を生成できない |
| `MotorTargets` | `MotorService` に一括で渡す変換済みの6軸分。CANの符号化・送信はサービス以下が担当する |

公開APIの角度・角速度・長さ・速度・トルク等は単位付きの軽量な専用型を使います。行列演算などでは内部で `float` を使用できます。物理上限とCANフレームで表現可能な範囲は、それぞれ `MotorCommand` とCodecが検証します。

送信調停はCANごとに行います。高優先の停止要求を通常イベントから独立して保持し、周期指令は対象ごとに最新値を使います。RS06のType 4停止は軸別に要求とTx完了を管理します。詳細は [RS06通信仕様](../specs/protocols/rs06_can1.md) を参照してください。

## `StateEstimator` と `ControlSnapshot`

| API | 入力・戻り値 | 世代・失敗時の扱い |
|---|---|---|
| `LegKinematics::solve()` | 片脚の機構入力軸の角度 [rad]・角速度 [rad/s] と幾何設定。脚長 [m]、機体基準の脚角 [rad] と各変化率、解の成立を返す | 左右を独立に計算。VMC統合時には、同じ周期の脚の解にヤコビアンを添える |
| `StateEstimator::update(input, elapsed)` | `StateEstimatorInput` に `ImuSnapshot` と `MotorFeedbackSnapshot` を入れる。前回からの実測周期間隔は `std::chrono::microseconds` で渡す | 戻り値は `std::variant<ControlSnapshot, StateEstimateError>`。失敗時は該当箇所と理由を返し、今周期の制御入力は成立しない |
| `StateEstimator::reset()` | `Application` がIMUの再初期化等で推定基準が変わったときに呼ぶ | 次の有効な入力から推定を初期化。通常のARM／DISARMをまたいで推定を継続する |

`ControlSnapshot` は、変換済みの物理状態を `Body`（機体姿勢・角速度）、`Chassis`（車体の前後位置・速度）、`Leg`（左右の脚長・脚角と変化率）、`Wheel`（左右車輪の状態）で構成します。機体に対する脚角は片脚の運動学で求め、鉛直に対する脚角が必要なときは推定側でIMU姿勢を組み合わせます。ヤコビアンはVMC用の脚の計算結果に保持します。

## `BalanceControl`

- `initialize()` で制御器の内部状態と目標の基準を設定する。ARM時の脚長基準は現在の左右脚長（MOTOR-159）。`reset()` で制御器の内部状態を初期化し直せる状態に戻す。両方の呼び出し時点は `Application` が管理する。
- `compute()` に今周期の `ControlSnapshot` と `BalanceReference` を渡す。操作側から渡す目標は `forwardVelocity`、`yawRate`、共通の `legLength`。位置や脚角などLQR用の目標は制御器内で組み立てる。
- `compute()` が返す指令は `MechanismTorques`。車輪は倒立を優先し、余力でYawを配分する。脚側はVMC導入時に、仮想脚の軸力・モーメントを同じ周期のヤコビアンで関節トルクへ写像する。
- `PID` 基底の役割はゲイン、必要な積分状態とリセットの再利用。Leg PD、Roll PD、Yaw PIはそれぞれ固有の演算規則を持つ。

## 500 Hzの指令確定

1. `Application` が入力を一度固定し、`StateEstimator::update()` を呼ぶ。
2. 推定結果または理由と、操作・機器の安全状態を `RobotSequence` に渡し、駆動可否を決める。
3. 駆動が許可され制御入力が成立した周期は、必要な初期化を行い、`BalanceControl` でトルクを計算して `MotorCommand` で変換する。駆動を止めるべき計算・変換異常は、送信前に `RobotSequence` に反映する。
4. 最終の `DesiredMotorState`、ARMセッション番号、6軸の `MotorTargets` を `MotorService` へ渡す。駆動許可が落ちた周期は、保持中およびソフトウェア上で送信待ちの非ゼロ指令を無効化し、ゼロ電流や停止要求をその周期に反映する。

制御周期の目標は2 ms。RS06通常指令は4 msごと、C620共通指令は2 msごとです。停止要求の送信には高い優先順位を与え、各バスの受付・Tx完了は非同期で追跡します。

## PC向けCAN診断

| 契約 | 挙動 |
|---|---|
| 配信 | 異常イベントは発生時に送信候補へ追加し、ステータスは定期配信する |
| CAN表現 | メッセージ種類ごとに固定CAN IDを割り当てる。`FaultEvent` は3フレーム、時刻は32 bitのミリ秒値 |
| 保持 | 重要イベントと通常イベントは別々の固定長FIFO。周期ステータスは最新値を保持 |
| 送信進捗 | イベントは該当フレームのCAN Tx完了まで保持する。`FaultEvent` の3フレームはそれぞれの進捗を追跡する |

固定IDとフレーム配置は[上位通信](../specs/protocols/remote_can2.md)に記録。配信周期候補・FIFO容量は検証により確定します。診断イベントの送信完了はローカルのCAN送信の証跡です。PCでの受信確認や物理停止の判定には使いません。

## 実装時の確認項目

以下は受入試験の観測点です。既存C11との比較では [ホスト試験](../software-core.md) と [仕様差分監査](../verification/core-mode-contract-audit.md) を使います。

| 対象 | 確認する結果 |
|---|---|
| ARM | 初期姿勢範囲と機構安全条件の分離。旧セッション・HAL受付のみ・4軸中3軸の状態で非ゼロにならない。期限の直前・境界、混雑再試行、確定故障、必須条件消失、手動DISARMを区別する |
| FAULTと停止 | 同周期に成立した異常とラッチ後の追加異常を識別。全4軸を独立して停止し、別世代のTx完了で停止要求が消えない。CLEAR_FAULT後は別のARMを必要とする |
| 推定と指令 | 同じ周期の入力と実測の周期間隔を使用する。右脚の推定失敗などをその周期で反映し、旧6軸指令を再送しない。検証済み上限での通常飽和と変換失敗を区別する |
| PC診断 | 重要・通常イベントのFIFO、最新ステータス、3フレームの部分送信・Tx完了をホストで確認する |
| 実機 | ARMの通信遅延、期限設定、可動域・符号・トルク／電流上限、制御周期、物理非常停止を実測・照合する |

## C++17 BalanceControlと500 HzのSIL経路（実機未接続）

`BalanceControl` は旧C11の純粋な数学演算 `b_lqr_compute` → `b_leg_pd_compute` → `b_leg_virtual_to_joint_torque`、および `b_yaw_pi_step` と `b_wheel_allocate` を順序付きで呼ぶ。6状態は `ControlSnapshot` から作り、左右脚の仮想トルクはLQRの両脚合計を左右へ等分し、脚別PD支持力とヤコビアンで4関節軸トルクへ変換する。車輪倒立トルクを優先して残った容量だけをYaw PIへ渡す。**飽和診断にはYaw PIによる事前制限と最後の車輪配分の両方を含める。**

公開 `BalanceReference` は位置基準 [m]、速度基準 [m/s]、Yaw角速度基準 [rad/s] を**上流が明示**する。位置目標の自動生成規則、操作者の変化率制限を先回りして実装しない。固定脚長目標は `initialize()` 時点の現在左右脚長から取得し、`reset()` でYaw積分と初期脚長目標を破棄する。未検証ゲイン・数値異常があれば `std::nullopt` を返し、古い指令を流用しない。

`BalanceControlOutput` はモータ側指令ではなく**6軸の機構側の数学トルク**である。現在の `fixtureOnly` は常にtrueで、本番の `MotorCommand`、`MotorService`、`CanPort`、`Application::controlTick()` へは接続していない。

`ControlCycle` はApp本体内のHAL非依存な周期実行器で、次の順番を試す。

1. 入力を1周期で固定し、`StateEstimator::update()` を呼ぶ。
2. ARM直前の脚長参照と今周期の観測・周期異常を準備する。
3. 既にBALANCING中なら制御の数学候補を計算し、数値演算失敗を状態機械へ同周期に報告する。
4. `RobotSequence::update()` は**一周期に一回**。FAULT/DISARMなら数値候補を破棄し、非BALANCING時のYaw履歴をクリアする。
5. BALANCINGを維持したときだけ `fixtureOnly` の計算候補をホスト試験へ返す。ハードウェアへの送信APIは存在しない。

SIL内で4軸のARM応答・検証済みの機構/ゲインを模擬するためのtrueは、**機体や実CANが検証されたという意味ではない**。そのような値を実機のARM許可へ使わないこと。詳細は [SIL制御周期検証](../verification/f446-sil-control-cycle-host.md) に記録する。

## MotorCommandのC++17換算とSILだけの最終制限

`MotorCommand::evaluate(BalanceControlOutput)` はRS06の機構軸トルク4軸[N·m]からモータ軸トルク[N·m]、C620の車輪トルク2軸[N·m]から電流[A]へ換算する。変換と最終制限は、このクラスが一度だけ担当する。起動時固定の `MotorCommandConfig` には、4軸それぞれの `Rs06TorqueConversion`（モータ/機構軸の回転比、符号、モータ側Nm上限、検証状態）と、2軸それぞれの `C620CurrentConversion`（車輪Nm/Aの実効係数、電流符号、A上限、検証状態）を保持する。機構側の計算に用いたヤコビアンと、モータ側の減速・効率の換算は別の責務であり、減速比を二重適用しない。

`CommandStatus` は `Ready` / `Saturated` / `UnverifiedConfiguration` / `InvalidInput` / `ConversionFailed`。通常の上限超過は軸別に制限して要求値・制限値・飽和の情報を返し、飽和だけでFAULTとしない。未検証の設定、符号不明、非有限値、floatの表現範囲外は全軸を原子的に不採用とし、部分的な非ゼロ値を返さない。上流の `BalanceControlOutput.wheelSaturated` を `upstreamWheelSaturated` に分けて保存するため、最終軸飽和との混同を避ける。

**現在はSILプレビュー専用。** `BalanceControlOutput.fixtureOnly` がtrueである数学結果しか取り込まず、返す `MotorCommandPreview.fixtureOnly` も常にtrue。`MotorTargets` やCANフレームへの変換API、MotorServiceへの実際の非ゼロ指令経路は実装していない。テスト用の `verified=true` やCANプロトコル表現範囲（RS06 ±36Nm、C620 ±20A）は実機安全限界の検証を意味しない。

App本体の `ControlCycle` は `MotorCommand::hasVerifiedConfiguration()` がfalseならARMを拒否する。BALANCING中の非有限値や換算失敗は同周期に `RobotSequence` へ `ExternalFault` を伝え、コマンド候補を一切公開しない。成功時でも数学値としてのみ利用する。

[MotorCommand 6軸換算のホスト検証](../verification/f446-motor-command-host.md)

## C++17の6軸CANフレーム計画（SILのみ）

`MotorFramePlan::compose()` は、`MotorCommandPreview` と `RobotSequenceResult` を受け、既存C11コーデックによりRS06 Type 1を4軸分、C620共通 `0x200` を1件符号化する。通常駆動時以外はRS06 Type 1を生成せず、C620ゼロ電流フレームだけを生成する。RS06 Type 4停止とType 3有効化は既存 `MotorCanLink` が一元管理し、停止キューを二重化しない。

この処理の生成物は常に `fixtureOnly=true` の数学プレビューであり、F446の実機起動経路にもCAN送信関数にも接続しない。RS06 250Hz/C620 500Hzの周期・送信期限・停止優先は既存 `CanTxScheduler` が所有するため、符号化側で再実装しない。実機のメールボックスに既に受付済みのトルク指令はソフトウェアから取り消せないので、停止の遅延については実機で別途検証する。

[CANフレーム計画・2バス停止優先のホスト検証](../verification/f446-motor-frame-plan-host.md)

## ControlCycleを本体Appへ移す（実機非ゼロ駆動なし）

`ControlCycle::step(ControlTickInput)` は、元の`tests/support/SilControlCycle.hpp`の数学経路をApp層へ移し、`StateEstimator` → `RobotSequence` → `BalanceControl` → `MotorCommand`を一周期にまとめる。初回の実測`firstElapsed`が必要であり、未取得時に2msを捏造しない。以降は観測スナップショットの共通`sampledAtUs`から実測時間差を得る。6msの更新空白をFAULTとするのはBALANCING中のみ。非常停止・DISARMがすでに分かる周期は数学トルクの候補を計算しない。

本体化は**実行経路の公開クラスと処理順の確定**であり、現時点の`Application::controlTick()`からは呼ばない。実I2C/CAN2、ControlCycleへの単調時刻供給、実機の幾何・ゲイン/限界が未検証のため、数学上の`fixtureOnly`はモータ送信へ接続しない。詳細は [ControlCycle本体化の検証](../verification/f446-control-cycle-core-host.md)。

## Applicationの実測周期診断

既存CAN1ランタイムでDWT時計を回収後に採取し、`balancing_f446_can1_bridge.h`の監視コピーC APIを通してApplicationへ渡す。初回や取得不能後の初回には差分を確定せず、重複・逆行時は診断カウンタを更新して時刻と差分を無効とする。通常の観測時刻と受信時刻を区別し、TIM7回数から実測時間を推測しない。`balancing_app_timing_snapshot()`は読み取り専用であり、ControlCycleの実機呼出・ARM・送信には接続しない。[検証記録](../verification/f446-application-timing-host.md)。

## CAN1/CAN2の共通受信ランタイム

`balancing_f446_can_start()` は両バスの起動完了後だけ監視コピーを公開する。既存のCAN1監視APIと独立したCAN2監視APIを維持し、DWT時計と主ループの回収後評価時刻は共有する。CAN2の分類件数は機器状態や鮮度の確定ではない。起動失敗・通知解除・停止失敗を扱い、復帰不能時は再起動を禁止する。通常CANモードでACK等は発生し得るがアプリケーション送信要求は追加しない。[検証記録](../verification/f446-dual-can-runtime-host.md)。

## C620受信順の累積位置

`C620Motor` は各報告の最短一周差を積算し、`C620AxisFeedback` に通信鮮度とは独立した累積位置・受理番号・原点世代・連続性を返す。半周未満の外部検証済み取得間隔設定が必須。CAN2のRxLost/BusOff、既知IDの復号失敗、時刻逆行、半回転差などで連続性を無効にし、正常報告で自動復旧しない。`WheelOdometry` は累積位置を距離とRPM速度へ換算する。明示原点再設定には `MotorService::resetWheelReference()` と `StateEstimator::reset()` の同期が必要。実機の上位操作は未接続。[検証記録](../verification/f446-c620-continuity-host.md)。


## Applicationの観測周期統合（非駆動）

ApplicationはMotorObservationSnapshotとImuAttitudeを同じ周期時刻で固定し、ControlCycleへ渡す。既定プロファイルは校正・モデル・駆動限界・電源・機構・操作条件が未検証で、ARM/指令送信に接続しない。ApplicationObservationは周期・6軸・姿勢・状態推定・FAULTの診断コピー。取得不能・時計異常では旧周期結果を返さない。最新回収番号の異常通知消費はFAULT解除とは独立する。[統合検証](../verification/f446-application-observation-cycle-host.md)。

## CAN2 v1共通コーデック（実機未接続）

個別Codecから `protocol::encodeId()/decodeId()` と `findMessage()/matchesProtocolFrame()` を使用する。35件の固定IDとDLCを照合し、DJI予約領域を除外する。 `ProtocolPayload` の整数・float32変換はDLC境界と非有限値を検査し、HALや送信経路に依存しない。形式検査と状態機械の操作許可は別責務。 [検証記録](../verification/f446-protocol-codec-host.md)。

## F303電源通信コーデック（実機未接続）

`PowerBoardCodec` は電源状態、投入・遮断要求、処理結果、共通非常停止の形式を検証し、`CanFrame` と論理データを相互変換する。未決定の状態フラグ・故障ビットは生値のまま。鮮度、要求実行、操作許可は別責務。 [検証記録](../verification/f446-power-board-codec-host.md)。

## Raspberry Pi操作通信コーデック（実機未接続）

`RemoteControlCodec` は運動指令、操作要求、操作結果と `CanFrame` を相互変換する。有限値と固定形式を検査し、操作元指定はSELECT_SOURCEのみ許す。物理上限・操作元の権限・受信鮮度・状態遷移は別責務。 [検証記録](../verification/f446-remote-control-codec-host.md)。

## 診断・設定読み出しコーデック（実機未接続）

`DiagnosticsCodec` は19種類のDTOに対する `encodeDiagnostic()` と、IDに対応する型を `DiagnosticPayload` として返す `decodeDiagnostic()` を提供する。観測鮮度・配信許可・FAULT再構成・設定番号の意味は別責務。未定義フラグと設定値は生ビット。 [検証記録](../verification/f446-diagnostics-codec-host.md)。

## CAN2受信分類

`CanRouter::dispatch()` は自作CAN2の12受信IDを6分類へ振り分け、従来の `CanEventSink` に採取時刻付きで渡す。 `MotorCanLink` は既存軸だけを担当。内容検査とサービス接続は下記の非駆動CAN2統合で行う。 [検証記録](../verification/f446-can2-rx-routing-host.md)。

## CAN2受信フィルタ

`makeCan2FilterPlan(false)` はC620標準2 IDを1バンクに、trueは自作12 IDを追加し計7バンクに配置する。32bit Listを使用し、CAN1のMaskは維持する。C ABIの既定起動はfalse。明示的な非駆動試験設定とC++起動指定が揃った場合だけtrueを使用し、未使用バンクは明示無効化する。 [検証記録](../verification/f446-can2-filter-plan-host.md)。


## 電源状態の受信境界（模擬CAN接続）

`PowerBoardService::acceptCanStatus(bus, target, frame, receivedAtUs, nowUs)` はCAN2のPOWER_STATUSだけを復号し、既存の `acceptDecoded()` に委譲する。ポート採取時刻を保持し、評価時刻を受信時刻に置換しない。 `PowerStatusReport::wire` は状態フラグ・故障ビット・報告連番の生値を持つ（従来の模擬入力は空）。500 msの鮮度と保留FAULTの既存規則を維持する。連番の重複・周回・送信側再起動の契約は未確定。この受信境界を下記の非駆動CAN2統合から利用する。実機電源操作は未接続。 [検証記録](../verification/f446-power-status-receive-host.md)。


## CAN2通信統合（非駆動プロファイル）

| API | 責務 |
|---|---|
| `Can2ServiceLink::receive(bus, target, frame, receivedAtUs, nowUs)` | CAN2自作通信を既存codec/サービスへ配送。E-STOPとDISARMは通常要求・応答容量から分離 |
| `takeCycleInput(nowUs, state, clearFaultAuthorized)` / `completeCycle(result, nowUs)` | 一周期の操作・異常入力と状態機械結果に基づくREMOTE_RESULT。操作元・ARM条件・解除の最終権限は既存サービスとRobotSequence |
| `publishTelemetry(frames, nowUs)` | 10種類の通常診断フレームを有効な最新値で置換。未成立の値は空 |
| `submitPending(scheduler, nowUs)` / `onTxTerminal(bus, token, outcome, atUs)` | 背景送信を1件だけ試行し、FAULTの3部分は送信終端で進める。通常応答は失敗・満杯・期限切れで欠落計数 |
| `CanTxScheduler::trySubmitBackground(frame, token, nowUs)` | 停止・通常・周期待ちがあれば拒否。背景1枠のみ、背景終端で周期送信を待たせない |
| `initializeCan2Observation(profile)` / `startCanObservation(..., true)` | 起動前の明示設定とフィルタ指定が両方必要。既定C ABIは車輪受信のみ、実機電源・駆動禁止を維持 |

`Can2ObservationProfile` は人工の入力設定、通信期限・配信周期・応答上限、外部固定FAULT領域を受け取る。値を実機の確定値とみなさない。通常運転での使用は別途設計・検証が必要。

購読はRegister 2〜9について設定された周期・期限内のみ。未知のPARAM_READにはPARAM_ERRORを返す。F446からの電源要求は独立した模擬リンクでだけ許可し、Applicationでは無効。POWER_RESULTは要求番号・種別と送信終端を照合して保持するがPOWER_STATUSの代用にはしない。同一IRQでTX終端とRXを観測し先後が不明な応答は保守的に採用しない。

[検証記録](../verification/f446-can2-service-integration-host.md)。


## Application通常運転経路（ホストSIL専用）

`DriveCycle` は既存のControlCycle・RobotSequence・MotorService・MotorCanLink・両CanTxSchedulerを参照する。MotorFramePlanの軸IDはMotorServiceの起動設定から取得し、重複設定しない。

| API | 責務 |
|---|---|
| `initializeDriveSil(profile)` | 人工の推定・制御・換算・ARM期限・送信期限・CAN2設定を起動前に一回注入 |
| `bindApplicationRuntime(motors, link, tx1, tx2)` | 共通入口から起動済みCAN所有者をリンク済み実行境界へ配送。本番側は非駆動 |
| `DriveCycle::step(input)` | 実際の4軸ARM応答・送信完了監視を集約し、ControlCycleを一度実行、状態に応じたフレームを予約 |
| `DriveCycle::poll(nowUs)` | 有限回のCAN受付、異常時の旧予約破棄と停止方向のみの抑止 |
| `DriveCycle::onTxTerminal(bus, token, outcome, atUs)` | 最新世代の送信完了だけ鮮度更新。バス・軸・世代を混同しない |
| `CanTxScheduler::invalidatePeriodic()` | 未受付の周期フレームを破棄し、受付済みに中止要求。対応する軸は終端まで停止送信を待つが他軸は独立 |

`ControlTickInput::movingTarget` の候補則は走行時に実測dtで目標位置を移動させ、速度0では現在位置へ戻す。物理モデルに対する制御性能の比較は後続。通常指令はCAN1 4ms / CAN2 2ms、ARMINGと非駆動は車輪ゼロ、停止は既存MotorServiceが軸別に管理する。

共通の `drive_runtime` はビルド対象ごとに別の実装をリンクする。F446側は `App/Src/balancing/app/DriveRuntime.cpp` の非駆動観測、SIL側は `tests/sil/DriveRuntime.cpp` と `tests/sil/DriveCycle.cpp` を選択する。`initializeDriveSil` と `driveSilObservation()` は試験側だけに存在する。試験用ヘッダーはARM/Thumbコンパイルをエラーにする。C ABIの既定起動は非駆動のまま。実機にfixtureを昇格させない。送信中止は終端が来るまで物理的な取消を意味しない。

[検証記録](../verification/f446-drive-cycle-sil.md)。


## DualSense USB入力境界

| API | 責務 |
|---|---|
| `DualSenseHidCodec::decode(data, length)` | 明示したID・長さ・軸・ボタン配置を検査、SI操作量に復号 |
| `UsbHidInput::connect/disconnect/receive` | 主ループで転送完了を世代・連番・採取時刻で確認。欠落時は連続性無効 |
| `DualSenseService::takeEvents(nowUs)` | 新規報告で成立した長押しを一度だけ回収。ARMは選択元に限定、DISARM優先 |
| `applicationDualSense()` | 明示設定済みの入力入口、既定は空 |
| `selectApplicationUsbSource(target, nowUs)` | USB側からの明示選択要求を現在のRobotSequenceで評価。割当は未確定 |

USB受信完了通知は主ループに配送し、再読出し時刻で鮮度を更新しない。既存RemoteControlServiceに速度目標・中立復帰・変化率を委ねる。USB Host列挙・HIDドライバは明示設定による起動経路を実装済み（既定無効、実基板未検証）。[検証記録](../verification/f446-dualsense-input-host.md)。

## 状態表示出力

| API | 責務 |
|---|---|
| `StatusIndicator::update(state, fault, nowUs)` | 既存論理パターンと点滅・鳴動期限 |
| `StatusIndicatorPort::apply(state)` | RGBとブザーの論理状態を検証済み極性で出力。変更時のみ書込み |
| `StatusIndicatorPort::disable()` | 観測欠損、時刻逆行、再初期化時のOFF要求 |
| `StatusIndicatorPort::summary()` | 設定有効性、失敗ラッチ、失敗回数、適用済み表示のコピー |
| `makeF446IndicatorOutput(activeHigh, verified)` | ARMビルド限定の生成main.hピン割当。構築ではGPIOを書かない |

論理表示と物理出力の両方が明示検証済みの設定だけApplicationに結合する。GPIOラッチ照合は物理発光・鳴動を保証しない。未設定の既定起動は出力しない。PWMやLED1〜5の用途・色・極性を新規に決めない。表示異常は診断コピーに保持しRobotSequenceやCAN契約の意味を変えない。

[検証記録](../verification/f446-indicator-output-host.md)。

## STM32F446 USB Host転送境界

| API | 契約 |
|---|---|
| `validUsbHostConfig` / `findDualSenseInterface` | 明示設定とVID/PID、Full Speed非Boot HID、Interrupt IN descriptorの境界を照合 |
| `UsbHostPort::start/poll` | 一度だけ起動、pollあたり下位イベント1件と新規転送受付最大1件 |
| `UsbHostPort::takeConnectionEvent/takeReport` | 接続世代・転送連番・完了採取時刻付きの通知を一回だけ回収 |
| `startApplicationUsbHost(config, backend)` | 設定済みDualSenseServiceと共通時計へ接続、既定無効 |
| `startF446UsbHost(board, config)` | 既存OTG_HS HCD、基板固有VBUSコールバック、明示期限、TIM7より低いIRQ優先度を照合 |

実USB Hostの所有中は手動の報告通知を混在させない。NAK・異常長は連続入力を無効化、STALL・期限超過・切断は操作を切断扱いにする。列挙と停止回収は状態遷移で進め、ST HALのIRQ内部待機時間は実測待ち。詳細は[USB Host転送検証](../verification/f446-usb-host-transport.md)。

## 制御周期完了とウォッチドッグ更新

| API | 責務 |
|---|---|
| CycleHealthMonitor::begin(ticket) | タイマ番号と共通単調時計で開始を照合 |
| CycleHealthMonitor::complete(ticket, atUs, processed) | 一致した正常完了だけ新しい証跡を返す |
| WatchdogPort::refresh(completion) | 明示済み更新コールバックを証跡ごとに一回だけ呼ぶ |
| balancing_app_begin_control_tick(event_sequence) | CAN回収前の主ループ開始 |
| ApplicationObservation::cycleHealth / watchdog | 欠落・完了・更新診断コピー |

既存6ms FAULT判断はControlCycle/RobotSequenceに保持する。物理IWDGの開始と期限実測は未実施。[検証記録](../verification/f446-cycle-watchdog-host.md)。
