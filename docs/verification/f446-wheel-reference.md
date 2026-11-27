# F446 車輪原点再設定の統合

基準 master 12c499b、実装 253aca0、試験 a9dabfb。CTRL-EST-001、PROTO-C620-001、BEHAV-001。App新規ファイル0（96ファイル、45 C++、旧C0）。

## 実装

ApplicationObservation.hpp の明示API resetApplicationWheelReference と起動時許可。application.cpp はDISARMED・周期外・未消費操作/FAULTなし、推定器校正と時刻を検査。f446_can1_runtime.cpp は短いIRQ排他区間で両CANの未配送イベント、FIFO0、保留異常、受付済みTX、RS06停止待ちを確認する。空でない場合は拒否し、イベントを廃棄・再初期化しない。左右原点世代と時刻の事前照合後だけMotorServiceと推定器を同期して進める。以前の公開観測・推定・SIL結果・未受付テレメトリを無効化する。回収番号・FAULT記録・制御周期・IWDG証跡は維持。

C620Motorはローカル原点境界と同一または古い時刻の正常報告を新原点に採用しない。両輪の新規報告が揃うまで推定は成立しない。WheelOdometryは以前の世代のコピーを拒否する。形式不成立や世代・時刻不一致のときに片輪だけ変更しない。受信復旧・ARM/DISARMでは自動再設定しない。数値パラメータ・CANワイヤID・安全閾値は不変。

## 試験

- SIL: 既存積算→FIFO/IRQキュー・バスオフ・未配送RxLostで拒否→欠落→明示再設定→同一採取時刻を除外→片輪のみ→左右成立・零→積算→再設定。未配送非常停止を保全しFAULT、FAULT中拒否。
- SIL: 起動許可なし、ARMING、BALANCING、DISARM後の旧TX/停止待ちで拒否。停止終端後の明示再設定で零。通常既定は拒否・非駆動。
- 単体: 片輪設定不正、世代不一致、片輪の未来時刻、同じ境界で拒否し両輪の旧状態保持、旧時刻排除。世代上限はコードに拒否条件を持つが、64bit上限到達を強制する試験は未実施。
- 最終CTest 74/74。通常・UBSan全体回帰 d4beb0b396824e36 PASS。最終追加ケースのUBSan 4/4（36a196ad479d4a04）もPASS。
- ARM管理cleanBuild d6c9ef73a543481d: Debug/Release/BenchDebugすべて45 Appソース、0 error/0 warning。nmで全構成にSIL DriveCycle、`MotorFramePlan::compose`、`MotorBench::requestPulse` なし。原点APIは実機のmainや入力から未接続で全ELFにリンクされない（ソースはコンパイル済み）。Debug text222344/data2828/bss13368 B、Release text77188/data2828/bss13368 B、BenchDebug text83808/data2748/bss14336 B。
- CAN契約19/19、CubeIDE設定24/24、仕様516参照。

物理原点の同時性、送信側の生成時刻、実機での受信境界・IRQ排他時間、エンコーダ校正は未確認。API呼出の実機操作経路は未実装、既定無効。実機・基板への操作なし。
