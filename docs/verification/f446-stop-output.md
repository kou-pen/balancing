# 通常運転の停止専用CAN送信経路

基準 master a2f309a。Issue #27設計承認済み。BOARD-F446-001 / BEHAV-001 / PROTO-RS06-001 / PROTO-C620-001、T-F446-08〜11。実機操作なし。

## 実装

Appのファイル数96、C++45、旧C0を維持。DriveRuntime.hpp/.cppと既存Application・CAN起動境界に接続。新規Appファイルなし。SILのDriveCycleとMotorFramePlanは製品へ接続しない。実機用ARM期限、モデル、ゲイン、変換、物理制限は未確定のまま。

MotorObservationConfig末尾のStopOutputConfigはenabled=false/verified=falseが既定。明示有効時はRS06 4軸の非ゼロ・非重複ID、mappingVerified、バス配線/主機/C620 ID1/2と未使用ID3/4の照合済み設定、自作CAN2有効、内部トークン発行元が診断と異なることを要求する。0xD001は内部キーでワイヤIDではない。C ABIのmainからの既定起動は送信なし。通常の受信専用起動の再呼出し挙動は保持し、停止専用設定が関与する二重起動は拒否する。SIL実装はこの実機停止設定を受け付けない。

## 周期と送信

Applicationの既存番号付き周期開始→CAN回収→ControlCycle→送信処理→診断・完了監視を維持する。停止側の新しい予約世代は進行中ticket・新規回収・単調時刻・前回処理番号との連続性を検査して生成する。重複、欠落、観測なし、回収を省略した周期で新しい世代を生成しない。欠落後に過去周期をまとめて生成しない。初回番号以前の欠落は検知できない。背景pollは生成済み要求の処理とRS06停止再試行だけを有限回進める。時計が逆行すれば処理しない。既存CycleHealthMonitor/IWDG更新条件を変更しない。

RS06停止はMotorService/MotorCanLinkが単一所有。StopAll以外の要求を渡さない。送信受付と終端、失敗、無効化報告を区別する。C620はCodecで0x200の全8バイトゼロを生成し、既存2ms予約期限・最新世代置換を用いる。受付済み旧世代を重ねない。期限切れ/置換を欠落として計数、終端失敗と欠落を区別する。旧/別バス/別発行元の終端では送達数を進めない。診断は既存背景枠を共用。出力起因の明示送信異常は次周期に既存状態機械へExternalFaultを渡し、停止処理を続ける。送達期限の実機安全閾値は追加しない。

StopOutputSummaryは最後に評価した周期のコピー。ゼロの予約世代・受付・送達・欠落・失敗、最新送達時刻、出力起因の故障履歴、既存RS06軸状態を保持。Bus-off/RX lossは既存MotorObservationSnapshot/RobotSequenceで扱う。診断コピーは物理停止保証ではない。

## ホスト試験

tests/test_f446_stop_output.cpp を実製品DriveRuntimeとリンクし、SILをリンクしない独立実行ファイルを追加。6ケース: default（既定ゼロ送信）、collision（内部発行元競合拒否）、normal（無効設定拒否、RS06三枠→四軸、C620ゼロ、ARM要求、背景枠）、timing（期限/置換/重複/欠落/未回収とIWDG）、faults（TX error、Bus-off、Rx loss、復旧）、retry（RS06停止失敗再試行、旧/別バス終端、停止後の新規Reset応答）。送信レジスタのID・IDE・DLC・データを独立期待値で確認。既存64件の回帰試験も保持。

## 制限

実機設定注入・CAN2有効化、物理停止、回生/電源安全、送信実時間・負荷、実機ゲイン・ARM・非ゼロ駆動は未検証。停止専用機能は既定では送信しない。BenchDebug・IWDG既定設定は変更しない。F303/Pi側の対向実装は別件。

## 結果

- CTest 70/70（初回69/69、追加retryを含め再実行70/70）、通常/UBSan全件（c071ac24c6c347e2、追加retryも通常・UBSan各直接実行PASS）。
- CubeIDE Debug/Release/BenchDebug 45 Appソース、全0 errors/0 warnings（cfd4aca490b84715）。nmで通常だけに停止専用起動境界、全3構成にSIL/試運転非ゼロ入口/`MotorFramePlan::compose` がないことを確認。独立停止試験のホストELFにもSILシンボルなし。
- Release text75352/data2828/bss12584 B（基準よりtext+5500、bss+312）。Debug text215924/data2828/bss12584、BenchDebug text83660/data2748/bss14272 B（基準よりtext+12、bss+96）。非駆動から既存の停止送信処理をリンクしたためサイズ増加。RAM/WCETの実機評価は未実施。
- 仕様476 API参照、CAN契約監査19/19、CubeIDE設定24/24、Doxygen全sourceリンク、diff check合格。
