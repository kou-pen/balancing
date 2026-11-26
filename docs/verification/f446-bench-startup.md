# BenchDebug専用起動とCAN IRQの排他検証

基準 master 79c90a1。BOARD-F446-001、BEHAV-001、PROTO-RS06-001、PROTO-C620-001、T-F446-11/12。Issue #27設計承認済み。実機操作なし。

## 実装

App新規2ファイル：`App/Inc/balancing_f446_bench_bridge.h`、`App/Src/balancing/bench/f446_motor_bench_runtime.cpp`。既存MotorBench、CanPort、CanRouter、CanTxScheduler、機器・Codecを共用。App92→94、旧Cソース0を維持。通常CANのDWT時刻処理をCanPort.hppのCanClockへ移した（32bit差分、IRQ排他、周回処理は同一）。

main.cとstm32f4xx_it.cのUSER CODEのみをビルドマクロで分岐する。両バスの生成初期化を残し、選択バスのみ14バンクを設定・開始・通知有効化する。RS06 Type2/24、軸/主機ID、IDE/RTRを厳密にマスク、C620対象1〜4の標準IDのみ受理。CAN2のCAN1クロック依存を維持。IRQは構築済みの選択ポートへ一度だけ配送。失敗時は通知停止とポート破棄を排他、HAL停止不能なら再試行拒否。

チェックイン済みboardConfigは無効。初期化を拒否しmainはError_Handlerへ進む。通常起動への切替なし。設定を確認済みとして扱う実機変更は行わない。C++のホスト試験から人工設定を注入し、初期停止と受信を確認する。非ゼロ要求関数への入口はない。時計停止、逆順・重複周期番号、6ms以上または3周期相当の欠落で故障をラッチし、既存停止処理を継続。時刻は1周回未満の間隔で読み取る前提。

## 受入試験

`tests/test_f446_bench_runtime.cpp` の独立プロセス5ケース（rs / wheel / failures / rollback / gap）。無効設定、誤バス、初期停止のみ、全標準ID/RS06マスク、IRQ有効化中の構築状態、二重起動拒否、失敗ロールバック、停止不能ラッチ、DWT周回、DWT停止、FIFO欠落・Bus-off、1周期欠落と6ms境界。模擬バイト列はType4および0x200全ゼロ。既存パルス単体試験は維持するが実機起動からは呼ばない。

CubeIDEの3構成は個別ELFとマクロを持ち、通常起動と試運転の起動/周期/IRQ関数をnmで検証する。通常ELFに試運転、BenchDebugに通常Application/RobotSequenceの制御経路がないこと、すべてのELFにrequestPulse/SILがないことを検証する。全App翻訳単位をコンパイルし、不要な関数をリンク時に除去する（全通常クラスの静的データが除去されるという意味ではない）。

## 未検証

実機ID・主機ID・CAN通信・実時間、物理非常停止・支持治具・IWDG、非ゼロ要求の入力接続、書込み・通電。CAN停止指令は物理停止を保証しない。基板への試験設定投入や安全閾値変更は別承認。

選択モータ以外のフレームを受理しないため、通常CAN2の操作・非常停止プロトコルはBenchDebugに接続されない。現段階は停止確認専用であり、非ゼロ実機試験の安全機構としては未完成。独立した物理非常停止は別に必要。

## 実測した検証結果（2026-11-26）

- CTest 53/53（ea6fac43c01547fa）、通常/UBSan全件（511233711c3245d0）、CubeIDEメタデータ24/24、CAN監査19/19。
- CubeIDE 1.19.0管理cleanBuild 3構成（4ef1e3b3408747af）：各44 Appソース、0 errors / 0 warnings。ARM ELF32とnmシンボル分離確認。
- Debug text=200292/data=2828/bss=12232 B、Release text=69220/data=2828/bss=12232 B、BenchDebug text=80000/data=2748/bss=13960 B。Releaseは共通時計化によりtext+12 B。
- 仕様462 API参照、Doxygen（既知日本語翻訳警告のみ）、全ナビゲーションsourceリンク、Git差分検査合格。
- 初回の設定生成でID置換がXMLバージョンにも及ぶ問題を検出し、id属性だけの置換へ修正。再生成後の3構成実ビルドで確認。6ms契約の境界試験とモード分離検証を追加し、最終版を再ビルド・再試験した。
