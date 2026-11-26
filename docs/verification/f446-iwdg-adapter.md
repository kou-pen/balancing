# F446 IWDG実機用アダプタ

基準 master ff9d338。BOARD-F446-001、BEHAV-001、T-F446-02/07/12。設計承認済み。実機起動・書込み・通電は未実施。

## 実装と起動条件

AppにF446Watchdog.hpp/.cppの2本のみ追加。App94→96、.cpp44→45、旧Cソース0。STM32CubeF4 V1.28.3のHAL IWDGヘッダー・ソースを原本のまま追加（既存HAL共通ソースおよびLICENSE.txtと同一パッケージをバイト比較）。HAL_IWDG_MODULE_ENABLEDを明示し、IOCではIWDGの自動起動を設定しない。再生成時の欠落・自動起動をsyncの監査で拒否。

既定boardConfigは無効。設定は公称LSI32kHz、分周32、reload499のみ候補とする（公称500ms。実測値ではない）。mainの最初のUSER CODEでRCC CSRを一度コピー。通常または試運転ランタイムとTIM7開始成功後に一度だけ起動判断。FLASH OPTCRのWDG_SWがhardware modeなら既定無効でも起動拒否するが、既に自動開始したIWDGを停止する機能ではない。明示有効時は未検証・候補値外・DBGMCU IWDG freezeを拒否する。option byte・debug設定を書き換えない。起動後のデバッガによる変更を監視する機能ではない。

起動前に選択ランタイムへWatchdogConfigを結合。結合成功時だけ保存済みリセットフラグをクリアする。HAL_IWDG_Initは開始キーを先に書き込み、完了時に初回reloadする。失敗後は再試行せず、mainのError_Handlerへ。無効時・事前検査失敗時はHAL Init/Refresh、フラグ消去を行わない。RCC CSRはstickyフラグを含む生値であり唯一のリセット原因と断定しない。診断はC++のf446WatchdogSummary。FAULT永続化は保留のまま。

通常Applicationは既存の周期完了証跡を使用。BenchDebugも同じCycleHealthMonitorとWatchdogPortで停止処理完了を証明する。開始番号が欠落・重複・逆順、前周期未完了、開始時計が停止・逆行した周期は更新しない。開始・完了の時計値は同じマイクロ秒でも許容するが、異なる周期の開始時刻は進行必須。既存の6ms FAULT条件、処理完了に基づくFAULT中の更新、周期番号周回を保持。更新ポートの再結合と設定済みApplicationの再初期化は禁止。HAL Refresh失敗をラッチして以後HALへ更新しない。HAL Refreshの成功はレジスタ書込みAPIの成功に過ぎず物理リセット期限を保証しない。ISR/background/番号なし周期での無条件更新なし。

## 試験

既存53 CTestに11件を追加。test_f446_can1_runtime.cppの--iwdg-* 10ケース（成功、既定無効、hardware mode、debug freeze、Init失敗、未検証、無効値、未結合、未取得reset flags、周期開始後結合）。test_f446_bench_runtime.cpp --iwdgで時計停止、欠落、FAULT継続、IRQ無更新、二重結合拒否、HAL更新失敗ラッチ。正常側も同様に新規完了、重複、欠落、未回収、時計停止、FAULT、poll無更新、再初期化拒否を検証。ホストHAL mockは実物のLSI、リセット、停止を再現しない。

- CTest 64/64（024ed6d2e18046a3）、通常・UBSan全件（3b354df0036343e9）。CubeIDEメタデータ24/24、CAN契約監査19/19、仕様471 API参照、Doxygen全sourceリンクとdiff check合格。Doxygenは既知の日本語翻訳警告のみ。
- CubeIDE管理cleanBuild 3構成（d2ed5696aa554dcf）、45 Appソース、全0 errors/0 warnings。
- Debug text201396/data2828/bss12272、Release text69852/data2828/bss12272、BenchDebug text83648/data2748/bss14176 B。
- 全3構成のobjects.listにvendor IWDG object、ELFにHAL_IWDG_Init/Refreshと選択側の結合関数を確認。別ELFへ明示startF446Watchdogの強制リンクも合格。通常・試運転/SIL/pulseの排他性維持。強制リンクELFは実行・書込みしていない。

## 未検証

実基板のoption byte、デバッグfreeze、LSI公差と期限、リセット原因、故障注入、500Hz WCET、独立非常停止・支持治具、実機USB・CAN・駆動。実機IWDG有効化は別承認。既定設定は無効のまま。

## ベンダーファイル SHA256

- stm32f4xx_hal_iwdg.h: 5a0259ec13549adf9a21224c7698b14479b777ec95a3996b4527f2abb295d484
- stm32f4xx_hal_iwdg.c: 4b902875a84554e9e608344e1fd47f8b30744444424817a030e87e04fdf53eb6
