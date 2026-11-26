# F446 USB Host転送とHCD接続

基準 master af834cc。PROTO-PS5-001、BOARD-F446-001、BEHAV-001、T-F446-06、T-PROTO-07。

## 実装と責務

| ファイル | 処理 |
|---|---|
| communication/usb/DualSenseHostClass.hpp/.cpp | VID/PID、Full Speed、非Boot HID、明示interface/alternate 0、IN endpoint、packet長・intervalを照合。生descriptorは512 B以内、各bLengthを検査 |
| device/UsbHostPort.hpp/.cpp | 接続世代・連番を発行し一件のIN転送を管理。完了時刻・実長・トークンの検査、期限超過と旧完了の隔離 |
| device/F446UsbHostPort.hpp / f446_usb_host_port.cpp | 既存OTG_HS HCDとST Coreを接続。基板設定、IRQ、非同期resetとチャネル停止/FIFO回収 |
| app/application.cpp / ApplicationObservation.hpp | 設定済みDualSenseServiceに主ループで一度だけ通知、診断集計。既定では開始しない |
| Middlewares/ST/STM32_USB_Host_Library | STM32CubeF4 V1.28.3 USB Host Core V3.5.3をSTM32向けに収録。元ファイルSHA256はUPSTREAM.txt |

## 動作契約

- 明示的なverified設定、VID/PID、interface、endpoint、report長、interval、転送期限が必要。ボード側もHCD、VBUSコールバック、列挙・回収期限、TIM7より低いIRQ優先度を必要とする。人工設定の本番採用なし。mainから起動関数は呼ばない。
- 固定64 Bの受信領域と一件の転送を使用。pollは下位イベント最大一件、受付最大一件。IRQ完了採取時刻を配送時刻に置き換えない。切断を優先し、旧世代・旧連番を拒否する。
- NAK・不正長は長押しと入力連続性を無効化する。STALL・転送期限超過は切断扱い、下位チャネルとFIFOを回収し抜去まで再起動しない。正常な再接続は新世代。
- ST Coreの200 ms接続安定待ち、100 ms reset assert、10 ms reset recovery、100 ms reset後待ち、2 ms address安定待ちを状態化。制御転送・回収は主ループで進める。標準Boot HIDクラスは使用しない。文字列取得・remote wakeupは不要なため省略し、異常descriptorを境界検査する。
- ST HALの既存IRQ内部にはチャネル停止・FIFOフラッシュの有限待機ループが残る。実基板のIRQ WCETと500 Hz制御との干渉は未計測であり、完全非ブロッキングや実時間保証を主張しない。
- 既存ARM・DISARM・減速・ニュートラル復帰の判定はDualSenseService、RemoteControlService、RobotSequenceのまま。生成Core/Drivers、CAN ID、制御パラメータ、安全閾値は変更なし。

## ライセンス

ローカルCubeF4 V1.28.3のパッケージSBOMに従いSLA0048 Rev4の該当条項全文をPackage_license.txtに保持（行末空白・末尾空行のみ正規化）。コンポーネントのLICENSE.txt（SLA0044 Rev5）も保持。STM32向け利用・再配布条件およびOpen Source Termsの制限に従うこと。ST CoreはARMビルド専用で、ホスト試験には自作クラスと模擬backendのみを組み込む。末尾空行は差分検査に合わせて正規化。変更は待ち状態、interface探索・選択、descriptor境界、LLエラー伝播等に限定。

## 検証結果（2026-11-26）

| 検証 | 結果 |
|---|---|
| CMake Debug / CTest | 48/48 PASS（job 9eac799e4c504e1b、終了0） |
| 通常ホスト / UBSan | 全件PASS（job 356613d3d0aa42f9、終了0） |
| CAN監査 / CubeIDE設定 | 19/19、22/22 PASS |
| CubeIDE 1.19.0 Debug/Release | 55 Appソース + ST Core 4 Cソース、0エラー/0警告 |
| ARMリンク検査 | 両構成でstartF446UsbHostを強制リンクし、USBH_Process・OTG_HS_IRQHandlerを確認、SILシンボルなし |

ARM job a89e91a87a934918。通常Release text=68536 B、data=2828 B、bss=11896 B（USB明示起動は既定無効）。強制リンクELFは検証専用で書き込まない。

追加試験 --usb-host: 複合descriptor、誤設定・不正長・旧世代/連番・NAK・STALL・期限、400 ms列挙待ち中の201回poll、転送間隔。--drive-sil-usb-host: Applicationの転送境界を介した操作、ARM、切断減速、再接続、DISARMを模擬CANと結合。

## 実機側の残件

基板のVBUS供給方式・過電流・配線、実際のDualSense VID/PIDとdescriptor/report設定、列挙・HID報告の取得、切断回収の実機挙動、IRQと500 Hz周期の最大実行時間。ST Core/HCDの実行はホスト試験ではなくARMコンパイル・リンクのみ検証した。実機通電・書込み・Host有効化は行っていない。
