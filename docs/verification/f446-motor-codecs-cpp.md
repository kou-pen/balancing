# モータ通信コーデックのC++統合

基準 master d31ff9e。PROTO-RS06-001 / PROTO-C620-001。

## 実装と責務

| ファイル | 内容 |
|---|---|
| communication/Rs06Codec.hpp/.cpp | Type3/4、Type1トルク、Type2/24復号を単一所有 |
| communication/C620Codec.hpp/.cpp | ID1〜4の共通電流指令と0x201〜204の復号 |
| communication/can/CanTypes.hpp | 既存CanFrameを共通使用（型自体は変更なし） |
| app/MotorService、MotorCanLink、device/Rs06Motor、C620Motor | C++型を直接渡す。送信完了・停止・鮮度・連続性の既存判定を維持 |
| communication/can/MotorFramePlan、bench/MotorBench、tests/sil/DriveCycle | 共通CodecとCanFrameを使用。旧C/C++フレーム変換を廃止 |
| communication/balancing_motor_can.h/.c | 廃止 |

通信ID、IDE、DLC8、データ順序、RS06角度・速度・トルクレンジ、C620電流量子化は不変。Type24を有効化確認へ昇格させない。予約RS06モード3は従来のC++境界同様拒否。未定義Rs06Command値を暗黙に停止へ変換していた挙動は明示拒否とした（正当なEnable/Stopは同一）。実機安全上限・駆動許可・通信周期は変更しない。

## 試験

- 旧Cゴールデンフレーム試験をtest_motor_can.cppへ移行。最大/最小量子値、非有限値、予約モード、拡張ID範囲、0 ID、全4電流チャネル、符号付き極値を追加。
- MotorFramePlanの既存Codecとの自己比較を、独立の期待ID・バイト列比較へ変更。
- CAN契約監査と19件の変異試験を新Codecへ追従。既存CANルーティング、試運転、停止、SIL全経路を回帰。
- 旧Cの実装をGit履歴から一時的にbuild/codec-differentialへ抽出して、通常とUBSanの両方で直接比較。各393600件一致、予約モード128件は既存C++と同様に拒否。Type3/4の全モータ/主機ID組合せ、65536点のトルクと電流、16bit全値の各復号、Type2/24・モード・故障ビットの組合せを検査。旧ソースは製品・通常試験へ再導入しない。
- CTest48/48（fce67243c6fc4e8b）。実機通電、書込み、実機FWの量子化範囲・通信測定は未実施。

通常/UBSan全件PASS（9cf2e28f86ae4d12）、CubeIDE Debug/Release全48 Appソース・0 errors/0 warnings（e7f0d8c115584356）。Release text=69456, data=2828, bss=12216 B（textは基準から200 B減）。仕様454 API参照、CAN監査19/19、CubeIDE設定22/22、Doxygen全sourceリンク、diff check PASS。両ELFに旧モータC API・SIL・試運転シンボルなし。旧比較の再実行はVM build/codec-differential/compare.cpp、旧Cの基準はd31ff9e。Appファイル102のまま、Cソース6→5。
