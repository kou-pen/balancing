# F446の小規模ファイル統合

目的: 機能を跨ぐファイル移動を減らす。クラス・公開関数の動作、安全判定、周期、CAN ID、HALと試験の分離は維持する。

| 旧配置 | 新配置 |
|---|---|
| Can1Observation.h / Can2Observation.h | balancing_f446_can1_bridge.h（監視型・C ABI） |
| CanEventPoll.hpp/.cpp | CanRouter.hpp/.cpp（交互回収と配送） |
| Can2ServiceLinkTx.cpp | Can2ServiceLink.cpp（背景送信を含む） |
| Led.hpp / Buzzer.hpp | IndicatorOutput.hpp |
| DualSenseHidCodec / DualSenseHostClass | DualSenseProtocol.hpp/.cpp |
| UsbHidInput.hpp/.cpp | DualSenseService.hpp/.cpp |
| CycleTicket.hpp / WatchdogPort.hpp/.cpp | CycleHealthMonitor.hpp/.cpp |

App配下124→111（13減）。StateMachineは既存設計に従い維持。DriveRuntime、CanPort、CanTxScheduler、HAL依存USBアダプタは独立。旧Cスケジューラ・単軸試運転の移行は別PRで扱う。過去の検証記録に出てくる旧ファイル名は当時の配置。公開ヘッダーのincludeパスのみ変更し、関数・型名は維持する。

Can2ServiceLinkの送信部分を同じ翻訳単位へまとめたため、Application単体試験にもCanTxScheduler/CanPortをリンク。HALスタブはtests側のみに置く。本番に試験マクロは追加しない。

検証: CTest 49/49（c64cce340ea940a6）。通常/UBSan全件PASS（38406431535c44f6）。CubeIDE Debug/Release、全52 Appソース、0 errors/0 warnings、ARM ELF32（e0a775b178cc4736）。Release text=69820/data=2828/bss=12216 B。仕様・Doxygen・CubeIDE設定・diff検証済み。
