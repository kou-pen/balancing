# F446 DualSense USB入力境界

対象: PROTO-PS5-001 / BEHAV-001 / PROTO-REMOTE-001、T-F446-06、T-PROTO-07。基準 master 49ea4ef。

## 実装

| ファイル | 役割 |
|---|---|
| communication/usb/DualSenseHidCodec.hpp/.cpp | 固定長（最大64バイト）・report ID・軸範囲・ボタン配置検査、SI量への復号 |
| device/UsbHidInput.hpp/.cpp | 転送完了の接続世代・連番・採取時刻、欠落による連続性無効化 |
| app/DualSenseService.hpp/.cpp | OPTIONS/○の報告連続長押し、選択元ARM、停止優先、接続直後の押下保持拒否 |
| app/RemoteControlService.hpp/.cpp | USB入力の無効化と既存の減速・中立復帰 |
| app/application.cpp | CAN2選択を確定してUSBイベントと合成、既存RobotSequenceへ配送 |

固定容量の報告を主ループで直接消費するため、独自の動的FIFOは持たない。転送番号の欠落は長押し・中立確認を無効化する。実USB側が同じ転送を再発行しないこと、世代/連番と共通時計を正しく発行することが前提。

## 追加試験

- `--usb-input`: 人工4バイト報告で軸端点、ID/長さ/範囲/設定重複、接続時押したまま、1秒長押しの境界、重複、切断、旧世代、連番欠落、USB/CAN選択、別選択元からのDISARM。
- `--drive-sil-usb`: Applicationと模擬CANの全経路でUSB選択中のPi ARM拒否、USB ARMから4軸確認、走行目標、切断後も倒立を維持する状態判定と目標減速、再接続後のDISARMと4軸停止・車輪ゼロ。

## USB Host導入の調査

- 現行CubeMX設定: OTG_HS embedded PHY Full Speed、PB14/PB15、HCD初期化あり。USB Host middlewareは未収録。
- ST公式 [USB Host middleware](https://github.com/STMicroelectronics/stm32-mw-usb-host) はCore/Classを提供する。標準HID実装の [InterfaceInit](https://github.com/STMicroelectronics/stm32-mw-usb-host/blob/master/Class/HID/Src/usbh_hid.c) はBoot subclassを探索し、キーボード/マウス以外を拒否する。そのままゲームパッドを扱えるとみなさない。
- [STM32CubeF4ライセンス一覧](https://github.com/STMicroelectronics/STM32CubeF4/blob/master/LICENSE.md) ではUSB Host libraryをSLA0044として記載。現在のHALと互換なCubeF4タグの確認、対応するmiddlewareリビジョン・ライセンス確認、非Boot HIDの専用クラス、割込み完了と主ループ配送、列挙の処理時間計測が次工程。
- 実機のVID/PID、USB報告ID/長さ/マッピング、選択操作、VBUSと配線、Host IRQ・列挙、転送成功は未検証。人工設定を本番へ採用せず、生成Core/Drivers、実機通電・駆動は非変更。

## 検証結果（2026-11-26）

| 検証 | 結果 |
|---|---|
| CMake Debug / CTest | 46/46 PASS（追加2モード、job da83befbe21e47a6） |
| 通常ホスト / UBSan | 全件PASS（job 5c571800e17e484e、終了0） |
| CAN監査 / CubeIDE設定 | 19/19、22/22 PASS |
| CubeIDE 1.19.0 Debug/Release cleanBuild | 全50 Appソース、ARM ELF32、0エラー/0警告（job 1d0462d9f9c248d9、終了0） |
| 仕様・Doxygen・差分 | PASS（既知の日本語翻訳警告のみ） |

Release text=61892 B、data=92 B、bss=11472 B。ログ: build/cubeide-headless-dualsense/{Debug,Release}.log。再現: cmake -S . -B build/dualsense -DCMAKE_BUILD_TYPE=Debug、cmake --build build/dualsense -j 8、ctest --test-dir build/dualsense --output-on-failure、python3 tools/run_host_tests.py、python3 tools/verify_f446_build.py。USB実機転送と列挙は未試験。

後続実装: [USB Host転送とHCD接続](f446-usb-host-transport.md)。本ページの未接続という記述は当時の検証状態。
