# F303 CubeF3開発環境とARMリンク検証（2026-11-27）

対象：BOARD-F303-001（F303-05/06/09/10）、Issue #102。実機用ファームウェアの生成、実機書込、通電試験とは区別する。

## 公式SDKとツール

Balancing VMのCubeIDE 1.19.0付属GNU Tools for STM32 13.3.1、CubeMX 6.15.0を使用。SDKは公式[STM32CubeF3](https://github.com/STMicroelectronics/STM32CubeF3) の v1.11.6 を固定した。最新版であることを主張せず、再現性のため明示的に固定する。VMのユーザー領域に配置し、F446のCubeF4パッケージを変更していない。

| 構成 | 固定コミット |
|---|---|
| CubeF3 v1.11.6 | 663d2baae3604244f90ed80d0d757df17e36ada9 |
| CMSIS Device F3 | 5558e64e3675a1e1fcb1c71f468c7c407c1b1134 |
| STM32F3 HAL | 953955afe65f89e60e556bbcdba752597f5da65d |

配置先は `~/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.6`。今回、Driversをsparse checkoutし、Drivers下のsubmoduleを取得した。再構築時には必要なHAL/CMSISのみを取得すればリンク試験に十分。

```sh
SDK="$HOME/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.6"
git clone --depth 1 --branch v1.11.6 --filter=blob:none --sparse https://github.com/STMicroelectronics/STM32CubeF3.git "$SDK"
git -C "$SDK" sparse-checkout set Drivers
git -C "$SDK" submodule update --init --recursive --depth 1 -- Drivers/CMSIS/Device/ST/STM32F3xx Drivers/STM32F3xx_HAL_Driver
python3 tools/build_f303_link_probe.py --sdk "$SDK"
```

既存SDKにcloneを上書きしない。ビルドスクリプトはSDKと必須submoduleのコミット、追跡ファイルの変更有無を照合する。コンパイラはPATHまたはCubeIDE内から取得し、必要に応じて --toolchain でbinを指定する。

## ビルド対象と境界

| ファイル | 責務 |
|---|---|
| [build_f303_link_probe.py](../../tools/build_f303_link_probe.py) | SDK照合、ARMコンパイル・リンク、ELF/ベクタ検査 |
| [f303_arm_link_probe.cpp](../../tests/f303_arm_link_probe.cpp) | 未検証設定の4コアをリンクし、GPIO・CAN・ADC・IWDGを開始しない試験入口 |
| [f303_arm_link_probe.ld](../../tests/f303_arm_link_probe.ld) | 試験専用メモリ領域・起動シンボル |

STM32F303CBは128 KiB Flash、通常SRAM 32 KiBを試験リンカに指定し、CCMは使わない（[ST製品情報](https://www.st.com/ja/microcontrollers-microprocessors/stm32f303cb.html)）。公式CMSISのSTM32F303xC定義・startup_stm32f303xc.sはxB/xC共用。生成したHAL confは公式テンプレートから対象モジュールを選択した試験用であり、CubeMX生成設定ではない。

HAL GPIO/ADC/DMA/CAN/TIM/IWDG等をコンパイルし、代表関数をリンク対象に保持するが呼び出さない。既存PowerController/Monitor/Can/BoardIOおよび共用Codecをリンクする。成果物はgit管理外の `build/f303-arm-link-probe/f303_link_probe_DO_NOT_FLASH.elf`。実GPIO設定やフラッシュコマンドは含まない。

## 実測結果

- 27ソース（C、C++、公式startup）コンパイル合格。ELF32 ARM、hard-float、実行形式のリンク合格。4コアの実装シンボルとHALの代表関数を確認。
- ベクタ位置0x08000000、初期SP 0x20008000、Reset_Handler 0x080019b1（Thumb）をELFから照合。
- size結果：text 9836、data 12、bss 4 bytes（Flash相当9848 bytes、ELFファイル28344 bytes）。スタック・最終FWの使用量や実行時間ではない。
- CubeMX同梱jarの起動を試したところ、Java 21.0.3でHeadlessException。VMのMCP実行環境ではDISPLAY未設定、Xvfbコマンドなし。CubeMXによる実プロジェクト生成は未実施。ユーザー領域のSDK cloneはGUIパッケージマネージャの導入成功とは同義ではない。

CMake Debug全CTest 85/85 PASS（job 8baa6b49f32f4b18）。全通常/UBSan回帰試験PASS（job 6f5650e87d824d82、CubeIDE設定25/25、CAN監査19/19）。仕様整合・Doxygen生成・差分検査PASS（既知の日本語翻訳警告のみ）。

## 次段階

applicationの周期処理、遮断イベント、出力拒否時の状態・CAN候補破棄、IWDG更新条件を先に論理試験できる。CubeMX生成と実GPIO/ADC/DMA/CAN/IRQ/IWDG接続、クロック・閾値、回路上の主電源導通確認は残件。実機投入許可は未設定。T-F303-01〜06の完了条件は維持する。
