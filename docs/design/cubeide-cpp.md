# STM32CubeIDEでC++を使用する（自動コード再生成対応）

**更新日：** 2026-11-10  
**適用：** F446制御基板、`firmware/f446/balancing_f446`。  
**状態：** CubeMX管理のmain.cからC++入口・TIM7 500 Hzイベントを受ける骨組みを維持。PR #16の旧App直下5ソース構成はCubeIDE 1.19.0によるDebug/Release管理ビルドPASS。**機能別ディレクトリへ一括再配置した6ソース構成も2026-11-10にCubeIDE本体のDebug/Release管理ビルドを行い、0エラー・0警告でPASS**。実機書込み・CAN通常運転への接続は未実施。

## 設計方針

CubeMXは`main.c`や`can.c`などCソースを生成する。**これらを`.cpp`にリネームしない**。自作C11/C++コードは自動生成外の`App/Src/balancing/`以下に、公開ヘッダーは対応する`App/Inc/balancing/`以下に置く。CubeMX管理の`Core/`・`Drivers/`は変更しない。

```text
balancing/firmware/f446/balancing_f446/
├── balancing_f446.ioc
├── Core/Inc/main.h             ← CubeMXが生成
├── Core/Src/main.c             ← Cのまま：USER CODE節からC++入口へ
├── Drivers/                    ← HAL C（CubeMX生成）
└── App/                        ← 人が管理するC/C++（自動生成対象外）
    ├── Inc/
    │   ├── balancing_app_bridge.h  ← 既存main.c互換用のCブリッジ
    │   └── balancing/
    │       ├── app/balancing_motor_bench.h
    │       ├── control/controller/balancing_core.h
    │       ├── device/balancing_f446_motor_bench_port.h
    │       └── communication/
    │           ├── balancing_motor_can.h
    │           └── balancing_tx_scheduler.h
    ├── Src/balancing/
    │   ├── app/application.cpp
    │   ├── app/balancing_motor_bench.c
    │   ├── control/controller/balancing_core.c
    │   ├── device/balancing_f446_motor_bench_port.c
    │   └── communication/
    │       ├── balancing_motor_can.c
    │       └── balancing_tx_scheduler.c
    └── Examples/documentation_example.h
```

`control/estimator`、`control/filter`、`core`は将来の実装先として確保する分類であり、空ディレクトリをGitへ登録するためだけのダミーファイルは追加しない。`util/`は作成しない。

**同じファイルの`USER CODE BEGIN/END`マーカー内だけを手修正**する。C++側の`application.cpp`は再生成せず保持する。Cの公開関数は`extern "C"`で名前修飾を防ぐ。

## CubeIDE側の操作

1. `.ioc`には現状HSE 16 MHz、CAN1/2各1 Mbps、I2C1、TIM7、USB Host等の設定がありコード生成済み。変更時はピン配置と実基板の整合を確認し、CubeMXの`Generate Code`を実行する。**CAN開始/フィルタ/通知は未実装のまま。**
2. 新規プロジェクト作成時は**Targeted Language: C++**。すでにCで作成した場合はProject Explorerでプロジェクト右クリック→**Convert to C++**（画面・バージョンにより表示が異なる）。
3. `Project → Properties → C/C++ Build → Settings`でC++コンパイラを有効にし、Debug/Releaseともに**GNU++17（`-std=gnu++17`）**を設定する。現行`.cproject`には明示設定済みで、`tools/sync_cubeide_cpp.py --fix`でも再生成後に復元できる。CソースはGNU11、リンクには`arm-none-eabi-g++`を使用する。
4. IDEでプロジェクトをRefresh（F5）。`App/Src/balancing/app/application.cpp`がプロジェクト内に見えていてビルド対象であることを確認する。階層化したCソースは`App/Inc`をCコンパイラのInclude Paths（Debug/Release両方）に維持し、`balancing/...`の修飾付きヘッダー名を使用する。C++ブリッジは従来の相対includeのまま。
5. C11制御コア・CAN codec・HAL試運転アダプタとC++アプリを`App/Src/balancing/`以下へ機能別に再配置した。CubeIDEの既存`App` sourcePathがネストした6ソースを検出していることを、2026-11-10のDebug/Release管理ビルドで確認した。モータ駆動経路、CAN開始・フィルタ・割込みは未接続。
6. `main.c`はリネームしない。**以下は現在の生成済みmain.cのUSER CODEへ適用済み**。今後の再生成で残っていることを確認する。

## main.cへの最小限の追加

### `/* USER CODE BEGIN Includes */`

```c
#include "../../App/Inc/balancing_app_bridge.h"
```

### `/* USER CODE BEGIN PV */`

```c
static volatile uint8_t control_tick_pending = 0u;
static volatile uint32_t control_tick_overruns = 0u;
```

### `/* USER CODE BEGIN 2 */`

```c
balancing_app_init();
/* CAN送信・非ゼロのモータ指令は未接続のままにする。 */
/* MX_TIM7_Initで生成済みのタイマに対して実行。 */
if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
    Error_Handler();
}
```

### `/* USER CODE BEGIN 4 */`

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        if (control_tick_pending != 0u) {
            ++control_tick_overruns;
        }
        control_tick_pending = 1u; /* ISRは短く、モータ指令を送らない。 */
    }
}
```

既に同じHAL callbackが存在する場合は**重複定義しない**。既存callbackの分岐にTIM7の処理を追加する。

### 無限ループ内の`/* USER CODE BEGIN 3 */`

```c
uint8_t do_control;
uint32_t old_primask = __get_PRIMASK();
__disable_irq();
do_control = control_tick_pending;
control_tick_pending = 0u;
__set_PRIMASK(old_primask);
if (do_control != 0u) {
    balancing_app_control_tick();
}
balancing_app_poll();
```

**注意：** これはC++入口を動かすための初期接続例。周期超過・複数tick欠落の監視やCAN故障処理は**まだ未実装**であるため、この状態でモータのトルク制御を開始しない。TIM7をHAL timebaseとして使用する場合は競合を避ける。今回はSYS timebaseをSysTickとする。

## このワークスペースのソース

| ファイル | 役割 |
|---|---|
| `App/Inc/balancing_app_bridge.h` | Cの`main.c`が呼べる入口API |
| `App/Src/balancing/app/application.cpp` | 静的なC++ Applicationクラスと入口の実装 |
| `firmware/f446/balancing_f446/App/Src/balancing/communication/balancing_motor_can.c` | 既存のRS06/C620 CAN codec（C11のまま利用） |
| `firmware/f446/balancing_f446/App/Src/balancing/device/balancing_f446_motor_bench_port.c` | 既存のHAL接続（配置済み・呼出し未接続） |

現在のC++ Applicationは**非駆動の骨組み**で、初期化、500 Hz制御周期の受け口、非ブロッキングポーリングだけを持つ。モータ出力はまだ接続しない。検証済みのC11モジュールを先に不要な書換えはしない。

## 確認

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 tools/run_host_tests.py
python3 tools/validate_specs.py
```

上記ではC++17アプリがCリンケージのAPIから呼び出せることを検証する。これに加えて次の**STM32CubeIDEヘッドレスビルドを実際に実施しPASS**した（2026-11-09、CubeIDE 1.19.0）。ホスト試験とクロスビルドは別々の合否として扱う。

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
/opt/st/stm32cubeide_1.19.0/headless-build.sh \
  -data "$PWD/build/cubeide-headless-workspace" \
  -import "$PWD/firmware/f446/balancing_f446" \
  -build balancing_f446/Debug
```

ビルド結果はELF生成、0エラー・0警告、`text=15708 B`、`data=12 B`、`bss=2820 B`。`application.cpp`は`-std=gnu++17`でコンパイルされた。ビルド対象は**非駆動アプリ骨組み**であり、RS06/C620のモータCAN層・実機CAN受信/送信経路はまだリンクしていない。詳細は `docs/verification/f446-crossbuild-20261109.md`。

## 再生成時の運用

- `.ioc`の変更→`Generate Code`をそのまま実行。Cの生成ファイルとHALは更新される。**その直後に下記の検証スクリプトを実行**。
- App/Src/balancing/app/application.cpp と App/Inc/balancing_app_bridge.h は基本的に変更不要。main.cのUSER CODEに置いたブリッジだけが残る。
- `main.cpp`にリネームする方法、生成CファイルにC++クラスを直書きする方法、自動生成後スクリプトで毎回ソース名を変更する方法は採用しない。
- .iocにあるProjectManager.KeepUserCode=trueを維持する。CubeIDEの生成オプションでUSER CODE保持が有効なことを毎回確認する。
- 最初の実機書込み前に、CAN1/CAN2、USB-CとUSB-Aのピン競合、8/12 MHz HSE実物、IWDG、非常停止を照合する。
## 生成後の整合チェックと自動修復

```sh
cd /home/dev-admin/chatgpt-workspace/balancing
python3 tools/sync_cubeide_cpp.py         # 読み取り検査。エラーなら下を実行
python3 tools/sync_cubeide_cpp.py --fix   # .project/.cprojectのC++構成だけ復旧
```

このスクリプトは生成済みHALコードや .ioc を書き換えない。TIM7やI2C等のピン・クロック設定、main.cのUSER CODE部分、C++ nature、Appソースパス、C++ Flashリンクスクリプトを検証する。

現在の`main.c`はCANの受信フィルタ／CAN Start／モータ制御ポートを**まだ有効化していない**。500 HzのC++アプリ呼び出しだけが動作する段階。GPIO `EMS_OUT_N`の有効極性・USBのVBUS方式は基板実物で確認すること。

CubeIDE 1.19.0のヘッドレスF446 DebugビルドはPASS。GUIの`Project → Build Project`と実機ビルド/書込みは未実施。IDEを開いたままプロジェクト設定を変更した場合は、プロジェクトをRefresh（F5）し、必要ならいったん閉じて開き直して設定を読み込ませる。

## 2026-11-10の機能別再配置後の検証境界

PR #16のCubeIDE Debug/Release管理ビルドPASSは**旧App直下5ソース**の記録。機能別再配置後の**6ソース新構成についても2026-11-10にCubeIDE本体が生成Makefileを更新し、Debug/Releaseとも0エラー・0警告でARM ELF32へクロスリンク成功**。実機書込み・CAN実通信・モータ駆動は未実施。新しい再生成手順と対象ファイルの一覧は[機能別配置検証](../verification/f446-module-layout.md)、旧構成の実測は[PR #16の検証記録](../verification/f446-source-layout.md)を参照。
