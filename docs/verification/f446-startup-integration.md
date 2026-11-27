# F446起動処理の統合と初期化失敗の伝達

基準 master 46bf975、実装 e1e279a、試験 a70116f。BOARD-F446-001、BEHAV-001、T-F446-01/02/06。

## 実装

`balancing_app_init()` をboolに変更。明示CAN2設定済みなら既存構成を維持してtrue、既定初期化は内部結果をそのまま返す。main.cのUSER CODE内でfalseならError_Handlerへ進み、CAN・TIM7・IWDG開始を行わない。通常とBenchDebugの排他、CANロールバック、未設定IMUの観測継続、USB Hostの既定未起動、IWDG既定無効を維持。新規Appファイルは0。

`Can2ServiceLink::validConfig` に従来の設定検査を集約し、明示CAN2構築前にも使用する。設定が不成立なら既存Applicationを破棄しない。`initializeCan2Observation` は初期化の成否を直接返す。既存の通信契約、実機設定値、駆動許可、安全閾値は変えない。

## 検証

- CTest 79/79（9d4f783102f44df5）。不正設定で未初期化状態・既存周期記録を保持、応答容量超過拒否、明示CAN2設定済みの再初期化回避、IWDG起動後の再初期化拒否・更新証跡維持を既存試験に追加。
- main.cの起動順と失敗分岐はPython静的試験。実機mainの実行を再現した試験ではない。
- 実機の通電・USB列挙・CAN通信・周期実測・IWDG物理動作・非ゼロ駆動は未実施。

- CubeIDE管理cleanBuild（bf423933ac96450c）：Debug/Release/BenchDebugの各0 errors、0 warnings、モード別ELFシンボル分離合格。text/data/bssはDebug 226980/2828/13352 B、Release 79036/2828/13352 B、BenchDebug 83964/2748/14336 B。
- 通常/UBSan全ホスト回帰（39fef7e9f6584912）：終了コード0、全件合格。CubeIDE設定25/25、CAN契約19/19。
- 仕様参照531件、文書リンク/CAN表、Doxygen生成、差分空白検査合格。Doxygenには既知の日本語翻訳更新警告あり。
