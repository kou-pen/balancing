# BNO055非同期取得・姿勢フィルタ接続

仕様：BOARD-F446-001、CTRL-EST-001。Issue #27 バッチB。根拠：Bosch BST-BNO055-DS000-18 rev1.8, §§3.6, 3.8.1, 3.8.2.1、p.56、p.92（[データシート](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)）。

I2cPortは固定18byteバッファと単一転送を所有し、HAL READYかつSR2 BUSY解除後だけHAL IT入口を呼ぶ。主ループで実測期限と終端通知を照合する。失敗・期限切れ後のバッファ再利用を禁止し、遅延通知は次の要求に使わない。HAL IT関数内部にもBUSY待機があるので、事前検査後のBUSY競合における実行時間は未保証。

Bno055はページ0・chip ID・SW revision 03.14以上を確認、CONFIG/ACCGYRO 0x05切替待ちを単調時計で管理する。単位をm/s²、rad/sとし、軸マップは既定値のみ許容。ページ1 INT_ENのACC/GYR DRDYを有効化し、ページ0 INT_STAのread-clearで両センサの新規性を確認する。加速度100 LSB/(m/s²)、ジャイロ900 LSB/(rad/s)の符号付き値を取得。前回バースト後のflushで残った通知を捨て、次の新規報告を待つ。時刻はflush要求開始の保守的な下限であり測定時刻そのものではない。ACC/GYRの同時性、読み出し中の更新やジッタは実測検証が必要で、検証済みの証拠窓設定がなければnewMeasurementProven=false。

ImuAttitudeSourceは既存C11の相補フィルタと静止バイアスを所有し、50ms未満の鮮度を元時刻で評価。静止証明がないと校正を開始しない。実機の既定値は転送期限・証拠窓・フィルタ未検証なのでI2C転送を開始しない。実機の独立静止証明は未接続。USER CODEにI2C1 IRQと主ループの入口を追加した。生成部分・.ioc・100kHz・ピンは変更していない。

ホスト試験：模擬HALでBUSY・NACK・タイムアウト・遅延完了・別ハンドル拒否、設定待機、旧SW拒否、DRDY片側・両側・flush、符号とSI換算、静止証明なし・重複校正・50ms鮮度、既定未設定を確認。CTest 25/25 PASS。実機I2C、IRQ優先度/処理時間、センサ実測間隔、静止・取付校正、ARM実クロスビルドは未検証。
