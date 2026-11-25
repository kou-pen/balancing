# CANランタイムの6軸観測接続

仕様：BOARD-F446-001、PROTO-RS06-001、PROTO-C620-001、CTRL-EST-001。Issue #27 バッチA。

既存CanRuntimeはMotorServiceとMotorCanLinkを所有し、CAN Routerから同じイベントを一度だけ配送する。回収後の単調時刻で6軸を評価してMotorObservationSnapshotを固定する。未検証RS06軸設定は空の観測として扱い、C620連続性設定未検証なら原報告のみ取得する。実機の値を確定しない。

CAN2 BusOffも既存BusOff故障に含める。読み取り専用motorObservation()は故障を消費しない。上位周期が最新回収番号を指定してacknowledgeMotorObservation()を呼ぶまで、回収間の故障を保持する。時刻が等しい回収も番号で識別する。未設定RS06と実際の故障を区別し、従来の運転用takeObservedFaults()は設定不正も故障として扱う。

模擬HALを用いた実ランタイム試験：4脚/2輪の個別受信時刻、同時刻2報告の累積、主機ID不一致、CAN2 BusOff/復旧、故障保持、期限、未設定既定プロファイル、両CANの送信要求なし。CTest 24/24 PASS。実機通信・クロスビルド・物理設定は未検証。

本PRではARM・MotorService::applySequence・送信候補投入・スケジューラ実行を接続しない。CAN通常モードのACK等は発生し得る。
