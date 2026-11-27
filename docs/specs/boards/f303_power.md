# STM32F303電源基板の設計仕様

**仕様ID:** BOARD-F303-001  
**基準日:** 2026-11-27 / F303設計10項目承認済み、未検証値はTODOで追跡。

**適用状態:** 動作方針は確定。電源定格・実配線・実機保護は要確認。

F303は12Sバッテリーの主電源、プリチャージ、電源状態、非常停止の処理を担当する。メイン制御F446が停止しても、電源基板側の安全機構は独立して成立させる。

車輪用C620には、主電源からDC-DCで降圧した24 Vを供給する。DC-DCの入力上限、出力電流、熱・回生電圧への対応は採用型番と実測により確認する。

プリチャージ抵抗・MOSFETの突入電流とパルス定格は回路に基づいて確認。物理非常停止はMCUソフトウェアに依存しない遮断経路を持つ。

CAN2でPOWER_STATUSを100 ms周期に報告。POWER_ON、POWER_OFF、EMERGENCY_STOPはイベント処理。具体的なCAN IDとpayloadは[電源CAN仕様](../protocols/power_can2.md)に一元化。

F446は状態ONの報告をARMの前提とし、BALANCING中に500 ms報告が途絶えた場合はFAULTとする。復旧操作の基本順はPOWER_ON→CLEAR_FAULT→ARM。MOTOR-095（再起動後のFAULT復元）は保留。

## F303実装設計（2026-11-27、ユーザー承認済み10項目）

決定履歴：[Issue #102](https://github.com/kou-pen/balancing/issues/102)。下表は要求の正本であり、実機で成立した証拠ではない。

| 決定 | 採用した設計 |
|---|---|
| F303-01 A | 電源異常・CAN ESTOPはFAULT保持。異常解消後の明示POWER_ONでのみプリチャージから再開。新規CANコマンドは設けない。 |
| F303-02 A | VBATとDRIVE_BUSの絶対電圧差が許容範囲内に一定時間連続して成立したら完了。新規有効観測を使用し、充電期限到達時はFAULT。 |
| F303-03 A | 主電源ON要求、投入成立確認、プリチャージOFF要求の順。重複導通中はPRECHARGE報告、成立期限または異常で両出力OFF。実回路が重複導通を許容することが前提。 |
| F303-04 B | ハードウェア保護と物理ESTOPを優先。ソフトウェアでも状態別の過電圧・低電圧・持続過電流・ADC無効/失効を監視。温度はTPS異常通知。積算・熱推定・詳細診断なし。 |
| F303-05 B | タイマ駆動ADC＋DMA、1 kHz主制御周期、CAN受信IRQ、100 ms POWER_STATUS、RTOSなし。 |
| F303-06 B | ISRは周期・観測・受信記録と重大異常時のOFFのみ。ON/状態遷移はメインループで一元管理し、遮断ラッチとの競合を防ぐ。 |
| F303-07 B | PowerBoardCodec・ProtocolId・ProtocolPayload・CanTypesをF446と単一実装で共用。F303の送受信管理は専用で最小限。CAN v1のID・payloadを変更しない。 |
| F303-08 B | LED_STATUSはOFF消灯、PRECHARGE 2 Hz、ON常時点灯、FAULT 5 Hz（点滅デューティ50%）。状態と時刻で更新し、遅延待機なし。 |
| F303-09 B | C++17。application、PowerController、PowerMonitor、PowerCan、BoardIOの主要5機能。PowerControllerのみ状態を所有し、Controller/MonitorはHAL非依存。Inc/Src分離、不要なクラス・RTOS・動的確保なし。 |
| F303-10 B | 正常な制御周期完了でIWDG更新（正常なFAULT保持も含む）。異常/未判別リセット後はFAULTで起動し明示POWER_ON待ち。100 msは暫定であり未確定。 |

### 電源状態と復旧

通常のバッテリー接続時は初期出力OFFから開始し、設定・観測・保護条件が揃った後に自動プリチャージする。物理非常停止はハードウェアで独立遮断し、ソフトウェアもOFF要求とする。物理停止解除時は、元の投入意図が残っている場合に限りプリチャージから自動復帰する。CAN OFFは投入意図を消去し、明示POWER_ONまたはバッテリー再接続まで保持。CAN ESTOPまたは電源異常ではFAULT保持し、物理停止解除だけで復帰しない。F446のCLEAR_FAULT/ARMは別操作。F446側MOTOR-095は引き続き保留。

| 論理状態 | プリチャージ要求 | 主電源要求 |
|---|---|---|
| OFF | OFF | OFF |
| PRECHARGE（充電） | ON | OFF |
| PRECHARGE（重複区間） | ON | ON |
| ON（投入成立確認後） | OFF | ON |
| FAULT | OFF | OFF |

ONとOFF/ESTOPが同周期ならESTOP、OFF、ONの順で扱う（複数イベントの集約はPowerCan/applicationの今後の責務）。既存FAULTを通常OFFで解除しない。無効設定・観測不足のまま電源投入しない。応答受理と実際のONは区別する。

### 発注版と回路境界

ユーザー提供 `balancing-power_Y9(1).zip` のGerber（2026-09-30 10:14生成）を発注版の基準とする。VM上の参照KiCadは `/home/dev-agent/workspace/balancing-power/v3/balancing-power-hierarchical/`、内部履歴 `419366a`（同日08:58）。部品ごとのピン・ネット集合は120部品中119一致し、J1機械固定パッドの未接続表記差を残す。J10座標・Gerberは不一致なので完全一致版とは扱わない。

| MCU信号 | ピン | 用途 |
|---|---|---|
| ADC_IMON / ADC_VBAT / ADC_DRIVE_BUS | PA0 / PA1 / PA2 | 電流・バッテリー・バス観測 |
| AUX_ADC4/5/1/2/3 | PA3〜PA7 | 拡張ADC |
| LED_STATUS | PA8 | 状態表示 |
| CAN_RX / CAN_TX | PA11 / PA12 | CAN |
| TPS_FLT_T_N / TPS_FLT_I_N | PB3 / PB4 | ハードウェア異常入力 |
| MCU_PRECHARGE_EN / MCU_MAIN_EN | PB5 / PB6 | 電源制御要求 |
| ESTOP_OK | PB7 | 物理非常停止 |
| BRAKE_ACTIVE | PB9 | ブレーキ状態 |

主電源導通を独立に確認する具体的な方法、ADC換算・有効範囲、電圧/電流閾値、観測期限、成立時間、充電・重複期限、GPIO極性、タイマ/ADC設定、IWDG実時間、リセット中の遮断性能は未確定。回路・BOM・負荷・SOA・実測により確定するまで実機投入を許可しない。充電済みのバス電圧だけを主電源導通の根拠にしない。

実装・検証作業は[todo/f303.md](../../todo/f303.md)を参照。