> **現在の配置:** この文書は当時のSIL試験記録です。周期実行器は後続の `ControlCycle` 本体へ移りました。[移行後の検証](f446-control-cycle-core-host.md) を参照してください。

# F446 MotorCommand 6軸変換・飽和のSIL検証

**仕様ID:** CTRL-LQR-001 / PROTO-RS06-001 / PROTO-C620-001  
**対象:** Issue #9/#11/#27、PR #47後の `master`（`f6cd075`）。  
**境界:** 数学上の6軸変換プレビューのみ。実機値・CAN送信・HAL・MotorServiceへの非ゼロ指令は含まない。

## 実装の読み順

1. `MotorCommand::evaluate()`：`BalanceControlOutput` の機構関節4軸[N·m]と車輪2軸[N·m]を、RS06モータ軸トルク[N·m]とC620の要求電流[A]に換算する。`fixtureOnly=false` と偽装された数学結果は拒否する。
2. `MotorCommand::valid()`：RS06は4軸独立のモータ回転数/関節回転数、符号±1、物理上限、検証状態を構築時に確認する。C620は左右各々の**実効車輪トルク[N·m/A]**（モータKt・内部/外部減速・伝達効率を含む測定係数）、電流方向±1、物理上限、検証状態を確認する。
3. `MotorCommandPreview`：要求値・最終制限値・軸別飽和を保持する。上流の車輪倒立/Yaw制限は `upstreamWheelSaturated` に分ける。`Ready` と `Saturated` は数学処理の成功であって、実機指令可能状態を意味しない。**`fixtureOnly=true` に固定し、CANフレームとMotorTargets生成APIは提供しない。**

## 変換式と単位

RS06 の外部伝達に `motorRevsPerJointRev=N`、方向符号 `s=±1` を用いる。機構関節とモータの仕事率保存から

```
motorTorqueNm = mechanismJointTorqueNm * s / N
```

C620 は実効係数 `k = wheelTorqueNmPerAmp` を校正して、車輪トルクから電流へ

```
motorCurrentA = mechanismWheelTorqueNm * currentDirection / k
```

とする。`k` はモータ定数・全減速・実効損失をすべて含むため、この段階で別のギヤ比をもう一度掛けない。任意の実機値を推測で配置しない。

**設定上限の検査:** CAN符号化範囲はRS06 ±36 N·m、C620 ±20 A（メーカー資料の現行基準）。この範囲内で、さらに別途実機検証した**軸別の安全上限**だけを受理する。メーカー表現範囲をそのまま機体の安全許容値とは見なさない。将来FWの版差・現物の安全限界が確認されたら独立の実機検証で確定する。

**飽和方針:** 有限で検証済みの物理上限を超えた要求は軸別にclampし、元要求値・制限値・飽和フラグを返す。`Saturated` だけではFAULTを要求しない。無効なゲイン/校正、NaN/Inf、変換結果のfloat範囲外は入力全6軸を**原子的に拒否**し、いずれの部分出力もゼロの結果を返す。停止/FAULTは `RobotSequence` が判断する。

## SIL結合と故障注入

`tests/support/SilControlCycle.hpp` はホスト専用に `StateEstimator → RobotSequence → BalanceControl → MotorCommandPreview` を連結する。未検証の6軸設定ではARM受理を拒否する。BALANCING中の数学変換失敗は `RobotSequence` に同周期の `ExternalFault` を通知し、数学トルクもプレビューも返さない。正常飽和は駆動異常としてFAULTへ昇格させない。

| 試験 | 期待結果 |
|---|---|
| RS06 4軸 | 異なる回転比・方向・トルク上限。4軸それぞれの要求値と制限値が一致 |
| C620 2軸 | 車輪Nm → 電流A、左右の符号とNm/A係数、左右独立の上限を確認 |
| 正常飽和 | 3軸以上のclamp、要求値と制限値を保持し、Saturatedのみ |
| 上流飽和 | 車輪Yaw側の上流飽和を下流の最終飽和と区別 |
| 無効な設定 | 1軸でも未検証、比ゼロ、方向0、CANの表現上限超過で全6軸不採用 |
| 無効な入力 | NaN/InfまたはfixtureOnly偽装で全6軸不採用・全ゼロ |
| 数値オーバーフロー | 先行軸の換算が成功しても後続軸のfloat表現限界超過なら部分出力なし |
| SIL 100周期 | 2ms仮想周期中に数学変換が成功し、出力はfixtureOnly |
| SIL変換失敗 | 変換失敗の同一周期でFAULT/StopAll、プレビュー・古い数学トルクを破棄 |
| SIL ARM禁止 | 未検証の最終換算設定でDisarmed継続 |

人工fixture：例としてRS06比1.5、C620実効係数0.4/0.6 N·m/A、電流限界2/4 A、トルク限界1～5 N·mを利用する。**これらは実機に設定してはならない。** 数学fixtureに記載した`verified=true`はホスト受入試験の人工条件であり、現物が検証されたわけではない。

## 実行検証

```sh
python3 tools/run_host_tests.py
cmake -S . -B build/host-cmake
cmake --build build/host-cmake -j4
cmake --build build/host-cmake --target test
python3 tools/validate_specs.py
python3 tools/sync_cubeide_cpp.py
doxygen Doxyfile
```

- CMake/CTest：**21/21 PASS**（新規6軸変換試験、前PRのSIL周期試験を更新）。初期のSIL故障注入は実際にfloatオーバーフローしない入力で一度失敗したため、`denorm_min` に修正して再試験し全件PASS。
- 通常ホスト・UBSan：**PASS**（`python3 tools/run_host_tests.py`、追加の個別変換試験・SIL統合試験を含む）。
- CubeIDE Debug/Releaseの実クロスビルド・ARM ELF生成：**NOT RUN**（接続VMのCubeIDE実行ゲート制限）。`sync_cubeide_cpp.py` の設定監査PASSとは区別する。
- 実機値、トルク・電流上限、実CAN指令、CAN送信完了、実動作：**NOT VERIFIED**。

## 後続と禁止事項

今回、`Application::controlTick()`、`RobotSequence` の実運用インターロック、`MotorService` の出力防護、CAN1/2の送信キュー、物理E-STOPには一切変更を加えない。次の実機結合には、モデル・機構CAD、モータ軸の原点/方向/伝達、車輪のNm/A換算、電流/トルク・飽和、安全閾値を**実機で検証**し、非ゼロ駆動を許可する独立した審査が必要。校正不能時は数学プレビューまでに留める。 
