#!/usr/bin/env python3
"""実機非依存の制御コアをホスト環境でビルド・試験する。"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build" / "host"
F446_APP = ROOT / "firmware" / "f446" / "balancing_f446" / "App"
OUT.mkdir(parents=True, exist_ok=True)

common = [
    "gcc",
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing_core.c"),
    str(ROOT / "tests" / "test_core.c"),
    "-lm",
]
for label, flags in [
    ("通常ビルド", []),
    ("未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_core_ubsan" if flags else "test_core")
    command = common + flags + ["-o", str(path)]
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(command, check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

motor_common = [
    "gcc",
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing_motor_can.c"),
    str(F446_APP / "Src" / "balancing_motor_bench.c"),
    str(ROOT / "tests" / "test_motor_can.c"),
    "-lm",
]
for label, flags in [
    ("CAN通常ビルド", []),
    ("CAN未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_motor_can_ubsan" if flags else "test_motor_can")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(motor_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# HALとは独立した送信スケジューラの故障注入を通常・UBSanの両方で確認する。
scheduler_common = [
    "gcc",
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing_tx_scheduler.c"),
    str(ROOT / "tests" / "test_tx_scheduler.c"),
]
for label, flags in [
    ("CANスケジューラ通常ビルド", []),
    ("CANスケジューラ未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_tx_scheduler_ubsan" if flags else "test_tx_scheduler")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(scheduler_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

hal_common = [
    "gcc",
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(ROOT / "tests" / "stubs"),
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing_motor_can.c"),
    str(F446_APP / "Src" / "balancing_motor_bench.c"),
    str(F446_APP / "Src" / "balancing_f446_motor_bench_port.c"),
    str(ROOT / "tests" / "test_motor_hal_port.c"),
    "-lm",
]
for label, flags in [
    ("HAL模擬通常ビルド", []),
    ("HAL模擬未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_motor_hal_port_ubsan" if flags else "test_motor_hal_port")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(hal_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

cpp_common = [
    "g++",
    "-std=c++17",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "application.cpp"),
    str(ROOT / "tests" / "test_cpp_app.cpp"),
]
for label, flags in [
    ("C++アプリ通常ビルド", []),
    ("C++アプリ未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_cpp_app_ubsan" if flags else "test_cpp_app")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(cpp_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)
print("[CubeIDE C++設定回帰試験] 実行", flush=True)
subprocess.run(
    [
        sys.executable,
        "-m",
        "unittest",
        "discover",
        "-s",
        "tests",
        "-p",
        "test_cubeide_cpp_metadata.py",
    ],
    check=True,
    cwd=ROOT,
    timeout=30,
)
print("ホスト試験：全件合格")
