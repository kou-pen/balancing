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
    str(F446_APP / "Src" / "balancing" / "control" / "controller" / "balancing_core.c"),
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

motor_codec_sources = [
    str(F446_APP / "Src/balancing/communication/Rs06Codec.cpp"),
    str(F446_APP / "Src/balancing/communication/C620Codec.cpp"),
]
motor_common = ["g++","-std=c++17","-Wall","-Wextra","-Werror","-pedantic",
    "-I",str(F446_APP / "Inc"), *motor_codec_sources,
    str(ROOT / "tests/test_motor_can.cpp"), "-lm"]
for label, flags in [
    ("CAN通常ビルド", []),
    ("CAN未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_motor_can_ubsan" if flags else "test_motor_can")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(motor_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# Both Application tests now link the actual C++ control cycle and C11 math.
app_source = F446_APP / "Src" / "balancing"
app_math_cpp = [str(app_source / name) for name in (
    "app/ControlCycle.cpp", "app/CycleHealthMonitor.cpp", "device/UsbHostPort.cpp", "app/PowerBoardService.cpp", "app/DualSenseService.cpp", "communication/usb/DualSenseProtocol.cpp", "app/RemoteControlService.cpp", "app/DiagnosticsService.cpp", "app/StatusIndicator.cpp", "device/StatusIndicatorPort.cpp", "control/controller/MotorCommand.cpp",
    "control/controller/BalanceControl.cpp", "control/estimator/StateEstimator.cpp",
    "control/estimator/LegKinematics.cpp", "control/estimator/WheelOdometry.cpp")]
power_codec_sources = [str(app_source / ("communication/can/" + name)) for name in
                       ("PowerBoardCodec.cpp", "ProtocolPayload.cpp")]
power_id_source = str(app_source / "communication/can/ProtocolId.cpp")
remote_codec_source = str(app_source / "communication/can/RemoteControlCodec.cpp")
app_math_cpp += power_codec_sources + [remote_codec_source,
    str(app_source / "communication/can/DiagnosticsCodec.cpp"),
    str(app_source / "app/Can2ServiceLink.cpp")]
app_math_objects = {}
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    app_math_objects[suffix] = []
    for index, source in enumerate(("control/controller/balancing_core.c",
        "control/controller/balancing_leg_pd.c", "control/controller/balancing_yaw_pi.c",
        "control/estimator/balancing_leg_kinematics.c")):
        obj = OUT / f"app_math_{index}{suffix}.o"
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(F446_APP / "Inc"), "-c", str(app_source / source), *flags,
            "-o", str(obj)], check=True, cwd=ROOT, timeout=30)
        app_math_objects[suffix].append(str(obj))

cpp_common = [
    "g++",
    "-std=c++17",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing" / "app" / "application.cpp"),
    str(app_source / "app" / "DriveRuntime.cpp"),
    str(ROOT / "tests" / "test_cpp_app.cpp"),
    "-I", str(ROOT / "tests" / "stubs_can1"),
    str(app_source / "communication/can/CanPort.cpp"),
    str(app_source / "communication/can/CanTxScheduler.cpp"),
    str(app_source / "app" / "RobotSequence.cpp"),
    *app_math_cpp, power_id_source,
]
for label, flags in [
    ("C++アプリ通常ビルド", []),
    ("C++アプリ未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_cpp_app_ubsan" if flags else "test_cpp_app")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(cpp_common + flags + app_math_objects["_ubsan" if flags else ""] + ["-lm", "-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)
# The fixture is not the actual six-bar CAD mechanism. Test only mathematical properties.
leg_common = [
    "gcc",
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing" / "control" / "estimator" / "balancing_leg_kinematics.c"),
    str(ROOT / "tests" / "test_leg_kinematics.c"),
    "-lm",
]
for label, flags in [
    ("運動学fixture通常ビルド", []),
    ("運動学fixture未定義動作検査", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_leg_kinematics_ubsan" if flags else "test_leg_kinematics")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(leg_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# USB/CAN操作元の切替、通信断→ニュートラル復旧を実際のC++入力サービスで試験。
operator_common = [
    "g++","-std=c++17","-Wall","-Wextra","-Werror","-pedantic",
    "-I",str(F446_APP / "Inc"),
    str(app_source / "app/RemoteControlService.cpp"),
    remote_codec_source, str(app_source / "communication/can/ProtocolPayload.cpp"), power_id_source,
    str(ROOT / "tests/test_operator_input.cpp"), "-lm",
]
for label, flags in [
    ("操作入力モック通常ビルド", []),
    ("操作入力モックUBSan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_operator_input_ubsan" if flags else "test_operator_input")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(operator_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# BNO055 mock: 静止ジャイロ校正・取付軸変換・相補フィルタの境界を検証。
attitude_common = [
    "gcc",
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-pedantic",
    "-I",
    str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing" / "control" / "filter" / "balancing_attitude_filter.c"),
    str(ROOT / "tests" / "test_attitude_filter.c"),
    "-lm",
]
for label, flags in [
    ("姿勢フィルタ通常ビルド", []),
    ("姿勢フィルタUBSan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_attitude_filter_ubsan" if flags else "test_attitude_filter")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(attitude_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# ARM前固定脚長目標とRoll差動をC11単体で検証。実機ゲインは使用しない。
leg_pd_common = [
    "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
    "-I", str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing" / "control" / "controller" / "balancing_leg_pd.c"),
    str(ROOT / "tests" / "test_leg_pd.c"), "-lm",
]
for label, flags in [
    ("固定脚長/ロールPD通常ビルド", []),
    ("固定脚長/ロールPD UBSan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_leg_pd_ubsan" if flags else "test_leg_pd")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(leg_pd_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# 車輪倒立LQR配分との境界も含むYaw PIの通常・UBSan回帰。
yaw_pi_common = [
    "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
    "-I", str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing" / "control" / "controller" / "balancing_core.c"),
    str(F446_APP / "Src" / "balancing" / "control" / "controller" / "balancing_yaw_pi.c"),
    str(ROOT / "tests" / "test_yaw_pi.c"), "-lm",
]
for label, flags in [
    ("Yaw PI通常ビルド", []),
    ("Yaw PI UBSan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_yaw_pi_ubsan" if flags else "test_yaw_pi")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(yaw_pi_common + flags + ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# 仮想時計でARMとFAULTの境界を検証する。実機のCANやモータは使用しない。
robot_sequence_common = [
    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
    "-I", str(F446_APP / "Inc"),
    str(F446_APP / "Src" / "balancing" / "app" / "RobotSequence.cpp"),
    str(ROOT / "tests" / "test_robot_sequence.cpp"),
]
for label, flags in [
    ("RobotSequence 通常ビルド", []),
    ("RobotSequence UBSan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / ("test_robot_sequence_ubsan" if flags else "test_robot_sequence")
    print(f"[{label}] コンパイル", flush=True)
    subprocess.run(robot_sequence_common + flags + ["-o", str(path)],
                   check=True, cwd=ROOT, timeout=30)
    print(f"[{label}] 実行", flush=True)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# C++モータcodecを共通利用する4軸ハンドシェイク。
rs06_source = F446_APP / "Src/balancing"
rs06_service_sources = [
    str(rs06_source / "app" / "RobotSequence.cpp"),
    str(rs06_source / "app" / "MotorService.cpp"),
    str(rs06_source / "device" / "Rs06Motor.cpp"),
    str(rs06_source / "device" / "C620Motor.cpp"),
    *motor_codec_sources,
]
rs06_service_common = [
    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
    "-I", str(F446_APP / "Inc"),
] + rs06_service_sources
can1_sources = [
    "-I", str(ROOT / "tests" / "stubs_can1"),
    str(rs06_source / "communication" / "can" / "CanPort.cpp"),
]
for suffix, flags in [
    ("", []),
    ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    for test, extra_sources in (("test_motor_service_rs06", []),
                                ("test_f446_can_port", can1_sources)):
        path = OUT / f"{test}{suffix}"
        print(f"[{test}{suffix}] コンパイル", flush=True)
        source = ROOT / "tests" / f"{test}.cpp"
        subprocess.run(rs06_service_common + extra_sources + flags +
                       [str(source), "-lm", "-o", str(path)],
                       check=True, cwd=ROOT, timeout=30)
        subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

can_dispatch_common = [
    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
    "-I", str(ROOT / "tests" / "stubs_can1"), "-I", str(F446_APP / "Inc"),
    str(rs06_source / "communication" / "can" / "CanPort.cpp"),
    str(rs06_source / "communication" / "can" / "CanRouter.cpp"),
    str(rs06_source / "communication" / "can" / "ProtocolId.cpp"),
    str(rs06_source / "communication" / "can" / "CanTxScheduler.cpp"),
]
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    executable=OUT / f"test_motor_bench{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(ROOT / "tests" / "stubs_can1"), "-I", str(F446_APP / "Inc")] + [
        str(rs06_source / "communication/can/CanPort.cpp"),
        str(rs06_source / "communication/can/CanRouter.cpp"),
        str(rs06_source / "communication/can/ProtocolId.cpp"),
        str(rs06_source / "communication/can/CanTxScheduler.cpp"),
        *motor_codec_sources,
        str(rs06_source / "device/Rs06Motor.cpp"),
        str(rs06_source / "device/C620Motor.cpp"),
        str(rs06_source / "bench/MotorBench.cpp"),
        str(ROOT / "tests/test_motor_bench.cpp"),
        "-lm",
    ] + flags + ["-o", str(executable)], check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(executable)], check=True, cwd=ROOT, timeout=30)

for test, extra_sources in [
    ("test_f446_can1_runtime", rs06_service_sources + app_math_cpp + [str(rs06_source / "app" / "MotorCanLink.cpp"), str(rs06_source / "device" / "f446_can1_runtime.cpp"), str(rs06_source / "communication" / "can" / "Can2FilterPlan.cpp"), str(rs06_source / "app" / "application.cpp"), str(ROOT / "tests" / "sil" / "DriveCycle.cpp"), str(ROOT / "tests" / "sil" / "DriveRuntime.cpp"), str(rs06_source / "communication" / "can" / "MotorFramePlan.cpp")]),
    ("test_f446_can_event_poll", rs06_service_sources + app_math_cpp + [str(rs06_source / "app" / "MotorCanLink.cpp"), str(rs06_source / "communication" / "can" / "MotorFramePlan.cpp"), str(rs06_source / "control" / "filter" / "ImuAttitudeSource.cpp")]),
]:
    for suffix, flags in [
        ("", []),
        ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
    ]:
        path = OUT / f"{test}{suffix}"
        source = ROOT / "tests" / f"{test}.cpp"
        link_libraries = []
        if test in ("test_f446_can_event_poll", "test_f446_can1_runtime"):
            link_libraries = ["-lm"]
        link_libraries += app_math_objects[suffix]
        if test == "test_f446_can_event_poll":
            attitude = OUT / f"integrated_attitude{suffix}.o"
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-I", str(F446_APP / "Inc"), "-c", str(app_source / "control/filter/balancing_attitude_filter.c"), *flags, "-o", str(attitude)], check=True, cwd=ROOT, timeout=30)
            link_libraries.append(str(attitude))
        print(f"[{test}{suffix}] コンパイル", flush=True)
        subprocess.run(can_dispatch_common + extra_sources + flags +
                       [str(source)] + link_libraries + ["-o", str(path)],
                       check=True, cwd=ROOT, timeout=30)
        subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)
        if test == "test_f446_can1_runtime":
            subprocess.run([str(path), "--rollback-failure"], check=True, cwd=ROOT, timeout=30)
            subprocess.run([str(path), "--motor-observation"], check=True, cwd=ROOT, timeout=30)
            subprocess.run([str(path), "--can2-services"], check=True, cwd=ROOT, timeout=30)
            subprocess.run([str(path), "--usb-host"], check=True, cwd=ROOT, timeout=30)
            subprocess.run([str(path), "--drive-sil-usb-host"], check=True, cwd=ROOT, timeout=30)
            subprocess.run([str(path), "--watchdog"], check=True, cwd=ROOT, timeout=30)
            for mode in ("--usb-input", "--drive-sil-usb", "--drive-sil", "--drive-sil-timeout", "--drive-sil-short-deadline", "--drive-sil-long-deadline", "--drive-sil-invalid",
                         "--drive-sil-stale", "--drive-sil-rs-timeout", "--drive-sil-tx-error", "--drive-sil-estop",
                         "--drive-sil-invalid-model", "--drive-sil-rx-loss", "--drive-sil-gap"):
                subprocess.run([str(path), mode], check=True, cwd=ROOT, timeout=30)


# C++17 wheel odometry: C620Codec -> MotorService -> wheel positions.
for suffix, flags in [
    ("", []),
    ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / f"test_f446_wheel_odometry{suffix}"
    source = ROOT / "tests" / "test_f446_wheel_odometry.cpp"
    wheel_source = rs06_source / "control" / "estimator" / "WheelOdometry.cpp"
    print(f"[test_f446_wheel_odometry{suffix}] コンパイル", flush=True)
    subprocess.run(rs06_service_common + [str(wheel_source)] + flags +
                   [str(source), "-lm", "-o", str(path)],
                   check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)


# C++17 StateEstimator combines the previously tested C11 IMU and leg fixture
# with all 6 independent CAN axis timestamps. Both sanitizer variants are tested.
for suffix, flags in [
    ("", []),
    ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / f"test_f446_state_estimator{suffix}"
    sources = [
        str(rs06_source / "control" / "estimator" / "StateEstimator.cpp"),
        str(rs06_source / "control" / "estimator" / "LegKinematics.cpp"),
        str(rs06_source / "control" / "estimator" / "WheelOdometry.cpp"),
        str(ROOT / "tests" / "test_f446_state_estimator.cpp"),
    ]
    # Use the same C11 math and attitude implementations already in the repo;
    # compile C11 with gcc instead of attempting to compile C as C++17.
    c_math = OUT / f"estimator_c_math{suffix}.o"
    c_attitude = OUT / f"estimator_c_attitude{suffix}.o"
    for input_file, output_file in (
        (rs06_source / "control" / "estimator" / "balancing_leg_kinematics.c", c_math),
        (rs06_source / "control" / "filter" / "balancing_attitude_filter.c", c_attitude),
    ):
        subprocess.run(
            ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
             "-I", str(F446_APP / "Inc"), "-c", str(input_file),
             *flags, "-o", str(output_file)],
            check=True, cwd=ROOT, timeout=30,
        )
    print(f"[test_f446_state_estimator{suffix}] コンパイル", flush=True)
    subprocess.run(
        rs06_service_common + sources + flags +
        [str(c_math), str(c_attitude), "-lm", "-o", str(path)],
        check=True, cwd=ROOT, timeout=30,
    )
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)


# A host-only 500Hz SIL sequence; it compiles C++17 math but has no CAN actuator.
for suffix, flags in [
    ("", []),
    ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / f"test_f446_sil_control_cycle{suffix}"
    sources = [
        str(rs06_source / "app" / "ControlCycle.cpp"),
        str(rs06_source / "control" / "controller" / "BalanceControl.cpp"),
        str(rs06_source / "control" / "controller" / "MotorCommand.cpp"),
        str(rs06_source / "control" / "estimator" / "StateEstimator.cpp"),
        str(rs06_source / "control" / "estimator" / "LegKinematics.cpp"),
        str(rs06_source / "control" / "estimator" / "WheelOdometry.cpp"),
        str(ROOT / "tests" / "test_f446_sil_control_cycle.cpp"),
    ]
    c_math = OUT / f"estimator_c_math{suffix}.o"
    c_attitude = OUT / f"estimator_c_attitude{suffix}.o"
    c_ctrl = OUT / f"sil_control_core{suffix}.o"
    c_pd = OUT / f"sil_leg_pd{suffix}.o"
    c_yaw = OUT / f"sil_yaw_pi{suffix}.o"
    for input_file, output_file in (
        (rs06_source / "control" / "controller" / "balancing_core.c", c_ctrl),
        (rs06_source / "control" / "controller" / "balancing_leg_pd.c", c_pd),
        (rs06_source / "control" / "controller" / "balancing_yaw_pi.c", c_yaw),
    ):
        subprocess.run(
            ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
             "-I", str(F446_APP / "Inc"), "-c", str(input_file), *flags,
             "-o", str(output_file)],
            check=True, cwd=ROOT, timeout=30,
        )
    print(f"[test_f446_sil_control_cycle{suffix}] コンパイル", flush=True)
    subprocess.run(
        rs06_service_common + sources + flags +
        [str(c_math), str(c_attitude), str(c_ctrl),
         str(c_pd), str(c_yaw), "-lm", "-o", str(path)],
        check=True, cwd=ROOT, timeout=30,
    )
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)


# Mathematical MotorCommand preview only (cannot emit CAN/MotorTargets).
for suffix, flags in [
    ("", []),
    ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / f"test_f446_motor_command{suffix}"
    sources = [
        str(rs06_source / "control" / "controller" / "MotorCommand.cpp"),
        str(ROOT / "tests" / "test_f446_motor_command.cpp"),
    ]
    print(f"[test_f446_motor_command{suffix}] コンパイル", flush=True)
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-I", str(F446_APP / "Inc")] + sources + flags +
                   ["-o", str(path)], check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)


# SIL-only CAN frame plan. The actual STM32 CAN send path is intentionally absent.
for suffix, flags in [
    ("", []),
    ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"]),
]:
    path = OUT / f"test_f446_motor_frame_plan{suffix}"
    sources = [
        str(rs06_source / "communication" / "can" / "MotorFramePlan.cpp"),
        str(rs06_source / "control" / "controller" / "MotorCommand.cpp"),
        str(ROOT / "tests" / "test_f446_motor_frame_plan.cpp"),
    ]
    print(f"[test_f446_motor_frame_plan{suffix}] コンパイル", flush=True)
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
         "-I", str(F446_APP / "Inc")] + sources + motor_codec_sources + flags +
        ["-lm", "-o", str(path)], check=True, cwd=ROOT, timeout=30,
    )
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# F446 asynchronous I2C and BNO055 DRDY with host register fixture.
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_imu_runtime{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"), "-I", str(ROOT / "tests" / "stubs_can1"),
        *[str(rs06_source / name) for name in ("device/I2cPort.cpp", "device/Bno055.cpp",
           "device/f446_imu_runtime.cpp", "control/filter/ImuAttitudeSource.cpp")],
        str(ROOT / "tests" / "test_f446_imu_runtime.cpp"),
        str(OUT / f"estimator_c_attitude{suffix}.o"), *flags, "-lm", "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_power_board_service{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"), str(app_source / "app/PowerBoardService.cpp"),
        str(app_source / "app/RobotSequence.cpp"), *power_codec_sources, power_id_source,
        str(ROOT / "tests/test_f446_power_board_service.cpp"),
        *flags, "-o", str(path)], check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_remote_control_service{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"), str(app_source / "app/RemoteControlService.cpp"),
        str(ROOT / "tests/test_f446_remote_control_service.cpp"), str(app_source / "app/RobotSequence.cpp"), remote_codec_source,
        str(app_source / "communication/can/ProtocolPayload.cpp"), power_id_source, *app_math_objects[suffix],
        *flags, "-lm", "-o", str(path)], check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_diagnostics_service{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"), str(app_source / "app/DiagnosticsService.cpp"),
        str(ROOT / "tests/test_f446_diagnostics_service.cpp"), *flags, "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_status_indicator{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"), str(app_source / "app/StatusIndicator.cpp"), str(app_source / "device/StatusIndicatorPort.cpp"),
        str(ROOT / "tests/test_f446_status_indicator.cpp"), *flags, "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# CAN2 v1の固定ID・予約領域とBE表現を通常・UBSanで検証。
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_protocol_codec{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"),
        str(app_source / "communication/can/ProtocolId.cpp"),
        str(app_source / "communication/can/ProtocolPayload.cpp"),
        str(ROOT / "tests/test_f446_protocol_codec.cpp"), *flags, "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# F303電源通信の固定IDとpayload。状態機械・実機出力とは接続しない。
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_power_board_codec{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"),
        *[str(app_source / ("communication/can/" + name)) for name in
          ("PowerBoardCodec.cpp", "ProtocolId.cpp", "ProtocolPayload.cpp")],
        str(ROOT / "tests/test_f446_power_board_codec.cpp"), *flags, "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# Pi操作通信の固定IDとpayload。実機駆動なし。
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_remote_control_codec{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"),
        *[str(app_source / ("communication/can/" + name)) for name in
          ("RemoteControlCodec.cpp", "ProtocolId.cpp", "ProtocolPayload.cpp")],
        str(ROOT / "tests/test_f446_remote_control_codec.cpp"), *flags, "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
    subprocess.run([str(path)], check=True, cwd=ROOT, timeout=30)

# 診断・設定読出しの固定IDとpayload。実機送信なし。
for suffix, flags in [("", []), ("_ubsan", ["-fsanitize=undefined", "-fno-sanitize-recover=all"])]:
    path = OUT / f"test_f446_diagnostics_codec{suffix}"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(F446_APP / "Inc"),
        *[str(app_source / ("communication/can/" + name)) for name in
          ("DiagnosticsCodec.cpp", "ProtocolId.cpp", "ProtocolPayload.cpp")],
        str(ROOT / "tests/test_f446_diagnostics_codec.cpp"), *flags, "-o", str(path)],
        check=True, cwd=ROOT, timeout=30)
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
print("[CAN通信契約の静的監査] 実行", flush=True)
subprocess.run(
    [sys.executable, "tools/check_can_contract.py"],
    check=True,
    cwd=ROOT,
    timeout=30,
)
subprocess.run(
    [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_can_contract.py"],
    check=True,
    cwd=ROOT,
    timeout=30,
)
subprocess.run([sys.executable, str(ROOT / "tests/test_drive_runtime_boundary.py")], check=True, cwd=ROOT, timeout=30)
print("ホスト試験：全件合格")
