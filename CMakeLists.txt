cmake_minimum_required(VERSION 3.16)
project(balancing_core LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(F446_APP firmware/f446/balancing_f446/App)

add_library(balancing_core STATIC
    ${F446_APP}/Src/balancing/control/controller/balancing_core.c
    ${F446_APP}/Src/balancing/control/controller/balancing_leg_pd.c
    ${F446_APP}/Src/balancing/control/controller/balancing_yaw_pi.c
    ${F446_APP}/Src/balancing/communication/balancing_motor_can.c
    ${F446_APP}/Src/balancing/app/balancing_motor_bench.c
    ${F446_APP}/Src/balancing/communication/balancing_tx_scheduler.c
    ${F446_APP}/Src/balancing/control/estimator/balancing_leg_kinematics.c
    ${F446_APP}/Src/balancing/control/estimator/balancing_observation.c
    ${F446_APP}/Src/balancing/app/balancing_operator_input.c
    ${F446_APP}/Src/balancing/control/filter/balancing_attitude_filter.c
)
target_include_directories(balancing_core PUBLIC
    ${F446_APP}/Inc
)
target_compile_options(balancing_core PRIVATE
    -Wall -Wextra -Wpedantic
)

enable_testing()
add_executable(balancing_core_tests tests/test_core.c)
target_link_libraries(balancing_core_tests PRIVATE balancing_core m)
add_test(NAME balancing_core_tests COMMAND balancing_core_tests)
add_executable(balancing_motor_can_tests tests/test_motor_can.c)
target_link_libraries(balancing_motor_can_tests PRIVATE balancing_core m)
target_compile_options(balancing_motor_can_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_motor_can_tests COMMAND balancing_motor_can_tests)

# HAL-independent issue #5 scheduler: 3 simulated CAN1 mailboxes + virtual time.
add_executable(balancing_tx_scheduler_tests tests/test_tx_scheduler.c)
target_link_libraries(balancing_tx_scheduler_tests PRIVATE balancing_core m)
target_compile_options(balancing_tx_scheduler_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_tx_scheduler_tests COMMAND balancing_tx_scheduler_tests)

# Yaw rate PI with conditional integration; no physical motor output (Issue #9).
add_executable(balancing_yaw_pi_tests tests/test_yaw_pi.c)
target_link_libraries(balancing_yaw_pi_tests PRIVATE balancing_core m)
target_compile_options(balancing_yaw_pi_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_yaw_pi_tests COMMAND balancing_yaw_pi_tests)

# CAD-unverified mathematical 2-input closed-chain fixture (Issue #7).
add_executable(balancing_leg_kinematics_tests tests/test_leg_kinematics.c)
target_link_libraries(balancing_leg_kinematics_tests PRIVATE balancing_core m)
target_compile_options(balancing_leg_kinematics_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_leg_kinematics_tests COMMAND balancing_leg_kinematics_tests)

# Fixed-leg PD and Roll PD support-force math; host fixture only (Issue #9).
add_executable(balancing_leg_pd_tests tests/test_leg_pd.c)
target_link_libraries(balancing_leg_pd_tests PRIVATE balancing_core m)
target_compile_options(balancing_leg_pd_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_leg_pd_tests COMMAND balancing_leg_pd_tests)

# Timestamped IMU and C620 wheel snapshots, host-only without an ISR/RTOS.
add_executable(balancing_observation_tests tests/test_observation.c)
target_link_libraries(balancing_observation_tests PRIVATE balancing_core m)
target_compile_options(balancing_observation_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_observation_tests COMMAND balancing_observation_tests)

# Explicitly decoded USB/CAN operator targets; host fixtures only, no HID/CAN IDs.
add_executable(balancing_operator_input_tests tests/test_operator_input.c)
target_link_libraries(balancing_operator_input_tests PRIVATE balancing_core m)
target_compile_options(balancing_operator_input_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_operator_input_tests COMMAND balancing_operator_input_tests)

# ACCGYRO static calibration and pitch/roll complementary math, no real I2C.
add_executable(balancing_attitude_filter_tests tests/test_attitude_filter.c)
target_link_libraries(balancing_attitude_filter_tests PRIVATE balancing_core m)
target_compile_options(balancing_attitude_filter_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_attitude_filter_tests COMMAND balancing_attitude_filter_tests)

# Integration across independently timestamped sensors, estimator and operator gate.
add_executable(balancing_issue10_flow_tests tests/test_issue10_flow.c)
target_link_libraries(balancing_issue10_flow_tests PRIVATE balancing_core m)
target_compile_options(balancing_issue10_flow_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_issue10_flow_tests COMMAND balancing_issue10_flow_tests)

# F446 HAL adapter is host-compiled with an explicit test-only HAL shim.
# This does not replace a real STM32CubeMX build or an MCU hardware test.
add_executable(balancing_motor_hal_port_tests
    tests/test_motor_hal_port.c
    ${F446_APP}/Src/balancing/device/balancing_f446_motor_bench_port.c
)
target_include_directories(balancing_motor_hal_port_tests PRIVATE
    tests/stubs
    ${F446_APP}/Inc
)
target_link_libraries(balancing_motor_hal_port_tests PRIVATE balancing_core m)
target_compile_options(balancing_motor_hal_port_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_motor_hal_port_tests COMMAND balancing_motor_hal_port_tests)
# C++ entry bridge is kept outside CubeMX-owned Core/Src. Host test only.
set(BALANCING_CONTROL_MATH_SOURCES
    ${F446_APP}/Src/balancing/app/ControlCycle.cpp
    ${F446_APP}/Src/balancing/control/controller/MotorCommand.cpp
    ${F446_APP}/Src/balancing/control/controller/BalanceControl.cpp
    ${F446_APP}/Src/balancing/control/estimator/StateEstimator.cpp
    ${F446_APP}/Src/balancing/control/estimator/LegKinematics.cpp
    ${F446_APP}/Src/balancing/control/estimator/WheelOdometry.cpp
)
add_executable(balancing_cpp_app_tests
    tests/test_cpp_app.cpp
    ${BALANCING_CONTROL_MATH_SOURCES}
    ${F446_APP}/Src/balancing/app/RobotSequence.cpp
    ${F446_APP}/Src/balancing/app/application.cpp
    ${F446_APP}/Src/balancing/app/PowerBoardService.cpp
    ${F446_APP}/Src/balancing/app/RemoteControlService.cpp
    ${F446_APP}/Src/balancing/app/DiagnosticsService.cpp
    ${F446_APP}/Src/balancing/app/StatusIndicator.cpp
)
target_include_directories(balancing_cpp_app_tests PRIVATE
    ${F446_APP}/Inc
)
target_link_libraries(balancing_cpp_app_tests PRIVATE balancing_core m)
target_compile_options(balancing_cpp_app_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_cpp_app_tests COMMAND balancing_cpp_app_tests)

# Host-only ARM/FAULT state sequence with clock and four-axis inputs injected.
add_executable(balancing_robot_sequence_tests
    tests/test_robot_sequence.cpp
    ${F446_APP}/Src/balancing/app/RobotSequence.cpp
)
target_include_directories(balancing_robot_sequence_tests PRIVATE
    ${F446_APP}/Inc
)
target_compile_options(balancing_robot_sequence_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_robot_sequence_tests COMMAND balancing_robot_sequence_tests)
set(BALANCING_RS06_SERVICE_SOURCES
    ${F446_APP}/Src/balancing/app/RobotSequence.cpp
    ${F446_APP}/Src/balancing/app/MotorService.cpp
    ${F446_APP}/Src/balancing/device/Rs06Motor.cpp
    ${F446_APP}/Src/balancing/device/C620Motor.cpp
    ${F446_APP}/Src/balancing/communication/Rs06Codec.cpp
)
# Four-axis RS06 handshake, bound to RobotSequence through simulated CAN events.
add_executable(balancing_motor_service_rs06_tests
    tests/test_motor_service_rs06.cpp
    ${BALANCING_RS06_SERVICE_SOURCES}
)
target_link_libraries(balancing_motor_service_rs06_tests PRIVATE balancing_core m)
target_compile_options(balancing_motor_service_rs06_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_motor_service_rs06_tests COMMAND balancing_motor_service_rs06_tests)

# F446 bxCAN register behavior and RS06 service connection with an isolated host stub.
add_executable(balancing_f446_can_port_tests
    tests/test_f446_can_port.cpp
    ${BALANCING_RS06_SERVICE_SOURCES}
    ${F446_APP}/Src/balancing/communication/can/CanPort.cpp
)
target_include_directories(balancing_f446_can_port_tests PRIVATE tests/stubs_can1)
target_link_libraries(balancing_f446_can_port_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_can_port_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_can_port_tests COMMAND balancing_f446_can_port_tests)

set(BALANCING_CAN_DISPATCH_SOURCES
    ${F446_APP}/Src/balancing/app/CanEventPoll.cpp
    ${F446_APP}/Src/balancing/communication/can/CanRouter.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolId.cpp
    ${F446_APP}/Src/balancing/communication/can/CanTxScheduler.cpp
)

# Startup, RX observation and IRQ ownership at both STM32F446 CAN buses.
add_executable(balancing_f446_can1_runtime_tests
    tests/test_f446_can1_runtime.cpp
    ${BALANCING_CONTROL_MATH_SOURCES}
    ${BALANCING_RS06_SERVICE_SOURCES}
    ${F446_APP}/Src/balancing/app/MotorCanLink.cpp
    ${F446_APP}/Src/balancing/app/application.cpp
    ${F446_APP}/Src/balancing/app/PowerBoardService.cpp
    ${F446_APP}/Src/balancing/app/RemoteControlService.cpp
    ${F446_APP}/Src/balancing/app/DiagnosticsService.cpp
    ${F446_APP}/Src/balancing/app/StatusIndicator.cpp
    ${BALANCING_CAN_DISPATCH_SOURCES}
    ${F446_APP}/Src/balancing/communication/can/CanPort.cpp
    ${F446_APP}/Src/balancing/device/f446_can1_runtime.cpp
)
target_include_directories(balancing_f446_can1_runtime_tests PRIVATE
    tests/stubs_can1
    ${F446_APP}/Inc
)
target_link_libraries(balancing_f446_can1_runtime_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_can1_runtime_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_can1_runtime_tests COMMAND balancing_f446_can1_runtime_tests)
add_test(NAME balancing_f446_can_rollback_failure_tests COMMAND balancing_f446_can1_runtime_tests --rollback-failure)
add_test(NAME balancing_f446_motor_observation_tests COMMAND balancing_f446_can1_runtime_tests --motor-observation)

# A shared main-loop poll with separate bxCAN IRQ sources.
add_executable(balancing_f446_can_event_poll_tests
    tests/test_f446_can_event_poll.cpp
    ${F446_APP}/Src/balancing/app/DiagnosticsService.cpp
    ${BALANCING_CONTROL_MATH_SOURCES}
    ${F446_APP}/Src/balancing/control/filter/ImuAttitudeSource.cpp
    ${F446_APP}/Src/balancing/communication/can/MotorFramePlan.cpp
    ${BALANCING_CAN_DISPATCH_SOURCES}
    ${BALANCING_RS06_SERVICE_SOURCES}
    ${F446_APP}/Src/balancing/app/MotorCanLink.cpp
    ${F446_APP}/Src/balancing/communication/can/CanPort.cpp
)
target_include_directories(balancing_f446_can_event_poll_tests PRIVATE
    tests/stubs_can1
    ${F446_APP}/Inc
)
target_link_libraries(balancing_f446_can_event_poll_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_can_event_poll_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_can_event_poll_tests COMMAND balancing_f446_can_event_poll_tests)

# C11 C620 decoder -> MotorService snapshot -> C++17 wheel displacement/velocity.
# Mathematical fixture only; it does not authorize the real motor outputs.
add_executable(balancing_f446_wheel_odometry_tests
    tests/test_f446_wheel_odometry.cpp
    ${F446_APP}/Src/balancing/control/estimator/WheelOdometry.cpp
    ${BALANCING_RS06_SERVICE_SOURCES}
)
target_link_libraries(balancing_f446_wheel_odometry_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_wheel_odometry_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_wheel_odometry_tests COMMAND balancing_f446_wheel_odometry_tests)

# A single 500Hz mathematical snapshot from C11 IMU filtering + C++17 six-axis data.
# Real fixture geometry and actuator permission are intentionally absent.
add_executable(balancing_f446_state_estimator_tests
    tests/test_f446_state_estimator.cpp
    ${F446_APP}/Src/balancing/control/estimator/StateEstimator.cpp
    ${F446_APP}/Src/balancing/control/estimator/LegKinematics.cpp
    ${F446_APP}/Src/balancing/control/estimator/WheelOdometry.cpp
    ${BALANCING_RS06_SERVICE_SOURCES}
)
target_link_libraries(balancing_f446_state_estimator_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_state_estimator_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_state_estimator_tests COMMAND balancing_f446_state_estimator_tests)

# Controller math and SIL state transition through 500 Hz snapshots.
# Test only: no CAN/HAL actuator output is linked into the executable.
add_executable(balancing_f446_sil_control_cycle_tests
    tests/test_f446_sil_control_cycle.cpp
    ${F446_APP}/Src/balancing/app/ControlCycle.cpp
    ${F446_APP}/Src/balancing/control/controller/MotorCommand.cpp
    ${F446_APP}/Src/balancing/control/controller/BalanceControl.cpp
    ${F446_APP}/Src/balancing/control/estimator/StateEstimator.cpp
    ${F446_APP}/Src/balancing/control/estimator/LegKinematics.cpp
    ${F446_APP}/Src/balancing/control/estimator/WheelOdometry.cpp
    ${BALANCING_RS06_SERVICE_SOURCES}
)
target_link_libraries(balancing_f446_sil_control_cycle_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_sil_control_cycle_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_sil_control_cycle_tests COMMAND balancing_f446_sil_control_cycle_tests)

# Six-axis SIL-only conversion from mechanism-side torque to RS06 Nm / C620 A.
# Hardware readiness and CAN frame construction are intentionally absent.
add_executable(balancing_f446_motor_command_tests
    tests/test_f446_motor_command.cpp
    ${F446_APP}/Src/balancing/control/controller/MotorCommand.cpp
)
target_include_directories(balancing_f446_motor_command_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_motor_command_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_motor_command_tests COMMAND balancing_f446_motor_command_tests)

# Reuse the C11 RS06/C620 codecs to build a five-frame output plan without
# connecting any of the resulting byte arrays to the F446 runtime.
add_executable(balancing_f446_motor_frame_plan_tests
    tests/test_f446_motor_frame_plan.cpp
    ${F446_APP}/Src/balancing/communication/can/MotorFramePlan.cpp
    ${F446_APP}/Src/balancing/control/controller/MotorCommand.cpp
)
target_link_libraries(balancing_f446_motor_frame_plan_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_motor_frame_plan_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_motor_frame_plan_tests COMMAND balancing_f446_motor_frame_plan_tests)

set(BALANCING_IMU_SOURCES
    ${F446_APP}/Src/balancing/device/I2cPort.cpp
    ${F446_APP}/Src/balancing/device/Bno055.cpp
    ${F446_APP}/Src/balancing/device/f446_imu_runtime.cpp
    ${F446_APP}/Src/balancing/control/filter/ImuAttitudeSource.cpp
)
add_executable(balancing_f446_imu_runtime_tests
    tests/test_f446_imu_runtime.cpp
    ${BALANCING_IMU_SOURCES})
target_include_directories(balancing_f446_imu_runtime_tests PRIVATE tests/stubs_can1)
target_link_libraries(balancing_f446_imu_runtime_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_imu_runtime_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_imu_runtime_tests COMMAND balancing_f446_imu_runtime_tests)

add_executable(balancing_f446_power_service_tests tests/test_f446_power_board_service.cpp
    ${F446_APP}/Src/balancing/app/PowerBoardService.cpp
    ${F446_APP}/Src/balancing/app/RobotSequence.cpp)
target_include_directories(balancing_f446_power_service_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_power_service_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_power_service_tests COMMAND balancing_f446_power_service_tests)

add_executable(balancing_f446_remote_service_tests tests/test_f446_remote_control_service.cpp
    ${F446_APP}/Src/balancing/app/RemoteControlService.cpp)
target_link_libraries(balancing_f446_remote_service_tests PRIVATE balancing_core m)
target_compile_options(balancing_f446_remote_service_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_remote_service_tests COMMAND balancing_f446_remote_service_tests)

add_executable(balancing_f446_diagnostics_tests tests/test_f446_diagnostics_service.cpp
    ${F446_APP}/Src/balancing/app/DiagnosticsService.cpp)
target_include_directories(balancing_f446_diagnostics_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_diagnostics_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_diagnostics_tests COMMAND balancing_f446_diagnostics_tests)

add_executable(balancing_f446_status_indicator_tests tests/test_f446_status_indicator.cpp
    ${F446_APP}/Src/balancing/app/StatusIndicator.cpp)
target_include_directories(balancing_f446_status_indicator_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_status_indicator_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_status_indicator_tests COMMAND balancing_f446_status_indicator_tests)

# CAN2 v1共通コーデックの独立ホスト試験。HAL/CAN送信経路をリンクしない。
add_executable(balancing_f446_protocol_codec_tests tests/test_f446_protocol_codec.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolId.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolPayload.cpp)
target_include_directories(balancing_f446_protocol_codec_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_protocol_codec_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_protocol_codec_tests COMMAND balancing_f446_protocol_codec_tests)

# F303通信の純粋コーデック。実機I/Oなし。
add_executable(balancing_f446_power_board_codec_tests tests/test_f446_power_board_codec.cpp
    ${F446_APP}/Src/balancing/communication/can/PowerBoardCodec.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolId.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolPayload.cpp)
target_include_directories(balancing_f446_power_board_codec_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_power_board_codec_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_power_board_codec_tests COMMAND balancing_f446_power_board_codec_tests)

# Pi操作通信の純粋コーデック。実機I/Oなし。
add_executable(balancing_f446_remote_control_codec_tests tests/test_f446_remote_control_codec.cpp
    ${F446_APP}/Src/balancing/communication/can/RemoteControlCodec.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolId.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolPayload.cpp)
target_include_directories(balancing_f446_remote_control_codec_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_remote_control_codec_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_remote_control_codec_tests COMMAND balancing_f446_remote_control_codec_tests)

# 診断・設定読出しの純粋コーデック。実機I/Oなし。
add_executable(balancing_f446_diagnostics_codec_tests tests/test_f446_diagnostics_codec.cpp
    ${F446_APP}/Src/balancing/communication/can/DiagnosticsCodec.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolId.cpp
    ${F446_APP}/Src/balancing/communication/can/ProtocolPayload.cpp)
target_include_directories(balancing_f446_diagnostics_codec_tests PRIVATE ${F446_APP}/Inc)
target_compile_options(balancing_f446_diagnostics_codec_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME balancing_f446_diagnostics_codec_tests COMMAND balancing_f446_diagnostics_codec_tests)
