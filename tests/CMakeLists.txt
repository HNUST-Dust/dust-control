add_executable(chassis_contracts chassis/contracts.cpp)
target_link_libraries(chassis_contracts PRIVATE dust_usb dust_chassis)
add_test(NAME chassis.contracts COMMAND chassis_contracts)
add_test(NAME chassis.disabled_smoke COMMAND chassis_controller --simulate --no-rt --seconds 0.1)
add_library(production_usb_mock SHARED chassis/usb_mock.cpp)
target_link_libraries(production_usb_mock PRIVATE dust_imu dust_chassis Threads::Threads PkgConfig::LIBUSB)
add_test(NAME usb.lifecycle COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/chassis/usb_integration.py" $<TARGET_FILE:chassis_controller> $<TARGET_FILE:production_usb_mock>)
set_tests_properties(usb.lifecycle PROPERTIES TIMEOUT 35)
add_executable(usb_reconnect_contract chassis/usb_reconnect.cpp)
target_link_libraries(usb_reconnect_contract PRIVATE dust_usb)
add_test(NAME usb.reconnect COMMAND ${CMAKE_COMMAND} -E env
  LD_PRELOAD=$<TARGET_FILE:production_usb_mock> WBR_MOCK_STALE_SESSION=1 WBR_MOCK_REQUIRE_PREPOST=1
  $<TARGET_FILE:usb_reconnect_contract>)
set_tests_properties(usb.reconnect PROPERTIES TIMEOUT 10)
add_executable(telemetry_contracts chassis/telemetry_contracts.cpp)
target_link_libraries(telemetry_contracts PRIVATE dust_telemetry)
add_test(NAME telemetry.queue_and_serialization COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/chassis/telemetry_udp.py" $<TARGET_FILE:telemetry_contracts> $<TARGET_FILE:chassis_controller>)
set_tests_properties(telemetry.queue_and_serialization PROPERTIES TIMEOUT 10)
add_executable(chassis_balance_fixture chassis/balance_fixture.cpp)
target_link_libraries(chassis_balance_fixture PRIVATE dust_chassis)
set(DUST_REFERENCE_CHASSIS_DIR "" CACHE PATH "Optional original MCU chassis source directory")
if(DUST_REFERENCE_CHASSIS_DIR)
  add_executable(chassis_reference_balance chassis/balance_fixture.cpp
    ${DUST_REFERENCE_CHASSIS_DIR}/balance_controller.cpp
    ${DUST_REFERENCE_CHASSIS_DIR}/body_motion_estimator.cpp
    ${DUST_REFERENCE_CHASSIS_DIR}/leg_vmc.cpp
    ${DUST_REFERENCE_CHASSIS_DIR}/leg_kinematics.cc
    ${DUST_REFERENCE_CHASSIS_DIR}/unified_lqr_schedule.cc)
  target_include_directories(chassis_reference_balance PRIVATE ${DUST_REFERENCE_CHASSIS_DIR} ${DUST_GENERATED_DIR})
  target_link_libraries(chassis_reference_balance PRIVATE dust_headers Eigen3::Eigen)
  target_compile_definitions(chassis_reference_balance PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
  add_dependencies(chassis_reference_balance dust_chassis)
  add_test(NAME chassis.original_balance_parity COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/chassis/compare_reference.py"
    $<TARGET_FILE:chassis_reference_balance> $<TARGET_FILE:chassis_balance_fixture>)
endif()
if(DUST_ENABLE_FOXGLOVE)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import websockets.asyncio.client"
    RESULT_VARIABLE websocket_client_missing OUTPUT_QUIET ERROR_QUIET)
  if(NOT websocket_client_missing)
    add_test(NAME telemetry.native_websocket COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/chassis/foxglove_native_integration.py"
      --controller $<TARGET_FILE:chassis_controller> --fixture $<TARGET_FILE:telemetry_contracts>)
    set_tests_properties(telemetry.native_websocket PROPERTIES TIMEOUT 20)
  endif()
endif()
if(DUST_BUILD_MPC)
  foreach(name leg_mpc_test leg_mpc_qp_test)
    add_executable(${name} mpc/${name}.cpp)
    target_link_libraries(${name} PRIVATE dust_mpc)
    target_compile_options(${name} PRIVATE -UNDEBUG)
    add_test(NAME mpc.${name} COMMAND ${name})
  endforeach()
  if(DUST_BUILD_TOOLS)
    add_executable(leg_mpc_benchmark mpc/leg_mpc_benchmark.cpp)
    target_link_libraries(leg_mpc_benchmark PRIVATE dust_mpc)
  endif()
endif()
# Shared realtime statistics remain covered independently of any transport.
add_executable(realtime_statistics_test runtime/statistics_test.cpp)
target_link_libraries(realtime_statistics_test PRIVATE dust_realtime)
target_compile_options(realtime_statistics_test PRIVATE -UNDEBUG)
add_test(NAME runtime.statistics COMMAND realtime_statistics_test)

# Standalone MPC development programs belong to the test/development tree.
if(DUST_BUILD_MPC AND DUST_BUILD_TOOLS)
  add_executable(leg_mpc_demo mpc/mpc_demo/main.cpp)
  target_link_libraries(leg_mpc_demo PRIVATE dust_mpc)
endif()

add_executable(runtime_config_contracts runtime/config_contracts.cpp)
target_link_libraries(runtime_config_contracts PRIVATE dust_startup)
add_test(NAME configuration.startup_loading COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/runtime/config_loading.py"
  $<TARGET_FILE:runtime_config_contracts> $<TARGET_FILE:chassis_controller>
  "${PROJECT_SOURCE_DIR}/config")
set_tests_properties(configuration.startup_loading PROPERTIES TIMEOUT 35)

add_executable(adapter_protocol_contracts protocols/adapter_protocol.cpp)
target_link_libraries(adapter_protocol_contracts PRIVATE dust_headers)
add_test(NAME usb.generic_protocol_contracts COMMAND adapter_protocol_contracts)

add_executable(uart_channel_contracts protocols/uart_channels.cpp)
target_link_libraries(uart_channel_contracts PRIVATE dust_usb)
target_compile_options(uart_channel_contracts PRIVATE -UNDEBUG)
add_test(NAME usb.uart_channels COMMAND uart_channel_contracts)

add_executable(uart_roundtrip_contracts protocols/uart_roundtrip.cpp)
target_link_libraries(uart_roundtrip_contracts PRIVATE dust_usb)
target_compile_options(uart_roundtrip_contracts PRIVATE -UNDEBUG)
add_test(NAME usb.uart_roundtrip COMMAND ${CMAKE_COMMAND} -E env
  LD_PRELOAD=$<TARGET_FILE:production_usb_mock> WBR_MOCK_ALL_UARTS=1
  $<TARGET_FILE:uart_roundtrip_contracts>)
set_tests_properties(usb.uart_roundtrip PROPERTIES TIMEOUT 10)

add_executable(ros_command_contracts runtime/ros_command_contracts.cpp)
target_link_libraries(ros_command_contracts PRIVATE dust_chassis)
target_compile_options(ros_command_contracts PRIVATE -UNDEBUG)
add_test(NAME runtime.command_arbitration COMMAND ros_command_contracts)

add_executable(motor_device_contracts protocols/motor_devices.cpp)
target_link_libraries(motor_device_contracts PRIVATE dust_motors)
target_compile_options(motor_device_contracts PRIVATE -UNDEBUG)
add_test(NAME devices.motor_protocol_boundary COMMAND motor_device_contracts)
add_executable(output_target_contracts chassis/output_targets.cpp)
target_link_libraries(output_target_contracts PRIVATE dust_chassis dust_motors)
target_compile_options(output_target_contracts PRIVATE -UNDEBUG)
add_test(NAME chassis.output_targets COMMAND output_target_contracts)
add_executable(independent_decoder_contracts protocols/independent_decoders.cpp)
target_link_libraries(independent_decoder_contracts PRIVATE dust_motors dust_remote dust_imu dust_referee)
target_compile_options(independent_decoder_contracts PRIVATE -UNDEBUG)
add_test(NAME devices.independent_decoders COMMAND independent_decoder_contracts)
add_executable(decoder_snapshot_contracts protocols/decoder_snapshots.cpp)
target_link_libraries(decoder_snapshot_contracts PRIVATE dust_motors dust_remote dust_imu Threads::Threads)
target_compile_options(decoder_snapshot_contracts PRIVATE -UNDEBUG)
add_test(NAME devices.snapshot_concurrency COMMAND decoder_snapshot_contracts)
set_tests_properties(devices.snapshot_concurrency PROPERTIES TIMEOUT 20)
add_test(NAME architecture.dependencies COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/architecture_boundaries.py" "${PROJECT_SOURCE_DIR}")

add_executable(usb_transmitter_contracts protocols/usb_transmitter.cpp)
target_link_libraries(usb_transmitter_contracts PRIVATE dust_usb)
target_compile_options(usb_transmitter_contracts PRIVATE -UNDEBUG)
add_test(NAME usb.transmit_scheduling COMMAND usb_transmitter_contracts)
add_executable(usb_transmit_integration chassis/usb_transmit_integration.cpp)
target_link_libraries(usb_transmit_integration PRIVATE dust_usb)
target_compile_options(usb_transmit_integration PRIVATE -UNDEBUG)
add_test(NAME usb.transmit_multi_producer COMMAND ${CMAKE_COMMAND} -E env
  LD_PRELOAD=$<TARGET_FILE:production_usb_mock> WBR_MOCK_ALL_UARTS=1
  $<TARGET_FILE:usb_transmit_integration>)
set_tests_properties(usb.transmit_multi_producer PROPERTIES TIMEOUT 10)

add_executable(chassis_runtime_lifecycle chassis/runtime_lifecycle.cpp)
target_link_libraries(chassis_runtime_lifecycle PRIVATE dust_chassis_runtime)
target_compile_options(chassis_runtime_lifecycle PRIVATE -UNDEBUG)
add_test(NAME chassis.runtime_lifecycle COMMAND ${CMAKE_COMMAND} -E env
  LD_PRELOAD=$<TARGET_FILE:production_usb_mock>
  $<TARGET_FILE:chassis_runtime_lifecycle>)
set_tests_properties(chassis.runtime_lifecycle PROPERTIES TIMEOUT 10)
