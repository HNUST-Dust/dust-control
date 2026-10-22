add_executable(chassis_contracts chassis/contracts.cpp)
target_link_libraries(chassis_contracts PRIVATE dust_usb)
add_test(NAME chassis.contracts COMMAND chassis_contracts)
add_test(NAME chassis.disabled_smoke COMMAND chassis_controller --simulate --no-rt --seconds 0.1)
add_library(production_usb_mock SHARED chassis/usb_mock.cpp)
target_link_libraries(production_usb_mock PRIVATE dust_chassis Threads::Threads PkgConfig::LIBUSB)
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
  add_executable(mpc_link_test protocols/mpc_link_test.cpp)
  target_link_libraries(mpc_link_test PRIVATE dust_mpc_link)
  target_compile_options(mpc_link_test PRIVATE -UNDEBUG)
  add_test(NAME protocols.mpc_link COMMAND mpc_link_test)
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
