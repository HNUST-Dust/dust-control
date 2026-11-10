add_executable(chassis_observation_queue chassis/observation_queue.cpp)
target_link_libraries(chassis_observation_queue PRIVATE dust_chassis Threads::Threads)
add_test(NAME chassis.observation_queue COMMAND chassis_observation_queue)
set_tests_properties(chassis.observation_queue PROPERTIES TIMEOUT 10)
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
  target_include_directories(chassis_reference_balance PRIVATE ${DUST_REFERENCE_CHASSIS_DIR})
  target_link_libraries(chassis_reference_balance PRIVATE dust_headers Eigen3::Eigen)
  target_compile_definitions(chassis_reference_balance PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
  add_dependencies(chassis_reference_balance dust_chassis)
  add_test(NAME chassis.original_balance_parity COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/chassis/compare_reference.py"
    $<TARGET_FILE:chassis_reference_balance> $<TARGET_FILE:chassis_balance_fixture>)
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


add_executable(motor_device_contracts protocols/motor_devices.cpp)
target_link_libraries(motor_device_contracts PRIVATE dust_motors)
target_compile_options(motor_device_contracts PRIVATE -UNDEBUG)
add_test(NAME devices.motor_protocol_boundary COMMAND motor_device_contracts)
add_executable(output_target_contracts chassis/output_targets.cpp)
target_link_libraries(output_target_contracts PRIVATE dust_chassis dust_motors)
target_compile_options(output_target_contracts PRIVATE -UNDEBUG)
add_test(NAME chassis.output_targets COMMAND output_target_contracts)
add_executable(independent_decoder_contracts protocols/independent_decoders.cpp)
target_link_libraries(independent_decoder_contracts PRIVATE dust_motors dust_remote dust_imu)
target_compile_options(independent_decoder_contracts PRIVATE -UNDEBUG)
add_test(NAME devices.independent_decoders COMMAND independent_decoder_contracts)
add_executable(decoder_snapshot_contracts protocols/decoder_snapshots.cpp)
target_link_libraries(decoder_snapshot_contracts PRIVATE dust_motors dust_remote dust_imu Threads::Threads)
target_compile_options(decoder_snapshot_contracts PRIVATE -UNDEBUG)
add_test(NAME devices.snapshot_concurrency COMMAND decoder_snapshot_contracts)
set_tests_properties(devices.snapshot_concurrency PROPERTIES TIMEOUT 20)
add_test(NAME architecture.dependencies COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/architecture_boundaries.py" "${PROJECT_SOURCE_DIR}")


add_test(NAME ethercat.pdo_layout COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/ethercat/pdo_layout.py"
  $<TARGET_FILE:dust_ethercat_layout> "${PROJECT_SOURCE_DIR}/config/dust_control.yaml")
add_test(NAME ethercat.schema COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/ethercat/schema.py" "${PROJECT_SOURCE_DIR}")

add_executable(ethercat_runtime_contracts ethercat/runtime.cpp)
target_link_libraries(ethercat_runtime_contracts PRIVATE dust_startup)
target_compile_definitions(ethercat_runtime_contracts PRIVATE DUST_TEST_IGH=$<BOOL:${DUST_BUILD_IGH}>)
target_compile_options(ethercat_runtime_contracts PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME ethercat.runtime COMMAND ethercat_runtime_contracts "${PROJECT_SOURCE_DIR}/config")
set_tests_properties(ethercat.runtime PROPERTIES TIMEOUT 15)

add_executable(ethercat_config_probe ethercat/config_probe.cpp)
target_link_libraries(ethercat_config_probe PRIVATE dust_startup)
add_test(NAME ethercat.configuration COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/ethercat/configuration.py"
  $<TARGET_FILE:ethercat_config_probe> "${PROJECT_SOURCE_DIR}/config")

add_executable(chassis_control_contracts chassis/control_contracts.cpp)
target_link_libraries(chassis_control_contracts PRIVATE dust_chassis dust_motors)
target_compile_options(chassis_control_contracts PRIVATE -UNDEBUG)
add_test(NAME chassis.control_contracts COMMAND chassis_control_contracts)

add_executable(ethercat_dtg1_contracts ethercat/dtg1.cpp)
target_link_libraries(ethercat_dtg1_contracts PRIVATE dust_ethercat)
target_compile_options(ethercat_dtg1_contracts PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME ethercat.dtg1_contracts COMMAND ethercat_dtg1_contracts)

add_executable(ethercat_motor_command_contracts ethercat/motor_commands.cpp)
target_link_libraries(ethercat_motor_command_contracts PRIVATE dust_chassis dust_ethercat)
target_compile_options(ethercat_motor_command_contracts PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME ethercat.motor_commands COMMAND ethercat_motor_command_contracts)

add_executable(ethercat_imu_contracts ethercat/imu.cpp)
target_link_libraries(ethercat_imu_contracts PRIVATE dust_ethercat)
target_compile_options(ethercat_imu_contracts PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME ethercat.imu COMMAND ethercat_imu_contracts)

add_executable(chassis_controller_sequence chassis/controller_sequence.cpp)
target_link_libraries(chassis_controller_sequence PRIVATE dust_chassis)
target_compile_options(chassis_controller_sequence PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME chassis.controller_sequence COMMAND chassis_controller_sequence)

add_executable(chassis_action_interfaces chassis/action_interfaces.cpp)
target_link_libraries(chassis_action_interfaces PRIVATE dust_chassis)
target_compile_options(chassis_action_interfaces PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME chassis.action_interfaces COMMAND chassis_action_interfaces)

add_executable(chassis_balance_sequence chassis/balance_sequence.cpp)
target_link_libraries(chassis_balance_sequence PRIVATE dust_chassis)
target_compile_options(chassis_balance_sequence PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME chassis.balance_sequence COMMAND chassis_balance_sequence)

add_executable(chassis_state_priorities chassis/state_priorities.cpp)
target_link_libraries(chassis_state_priorities PRIVATE dust_chassis)
target_compile_options(chassis_state_priorities PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME chassis.state_priorities COMMAND chassis_state_priorities)

add_executable(chassis_coordinate_boundaries chassis/coordinate_boundaries.cpp)
target_link_libraries(chassis_coordinate_boundaries PRIVATE dust_chassis)
target_compile_options(chassis_coordinate_boundaries PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME chassis.coordinate_boundaries COMMAND chassis_coordinate_boundaries)

# LQR14 的实时端不依赖 Python 数值库；SciPy 仅用于离线设计回归。
add_executable(chassis_lqr14 chassis/lqr14.cpp)
target_link_libraries(chassis_lqr14 PRIVATE dust_chassis)
target_compile_options(chassis_lqr14 PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
target_compile_definitions(chassis_lqr14 PRIVATE EIGEN_RUNTIME_NO_MALLOC EIGEN_DONT_VECTORIZE)
add_test(NAME chassis.lqr14 COMMAND chassis_lqr14)
set_tests_properties(chassis.lqr14 PROPERTIES TIMEOUT 15)
execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy, scipy"
  RESULT_VARIABLE DUST_LQR14_SCIPY_STATUS OUTPUT_QUIET ERROR_QUIET)
if(DUST_LQR14_SCIPY_STATUS EQUAL 0)
  add_test(NAME chassis.lqr14_design COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/chassis/lqr14_design.py")
else()
  message(STATUS "LQR14 design regression skipped: Python numpy/scipy unavailable")
endif()

if(DUST_LQR14_SCIPY_STATUS EQUAL 0)
  set(DUST_LQR14_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/lqr14_generated")
  add_custom_command(
    OUTPUT "${DUST_LQR14_GENERATED_DIR}/lqr14_development.h"
           "${DUST_LQR14_GENERATED_DIR}/lqr14_schedule.json"
    COMMAND ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/tools/derive_lqr14.py"
            --output-dir "${DUST_LQR14_GENERATED_DIR}"
    DEPENDS "${PROJECT_SOURCE_DIR}/tools/derive_lqr14.py"
            "${PROJECT_SOURCE_DIR}/tools/derive_lqr_schedule.py"
            "${PROJECT_SOURCE_DIR}/tools/design_lqr14.py"
            "${PROJECT_SOURCE_DIR}/tools/data/leg_mass_properties0820.csv"
            "${PROJECT_SOURCE_DIR}/tools/data/lqr14_development.json"
    VERBATIM)
  add_executable(chassis_lqr14_robot_schedule chassis/lqr14_robot_schedule.cpp
    "${DUST_LQR14_GENERATED_DIR}/lqr14_development.h")
  target_include_directories(chassis_lqr14_robot_schedule PRIVATE "${DUST_LQR14_GENERATED_DIR}")
  target_link_libraries(chassis_lqr14_robot_schedule PRIVATE dust_chassis)
  target_compile_options(chassis_lqr14_robot_schedule PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  target_compile_definitions(chassis_lqr14_robot_schedule PRIVATE EIGEN_RUNTIME_NO_MALLOC EIGEN_DONT_VECTORIZE)
  add_test(NAME chassis.lqr14_robot_schedule COMMAND chassis_lqr14_robot_schedule)
  add_test(NAME chassis.lqr14_robot_model COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/chassis/lqr14_robot_model.py")
  set_tests_properties(chassis.lqr14_robot_schedule PROPERTIES TIMEOUT 15)
  set_tests_properties(chassis.lqr14_robot_model PROPERTIES TIMEOUT 30)
endif()

add_executable(chassis_lqr14_integration chassis/lqr14_integration.cpp)
target_link_libraries(chassis_lqr14_integration PRIVATE dust_chassis)
target_compile_definitions(chassis_lqr14_integration PRIVATE EIGEN_RUNTIME_NO_MALLOC EIGEN_DONT_VECTORIZE)
target_compile_options(chassis_lqr14_integration PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic -Werror)
add_test(NAME chassis.lqr14_integration COMMAND chassis_lqr14_integration)
add_test(NAME chassis.lqr14_profiles COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/chassis/lqr14_profiles.py" $<TARGET_FILE:ethercat_config_probe>)
