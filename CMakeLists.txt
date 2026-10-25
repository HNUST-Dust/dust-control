cmake_minimum_required(VERSION 3.21)
project(dust_control VERSION 0.1.0 LANGUAGES CXX)
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "dust-control runtime requires Linux; build inside Orb Ubuntu or on the robot host")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Export compile_commands.json in the build directory for clangd (cmake -S . -B build).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(BUILD_TESTING "Build automated regressions (never linked to production)" OFF)
option(DUST_BUILD_MPC "Build the actively developed OSQP MPC algorithm" OFF)
option(DUST_BUILD_TOOLS "Build optional standalone commissioning/development apps" OFF)
find_package(ament_cmake QUIET)
option(DUST_BUILD_ROS2 "Build the ROS 2 node and interfaces" ${ament_cmake_FOUND})
if(DUST_BUILD_ROS2)
  enable_language(C)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME} "msg/ChassisTelemetry.msg" DEPENDENCIES std_msgs)
endif()
if(NOT DUST_BUILD_ROS2 AND NOT BUILD_TESTING)
  message(FATAL_ERROR "The production application requires ROS 2; source its setup.bash. Use BUILD_TESTING=ON for module-only builds.")
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
add_library(dust_headers INTERFACE)
target_include_directories(dust_headers INTERFACE "${PROJECT_SOURCE_DIR}/src")
find_package(yaml-cpp REQUIRED)
add_subdirectory(src/hardware)
add_subdirectory(src/chassis)
add_subdirectory(src/gimbal)
add_subdirectory(src/ui)
add_library(dust_startup src/robot.cpp src/robot_config.cpp)
target_link_libraries(dust_startup PUBLIC dust_chassis_runtime dust_gimbal dust_ui dust_usb)
target_link_libraries(dust_startup PRIVATE yaml-cpp)
target_compile_options(dust_startup PRIVATE -Wall -Wextra -Wpedantic -Werror)
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
install(DIRECTORY tools/foxglove/ DESTINATION share/dust-control/foxglove
        FILES_MATCHING PATTERN "*.json" PATTERN "*.md")

install(DIRECTORY config/ DESTINATION share/dust-control/config FILES_MATCHING PATTERN "*.yaml" PATTERN "README.md")
install(FILES docs/protocols/USB.md docs/protocols/USB_V4_WIRE.md docs/protocols/UART.md
        DESTINATION share/dust-control/docs/protocols)

if(DUST_BUILD_ROS2)
  install(TARGETS dust_startup dust_chassis_runtime dust_usb dust_chassis
                  dust_usb_protocol dust_realtime dust_motors dust_remote dust_imu dust_referee dust_gimbal dust_ui
          LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
  add_executable(dust_control_node src/main.cpp src/dust_control_node.cpp src/chassis_telemetry.cpp)
  ament_target_dependencies(dust_control_node rclcpp rcl_interfaces sensor_msgs diagnostic_msgs)
  target_link_libraries(dust_control_node dust_startup)
  rosidl_get_typesupport_target(dust_control_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(dust_control_node "${dust_control_typesupport}")
  target_compile_options(dust_control_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
  install(TARGETS dust_control_node DESTINATION lib/${PROJECT_NAME})
  if(BUILD_TESTING)
    add_executable(ros_telemetry_contracts tests/ros/chassis_telemetry.cpp src/chassis_telemetry.cpp)
    ament_target_dependencies(ros_telemetry_contracts rclcpp)
    target_link_libraries(ros_telemetry_contracts dust_chassis "${dust_control_typesupport}")
    target_compile_options(ros_telemetry_contracts PRIVATE -UNDEBUG)
    add_test(NAME ros.telemetry_contracts COMMAND ros_telemetry_contracts)
    add_test(NAME ros.node_integration COMMAND ${CMAKE_COMMAND} -E env
      "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=167" "ROS_LOG_DIR=/tmp/dust-control-ros-test-logs"
      "PYTHONPATH=${PROJECT_BINARY_DIR}/rosidl_generator_py:$ENV{PYTHONPATH}"
      "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
      ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/tests/ros/node_integration.py"
      $<TARGET_FILE:dust_control_node> $<TARGET_FILE:production_usb_mock>)
    set_tests_properties(ros.node_integration PROPERTIES TIMEOUT 40 RUN_SERIAL TRUE)
    add_executable(ros_state_benchmark_publisher EXCLUDE_FROM_ALL tests/benchmarks/ros_state_publisher.cpp)
    ament_target_dependencies(ros_state_benchmark_publisher rclcpp)
    target_link_libraries(ros_state_benchmark_publisher "${dust_control_typesupport}")
    target_compile_options(ros_state_benchmark_publisher PRIVATE -Wall -Wextra -Wpedantic -Werror)
    find_package(foxglove_bridge QUIET)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import websockets.asyncio.client"
      RESULT_VARIABLE websocket_client_missing OUTPUT_QUIET ERROR_QUIET)
    if(foxglove_bridge_FOUND AND NOT websocket_client_missing)
      get_filename_component(DUST_BRIDGE_PREFIX "${foxglove_bridge_DIR}/../../.." ABSOLUTE)
      add_test(NAME ros.foxglove_bridge COMMAND ${CMAKE_COMMAND} -E env
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=168" "ROS_LOG_DIR=/tmp/dust-control-bridge-test-logs"
        "PYTHONPATH=${PROJECT_BINARY_DIR}/rosidl_generator_py:$ENV{PYTHONPATH}"
        "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
        ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/tests/ros/foxglove_bridge.py"
        $<TARGET_FILE:dust_control_node> "${DUST_BRIDGE_PREFIX}"
        "${PROJECT_SOURCE_DIR}/launch/foxglove.launch.py")
      set_tests_properties(ros.foxglove_bridge PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE)
    endif()
  endif()
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
          FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
          FILES_MATCHING PATTERN "*.yaml")
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
