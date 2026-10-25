add_subdirectory(runtime)
add_library(dust_chassis
  chassis_config.cpp
  balance_controller.cpp
  body_motion_estimator.cpp
  chassis_output_policy.cpp
  chassis_controller.cpp
  chassis_input_reader.cpp
  chassis_state_machine.cpp
  climb_stairs_controller.cpp
  flight_controller.cpp
  jump_controller.cpp
  leg_kinematics.cc
  leg_vmc.cpp
  recovery_controller.cpp
  stool_controller.cpp
  support_force_estimator.cpp
  unified_lqr_schedule.cc
)
target_link_libraries(dust_chassis PUBLIC dust_headers Eigen3::Eigen)
target_include_directories(dust_chassis PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(dust_chassis PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(dust_chassis PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
add_library(dust::chassis ALIAS dust_chassis)

if(DUST_BUILD_MPC)
  add_subdirectory(mpc)
endif()

# 业务运行依赖设备和观测；纯控制算法 dust_chassis 继续独立。
add_library(dust_chassis_runtime chassis_runtime.cpp chassis_config_loading.cpp)
target_link_libraries(dust_chassis_runtime PRIVATE yaml-cpp)
target_link_libraries(dust_chassis_runtime PUBLIC dust_chassis dust_usb)
target_compile_options(dust_chassis_runtime PRIVATE -Wall -Wextra -Wpedantic -Werror)
