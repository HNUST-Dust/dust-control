# 阅读主线：周期输入、状态选择、动作计算、输出策略。
add_library(dust_chassis
  chassis_controller.cpp
  chassis_input_reader.cpp
  chassis_state_machine.cpp
  chassis_output_policy.cpp

  # 各动作独立拥有阶段、参考和控制状态。
  balance_controller.cpp
  balance_lqr14.cpp
  recovery_controller.cpp
  flight_controller.cpp
  jump_controller.cpp
  climb_stairs_controller.cpp

  # 共享运动学、估计和力矩算法。
  leg_kinematics.cpp
  leg_vmc.cpp
  body_motion_estimator.cpp
  support_force_estimator.cpp
  unified_lqr14.cpp
  lqr14_schedule.cpp
  chassis_config.cpp
)
target_link_libraries(dust_chassis PUBLIC dust_headers Eigen3::Eigen)
target_include_directories(dust_chassis PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(dust_chassis PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(dust_chassis PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
add_library(dust_control::chassis ALIAS dust_chassis)

if(DUST_BUILD_MPC)
  add_subdirectory(mpc)
endif()

# 业务运行依赖设备和观测；纯控制算法 dust_chassis 继续独立。
add_library(dust_chassis_runtime chassis_runtime.cpp chassis_config_loading.cpp)
target_link_libraries(dust_chassis_runtime PRIVATE yaml-cpp)
target_link_libraries(dust_chassis_runtime PUBLIC dust_chassis dust_ethercat)
target_compile_options(dust_chassis_runtime PRIVATE -Wall -Wextra -Wpedantic -Werror)
