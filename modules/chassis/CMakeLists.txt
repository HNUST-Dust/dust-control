add_library(dust_chassis
  balance_controller.cpp
  body_motion_estimator.cpp
  chassis_actuator.cpp
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
target_link_libraries(dust_chassis PUBLIC dust_protocols Eigen3::Eigen)
target_include_directories(dust_chassis PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${DUST_GENERATED_DIR}")
target_compile_options(dust_chassis PRIVATE -Wall -Wextra -Wpedantic)
add_dependencies(dust_chassis dust_parameters)
target_compile_definitions(dust_chassis PRIVATE EIGEN_NO_MALLOC EIGEN_DONT_VECTORIZE)
add_library(dust::chassis ALIAS dust_chassis)
