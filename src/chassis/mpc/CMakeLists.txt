add_library(dust_mpc
  leg_mpc.cpp
)
target_link_libraries(dust_mpc PUBLIC dust_headers Eigen3::Eigen)
target_compile_options(dust_mpc PRIVATE -Wall -Wextra -Wpedantic)
include("${PROJECT_SOURCE_DIR}/cmake/Osqp.cmake")
target_link_libraries(dust_mpc PRIVATE osqpstatic)
add_library(dust_control::mpc ALIAS dust_mpc)
