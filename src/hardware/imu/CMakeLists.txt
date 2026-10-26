add_library(dust_imu
  imu_config.cpp
  hi91_protocol.cpp
  onboard_imu.cpp
)
target_link_libraries(dust_imu PUBLIC dust_headers Threads::Threads PRIVATE yaml-cpp)
target_compile_options(dust_imu PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust_control::imu ALIAS dust_imu)
