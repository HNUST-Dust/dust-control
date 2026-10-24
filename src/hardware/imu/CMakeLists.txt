add_library(dust_imu
  imu_config.cpp
  hi91_decoder.cpp
  hi91_protocol.cpp
)
target_link_libraries(dust_imu PUBLIC dust_headers Threads::Threads PRIVATE yaml-cpp)
target_include_directories(dust_imu PRIVATE "${DUST_GENERATED_DIR}")
add_dependencies(dust_imu dust_parameters)
target_compile_options(dust_imu PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust::imu ALIAS dust_imu)
