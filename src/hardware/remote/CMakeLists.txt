add_library(dust_remote
  remote_config.cpp
  sbus_decoder.cpp
  wfly_sbus_protocol.cpp
  dr16_protocol.cpp
  vt03_protocol.cpp
)
target_link_libraries(dust_remote PUBLIC dust_headers Threads::Threads PRIVATE yaml-cpp)
target_include_directories(dust_remote PRIVATE "${DUST_GENERATED_DIR}")
add_dependencies(dust_remote dust_parameters)
target_compile_options(dust_remote PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust::remote ALIAS dust_remote)
