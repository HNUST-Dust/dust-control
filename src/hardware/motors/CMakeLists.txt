add_library(dust_motors
  motor_config.cpp
  motor_protocol.cpp
  dji_protocol.cpp
  dm_protocol.cpp
  cubemars_protocol.cpp
)
target_link_libraries(dust_motors PUBLIC dust_headers Threads::Threads PRIVATE yaml-cpp)
target_compile_options(dust_motors PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust::motors ALIAS dust_motors)
