add_library(dust_protocols
  imu/hi91_protocol.cpp
  motors/cubemars_motor_protocol.cpp
  motors/dji_motor_protocol.cpp
  motors/dm_motor_protocol.cpp
  referee/referee_protocol.cpp
  remote_input/dr16_protocol.cpp
  remote_input/vt03_protocol.cpp
  remote_input/wfly_sbus_protocol.cpp
)
target_link_libraries(dust_protocols PUBLIC dust_headers)
target_compile_options(dust_protocols PRIVATE -Wall -Wextra -Wpedantic)
