# 纯协议库不依赖 libusb、设备驱动或业务线程，可独立验证线上格式。
add_library(dust_usb_protocol protocol/usb_protocol.cpp)
target_link_libraries(dust_usb_protocol PUBLIC dust_headers)
target_compile_options(dust_usb_protocol PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust_control::usb_protocol ALIAS dust_usb_protocol)

add_library(dust_usb
  uart_receive_queue.cpp
  usb_receive.cpp
  usb_transmitter.cpp
  usb_session.cpp
  usb_link.cpp
)
target_link_libraries(dust_usb PUBLIC dust_usb_protocol dust_motors dust_remote dust_imu dust_referee dust_realtime PkgConfig::LIBUSB)
target_include_directories(dust_usb PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(dust_usb PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(dust_usb PRIVATE -Werror)
add_library(dust_control::usb ALIAS dust_usb)
