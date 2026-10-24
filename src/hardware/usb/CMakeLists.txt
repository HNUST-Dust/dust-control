add_library(dust_usb
  uart_receive_queue.cpp
  usb_receive.cpp
  usb_transmitter.cpp
  usb_session.cpp
  usb_link.cpp
)
target_link_libraries(dust_usb PUBLIC dust_headers dust_motors dust_remote dust_imu dust_referee dust_realtime PkgConfig::LIBUSB)
target_include_directories(dust_usb PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(dust_usb PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(dust_usb PRIVATE -Werror)
add_library(dust::usb ALIAS dust_usb)
