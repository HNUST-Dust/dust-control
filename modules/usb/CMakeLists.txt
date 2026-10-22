# Shared MCU/host device-profile format; no YAML parsing runs in the RT loop.
set(DUST_ADAPTER_CAN_CONFIG "${PROJECT_SOURCE_DIR}/config/can_devices.yaml" CACHE FILEPATH "Expected MCU CAN device/protection configuration")
set(DUST_CAN_PROFILE_HEADER "${DUST_GENERATED_DIR}/can_profile_generated.hpp")
add_custom_command(OUTPUT "${DUST_CAN_PROFILE_HEADER}"
  COMMAND ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/tools/generate_can_profile.py"
    --input "${DUST_ADAPTER_CAN_CONFIG}" --output "${DUST_CAN_PROFILE_HEADER}"
  DEPENDS "${PROJECT_SOURCE_DIR}/tools/generate_can_profile.py" "${DUST_ADAPTER_CAN_CONFIG}"
  VERBATIM)
add_custom_target(dust_adapter_profile DEPENDS "${DUST_CAN_PROFILE_HEADER}")
add_library(dust_usb
  sensor_decoder.cpp
  usb_link.cpp
  "${DUST_CAN_PROFILE_HEADER}"
)
add_dependencies(dust_usb dust_adapter_profile)
target_link_libraries(dust_usb PUBLIC dust_chassis dust_realtime PkgConfig::LIBUSB)
target_include_directories(dust_usb PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${DUST_GENERATED_DIR}")
target_compile_options(dust_usb PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(dust_usb PRIVATE -Werror)
add_library(dust::usb ALIAS dust_usb)
