set(DUST_FOXGLOVE_SDK_DIR "" CACHE PATH "Extracted official Foxglove C++ SDK")
if(NOT DUST_FOXGLOVE_SDK_DIR)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    message(FATAL_ERROR "Set DUST_FOXGLOVE_SDK_DIR for this architecture or disable DUST_ENABLE_FOXGLOVE")
  endif()
  include(FetchContent)
  FetchContent_Declare(foxglove_sdk
    URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v0.27.0/foxglove-v0.27.0-cpp-x86_64-unknown-linux-gnu.zip
    URL_HASH SHA256=4790dad26adaced1a5f81530ed406bab7b60aa202a868eb78cfc04976b9ce6d7)
  FetchContent_MakeAvailable(foxglove_sdk)
  set(DUST_FOXGLOVE_SDK_DIR "${foxglove_sdk_SOURCE_DIR}")
endif()
find_package(foxglove-sdk CONFIG REQUIRED HINTS "${DUST_FOXGLOVE_SDK_DIR}")
foxglove_sdk_add_cpp_library(foxglove_cpp TYPE STATIC)
