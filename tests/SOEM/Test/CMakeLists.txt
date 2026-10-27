cmake_minimum_required(VERSION 3.28)
project(ecat_rtt_test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(SOEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SOEM" CACHE PATH
    "Path to the SOEM v2.x source tree")
if(NOT EXISTS "${SOEM_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "SOEM not found at '${SOEM_DIR}'")
endif()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${SOEM_DIR}" "${CMAKE_BINARY_DIR}/soem")

find_package(Threads REQUIRED)

add_executable(ecat_rtt_test rtt_test.cpp)
target_link_libraries(ecat_rtt_test PRIVATE soem Threads::Threads)
target_compile_options(ecat_rtt_test PRIVATE -Wall -Wextra)
