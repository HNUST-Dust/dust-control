add_library(dust_telemetry
  telemetry.cpp
)
target_link_libraries(dust_telemetry PUBLIC dust_chassis dust_realtime)
target_include_directories(dust_telemetry PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(dust_telemetry PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(dust_telemetry PRIVATE -Werror)
if(DUST_ENABLE_FOXGLOVE)
  target_include_directories(dust_telemetry PRIVATE "${DUST_GENERATED_DIR}")
  include("${PROJECT_SOURCE_DIR}/cmake/Foxglove.cmake")
  target_link_libraries(dust_telemetry PRIVATE foxglove_cpp)
  target_compile_definitions(dust_telemetry PRIVATE WBR_ENABLE_FOXGLOVE=1)
  file(READ "${PROJECT_SOURCE_DIR}/tools/foxglove/schema.json" telemetry_schema_json)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/tools/foxglove/schema.json")
  configure_file(telemetry_schema.hpp.in "${DUST_GENERATED_DIR}/telemetry_schema.hpp" @ONLY)
endif()
add_library(dust::telemetry ALIAS dust_telemetry)
