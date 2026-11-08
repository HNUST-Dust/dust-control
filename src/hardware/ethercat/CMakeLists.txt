set(pdo_header "${PROJECT_BINARY_DIR}/generated/pdo_schema.hpp")
add_custom_command(OUTPUT "${pdo_header}"
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/generate_pdo_schema.py"
          --schema "${PROJECT_SOURCE_DIR}/config/pdo_schema.yaml" --output "${pdo_header}"
  DEPENDS "${PROJECT_SOURCE_DIR}/tools/generate_pdo_schema.py"
          "${PROJECT_SOURCE_DIR}/config/pdo_schema.yaml"
  VERBATIM)
add_custom_target(dust_pdo_schema DEPENDS "${pdo_header}")
add_library(dust_ethercat_pdo pdo_config.cpp "${pdo_header}")
add_dependencies(dust_ethercat_pdo dust_pdo_schema)
target_include_directories(dust_ethercat_pdo PUBLIC "${PROJECT_BINARY_DIR}/generated")
target_link_libraries(dust_ethercat_pdo PUBLIC dust_headers)
target_compile_options(dust_ethercat_pdo PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust_ethercat commands.cpp session.cpp master.cpp transport_igh.cpp)
target_link_libraries(dust_ethercat PUBLIC dust_ethercat_pdo dust_motors dust_remote dust_imu dust_realtime)
add_dependencies(dust_ethercat dust_pdo_schema)
target_compile_options(dust_ethercat PRIVATE -Wall -Wextra -Wpedantic -Werror)
if(DUST_BUILD_IGH)
  find_path(IGH_INCLUDE_DIR ecrt.h REQUIRED HINTS /opt/etherlab/include)
  find_library(IGH_LIBRARY ethercat REQUIRED HINTS /opt/etherlab/lib /opt/etherlab/lib64)
  target_include_directories(dust_ethercat PRIVATE "${IGH_INCLUDE_DIR}")
  target_link_libraries(dust_ethercat PRIVATE "${IGH_LIBRARY}")
  target_compile_definitions(dust_ethercat PRIVATE DUST_HAVE_IGH=1)
else()
  target_compile_definitions(dust_ethercat PRIVATE DUST_HAVE_IGH=0)
  message(STATUS "IgH disabled: hardware startup will fail; offline/tests only")
endif()
add_library(dust_control::ethercat ALIAS dust_ethercat)
install(TARGETS dust_ethercat_pdo dust_ethercat ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
