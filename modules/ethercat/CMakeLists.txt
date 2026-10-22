add_library(dust_ethercat
  ethercat_mpc_safety.cpp
  ethercat_protocol.cpp
  ethercat_transport_igh.cpp
  ethercat_transport_mock.cpp
  ethercat_transport_soem.cpp
)
target_link_libraries(dust_ethercat PUBLIC dust_realtime)
target_compile_options(dust_ethercat PRIVATE -Wall -Wextra -Wpedantic)
if(DUST_ENABLE_IGH)
  find_path(IGH_INCLUDE_DIR ecrt.h PATHS /usr/local/include)
  find_library(IGH_LIBRARY ethercat PATHS /usr/local/lib)
  if(IGH_INCLUDE_DIR AND IGH_LIBRARY)
    target_compile_definitions(dust_ethercat PRIVATE WBR_HAVE_IGH=1)
    target_include_directories(dust_ethercat SYSTEM PRIVATE "${IGH_INCLUDE_DIR}")
    target_link_libraries(dust_ethercat PRIVATE "${IGH_LIBRARY}")
    message(STATUS "EtherCAT: IgH enabled")
  else()
    message(STATUS "EtherCAT: IgH unavailable; mock backend remains available")
  endif()
endif()
if(DUST_ENABLE_SOEM)
  find_path(SOEM_INCLUDE_DIR soem/ethercat.h)
  find_library(SOEM_LIBRARY NAMES soem SOEM)
  if(SOEM_INCLUDE_DIR AND SOEM_LIBRARY)
    target_compile_definitions(dust_ethercat PRIVATE WBR_HAVE_SOEM=1)
    target_include_directories(dust_ethercat SYSTEM PRIVATE "${SOEM_INCLUDE_DIR}" "${SOEM_INCLUDE_DIR}/soem")
    target_link_libraries(dust_ethercat PRIVATE "${SOEM_LIBRARY}")
    message(STATUS "EtherCAT: SOEM enabled")
  else()
    message(STATUS "EtherCAT: SOEM unavailable; mock backend remains available")
  endif()
endif()
add_library(dust::ethercat ALIAS dust_ethercat)
