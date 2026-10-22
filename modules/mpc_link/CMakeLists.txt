add_library(dust_mpc_link
  mpc_link.cpp
)
target_link_libraries(dust_mpc_link PUBLIC dust_headers)
target_compile_options(dust_mpc_link PRIVATE -Wall -Wextra -Wpedantic)
