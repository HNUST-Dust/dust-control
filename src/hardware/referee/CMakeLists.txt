add_library(dust_referee
  referee_decoder.cpp
  referee_protocol.cpp
)
target_link_libraries(dust_referee PUBLIC dust_headers Threads::Threads)
target_compile_options(dust_referee PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(dust::referee ALIAS dust_referee)
