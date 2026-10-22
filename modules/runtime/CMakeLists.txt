add_library(dust_realtime
  realtime_loop.cpp
)
target_link_libraries(dust_realtime PUBLIC dust_headers Threads::Threads)
target_compile_options(dust_realtime PRIVATE -Wall -Wextra -Wpedantic)
