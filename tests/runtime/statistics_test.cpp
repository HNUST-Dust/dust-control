#include <cassert>
#include <iostream>
#include "wbr/realtime_loop.hpp"

int main() {
  // Preserve the shared checks previously housed in the EtherCAT test.
  const auto summary = wbr::realtime::Summarize({1, 2, 3, 4, 100});
  assert(summary.count == 5U);
  assert(summary.minimum_ns == 1);
  assert(summary.maximum_ns == 100);
  assert(summary.p50_ns == 3);
  std::cout << "Realtime statistics passed\n";
}
