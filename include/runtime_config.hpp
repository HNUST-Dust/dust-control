/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <string>
namespace dust::config {
struct Deadlines {
  std::array<uint64_t, 6> motors{3000,3000,3000,3000,3000,3000};
  uint64_t hi91_us = 3000;
  uint64_t remote_us = 1000000;
};
const Deadlines &Freshness();
// Call once, before constructing the controller or starting any threads.
void Load(const std::string &directory);
}
