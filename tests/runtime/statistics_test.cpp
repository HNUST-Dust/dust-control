#include <cassert>
#include <iostream>

#include "chassis/runtime/realtime_loop.hpp"

int main() {
    // 保留原 EtherCAT 测试中覆盖的共享统计行为。
    const auto summary = wbr::realtime::Summarize({1, 2, 3, 4, 100});
    assert(summary.count == 5U);
    assert(summary.minimum_ns == 1);
    assert(summary.maximum_ns == 100);
    assert(summary.p50_ns == 3);
    std::cout << "Realtime statistics passed\n";
}
