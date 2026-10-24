/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <memory>
#include <string>

namespace stress_host {
struct BenchmarkOptions {
    double seconds = 65;
    int cpu = 3;
    int priority = 80;
    bool realtime = true;
    bool self_check = false;
    bool can_bridge = false;
    bool can_stream = false;
    bool control_latency = false;
    bool right_first = false;
    unsigned feedback_grace_us = 0;
    std::string usb_transport = "auto";
    std::string json_path = "usb_native_result.json";
    std::string csv_path;
};

// 异步 USB 事件循环由唯一线程持有；无论回调顺序如何，
// 缓冲区均需等 OUT 完成且匹配的 IN 回复到达后才能复用。
class UsbBenchmark {
   public:
    explicit UsbBenchmark(const BenchmarkOptions &options);
    ~UsbBenchmark();
    bool Open();
    void Run();
    bool WriteResults();
    static void RequestStop(int signal);

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace stress_host
