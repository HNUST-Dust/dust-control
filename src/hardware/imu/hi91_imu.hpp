/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "concurrency/published_snapshot.hpp"
#include "imu_sample.hpp"

namespace dust_control::devices {
// 单接收线程校验与转换，Read 可由其他线程复制最新样本。
class Hi91Imu {
   public:
    void Reset();
    // 传输层负责序号检查；无效物理量不刷新已发布样本。
    bool Consume(const Hi91Measurement& measurement, uint64_t sample_us);
    ImuSample Read() const {
        return published_.Read();
    }

   private:
    uint32_t sequence_ = 0;
    concurrency::PublishedSnapshot<ImuSample> published_;
};
}  // namespace dust_control::devices
