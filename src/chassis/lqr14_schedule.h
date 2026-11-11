/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>

#include "unified_lqr14.h"

namespace dust_control::chassis {

struct Lqr14GainSample {
    double leg_length_m = 0.0;
    bool has_pitch_trim = false;
    double theta_equilibrium_rad = 0.0;
    double body_on_leg_feedforward_nm = 0.0;
    double trim_anchor_theta_deg = 0.0;
    UnifiedLqr14::Gain gain = UnifiedLqr14::Gain::Zero();
};

/** 按当前 CSV 的十二个腿长工作点调度；构造时复制数据，无悬垂引用。 */
class Lqr14GainSchedule {
   public:
    static constexpr int kSampleCount = 12;
    using Samples = std::array<Lqr14GainSample, kSampleCount>;
    struct Evaluation {
        bool valid = false;
        bool clamped = false;
        UnifiedLqr14::Design design;
        bool has_pitch_trim = false;
        double theta_equilibrium_rad = 0.0;
        double body_on_leg_feedforward_nm = 0.0;
        double trim_anchor_theta_deg = 0.0;
    };

    // 启动时拒绝无效或乱序样本。闭环稳定性由离线密集采样验证。
    explicit Lqr14GainSchedule(Samples samples);

    // 固定大小线性插值，无分配；超范围夹紧并显式报告，NaN/Inf 无效。
    // 参数是左右平均等效腿长，不是高度。仅适用于近似对称双侧支撑。
    [[nodiscard]] Evaluation Evaluate(double leg_length_m) const noexcept;

   private:
    const Samples samples_;
};

}  // namespace dust_control::chassis
