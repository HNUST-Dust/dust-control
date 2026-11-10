/* SPDX-License-Identifier: Apache-2.0 */
#include "lqr14_schedule.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dust_control::chassis {

Lqr14GainSchedule::Lqr14GainSchedule(Samples samples)
    : samples_(std::move(samples)) {
    double previous_length = 0.0;
    for (const auto& sample : samples_) {
        if (!std::isfinite(sample.leg_length_m) ||
            sample.leg_length_m <= previous_length ||
            !sample.gain.allFinite() || sample.gain.isZero()) {
            throw std::invalid_argument("invalid LQR14 gain schedule");
        }
        previous_length = sample.leg_length_m;
    }
}

Lqr14GainSchedule::Evaluation Lqr14GainSchedule::Evaluate(
    double leg_length_m) const noexcept {
    if (!std::isfinite(leg_length_m)) {
        return {};
    }
    Evaluation result;
    const double length =
        std::clamp(leg_length_m, samples_.front().leg_length_m,
                   samples_.back().leg_length_m);
    result.clamped = length != leg_length_m;
    for (int upper = 1; upper < kSampleCount; ++upper) {
        if (length <= samples_[upper].leg_length_m) {
            const auto& left = samples_[upper - 1];
            const auto& right = samples_[upper];
            const double fraction = (length - left.leg_length_m) /
                                    (right.leg_length_m - left.leg_length_m);
            result.design.gain =
                (1.0 - fraction) * left.gain + fraction * right.gain;
            result.valid = true;
            return result;
        }
    }
    return {};
}

}  // namespace dust_control::chassis
