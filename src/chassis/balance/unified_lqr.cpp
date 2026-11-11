/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis/balance/unified_lqr.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "chassis_config.h"

namespace dust_control::chassis {

UnifiedLqr::UnifiedLqr(Design design, Limits limits)
    : design_(std::move(design)), limits_(std::move(limits)) {
    if (!design_.gain.allFinite() || design_.gain.isZero() ||
        !std::isfinite(design_.sample_time_s) ||
        std::abs(design_.sample_time_s - kSampleTime) > 1e-12 ||
        !limits_.lower.allFinite() || !limits_.upper.allFinite() ||
        (limits_.lower.array() > limits_.upper.array()).any() ||
        limits_.lower[kLeftVerticalForce] < 0.0 ||
        limits_.lower[kRightVerticalForce] < 0.0) {
        throw std::invalid_argument("invalid 1 kHz LQR design or limits");
    }
}

UnifiedLqr::Result UnifiedLqr::Compute(
    const State& measured, const Reference& reference) const noexcept {
    if (!measured.allFinite() || !reference.state.allFinite() ||
        !reference.feedforward.allFinite()) {
        return {};
    }
    Result result;
    result.error = measured - reference.state;
    if (!result.error.allFinite()) {
        return {};
    }
    // 所有姿态角的误差走短弧；位置、高度及角速度不折返。
    for (int index : {kYaw, kLeftTheta, kRightTheta, kPitch, kRoll}) {
        result.error[index] =
            std::remainder(result.error[index], chassis_config::kTwoPi);
    }
    result.feedback.noalias() = -design_.gain * result.error;
    result.requested = reference.feedforward + result.feedback;
    if (!result.feedback.allFinite() || !result.requested.allFinite()) {
        return {};
    }
    result.command =
        result.requested.cwiseMax(limits_.lower).cwiseMin(limits_.upper);
    result.saturated =
        (result.command.array() != result.requested.array()).any();
    result.valid = true;
    return result;
}

}  // namespace dust_control::chassis
