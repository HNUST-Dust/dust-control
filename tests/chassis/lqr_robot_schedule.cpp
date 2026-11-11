/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "chassis/balance/lqr_schedule.h"
#include "lqr_development.h"

namespace chassis = dust_control::chassis;
using Lqr = chassis::UnifiedLqr;

int main() {
    const auto samples = chassis::LqrDevelopmentSamples();
    const chassis::LqrGainSchedule schedule(samples);
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < chassis::LqrGainSchedule::kSampleCount; ++i) {
        const auto evaluation = schedule.Evaluate(samples[i].leg_length_m);
        assert(evaluation.valid && !evaluation.clamped);
        assert((evaluation.design.gain - samples[i].gain).norm() < 1e-10);
        if (i > 0) {
            const auto midpoint = schedule.Evaluate(
                0.5 * (samples[i - 1].leg_length_m + samples[i].leg_length_m));
            assert(midpoint.valid && !midpoint.clamped);
            assert((midpoint.design.gain -
                    0.5 * (samples[i - 1].gain + samples[i].gain))
                       .norm() < 1e-9);
        }
        Lqr::Limits limits;
        limits.lower << -4.7, -4.7, -54, -54, 0, 0;
        limits.upper << 4.7, 4.7, 54, 54, 200, 200;
        const Lqr controller(evaluation.design, limits);
        Lqr::Reference reference;
        reference.state[Lqr::kHeight] = 0.058 + samples[i].leg_length_m;
        reference.feedforward[Lqr::kLeftVerticalForce] = 12.054 * 9.80665 / 2;
        reference.feedforward[Lqr::kRightVerticalForce] =
            reference.feedforward[Lqr::kLeftVerticalForce];
        auto measured = reference.state;
        auto result = controller.Compute(measured, reference);
        assert(result.valid && result.command == reference.feedforward);
        measured[Lqr::kHeight] += 0.001;
        result = controller.Compute(measured, reference);
        assert(result.valid && !result.saturated);
        assert(result.feedback[Lqr::kLeftVerticalForce] < 0);
        assert(result.feedback[Lqr::kRightVerticalForce] < 0);
        assert(std::abs(result.feedback[Lqr::kLeftVerticalForce] -
                        result.feedback[Lqr::kRightVerticalForce]) < 1e-8);
        measured = reference.state;
        measured[Lqr::kRoll] += 0.001;
        result = controller.Compute(measured, reference);
        assert(result.valid && !result.saturated);
        assert(result.feedback[Lqr::kLeftVerticalForce] < 0);
        assert(result.feedback[Lqr::kRightVerticalForce] > 0);
    }
    const auto below = schedule.Evaluate(0.15);
    const auto above = schedule.Evaluate(0.33);
    assert(below.valid && below.clamped && above.valid && above.clamped);
    assert((below.design.gain - samples.front().gain).norm() < 1e-10);
    assert((above.design.gain - samples.back().gain).norm() < 1e-10);
    assert(!schedule.Evaluate(std::numeric_limits<double>::quiet_NaN()).valid);
    assert(!schedule.Evaluate(std::numeric_limits<double>::infinity()).valid);
    Eigen::internal::set_is_malloc_allowed(true);
    auto trimmed = samples;
    for (int i = 0; i < chassis::LqrGainSchedule::kSampleCount; ++i) {
        trimmed[i].has_pitch_trim = true;
        trimmed[i].trim_anchor_theta_deg = 7.0;
        trimmed[i].theta_equilibrium_rad = .1 + .001*i;
        trimmed[i].body_on_leg_feedforward_nm = -1.0;
    }
    const chassis::LqrGainSchedule trim_schedule(trimmed);
    const auto trim_mid = trim_schedule.Evaluate(.5*(trimmed[0].leg_length_m+trimmed[1].leg_length_m));
    assert(trim_mid.has_pitch_trim && trim_mid.trim_anchor_theta_deg == 7.0);
    assert(std::abs(trim_mid.theta_equilibrium_rad-.1005) < 1e-12);
    assert(trim_mid.body_on_leg_feedforward_nm == -1.0);
    trimmed[1].trim_anchor_theta_deg = 5.0;
    bool trim_rejected = false;
    try { const chassis::LqrGainSchedule bad_trim(trimmed); }
    catch (const std::invalid_argument&) { trim_rejected = true; }
    assert(trim_rejected);
    auto invalid = samples;
    invalid[1].leg_length_m = invalid[0].leg_length_m;
    bool rejected = false;
    try {
        const chassis::LqrGainSchedule bad(invalid);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}
