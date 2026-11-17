/* SPDX-License-Identifier: Apache-2.0 */
#include "wheel_contact_gate.h"

#include <cmath>
#include <stdexcept>

namespace dust_control::chassis {
void ValidateWheelContactParameters(const WheelContactParameters& p) {
    for (double value : {p.contact_off_fraction, p.contact_on_fraction,
                         p.contact_loss_delay_s}) {
        if (!std::isfinite(value) || value <= 0.) {
            throw std::invalid_argument(
                "Contact thresholds must be finite and positive");
        }
    }
    if (p.contact_off_fraction >= p.contact_on_fraction) {
        throw std::invalid_argument(
            "Contact off threshold must be below on threshold");
    }
}
WheelContactGate::WheelContactGate(WheelContactParameters parameters)
    : parameters_(parameters) {
    ValidateWheelContactParameters(parameters_);
}
void WheelContactGate::Reset() {
    history_ = {};
}

std::array<WheelSpeedAssessment, 2> WheelContactGate::Update(
    const WheelContactInput& input) {
    std::array<WheelSpeedAssessment, 2> result{};
    if (!std::isfinite(input.dt_s) || input.dt_s <= 0. || input.dt_s > .02) {
        return result;
    }
    for (int side = 0; side < 2; ++side) {
        auto& history = history_[side];
        auto& assessment = result[side];
        assessment.contact_known =
            input.support_valid && std::isfinite(input.support_fraction[side]);
        if (!assessment.contact_known) {
            // Unknown contact supplies no rolling constraint.
            history = {};
            assessment.evidence |= kContactUnknown;
        } else if (input.support_fraction[side] >=
                   parameters_.contact_on_fraction) {
            history.contact = true;
            history.low_support_time_s = 0.;
        } else if (input.support_fraction[side] <
                   parameters_.contact_off_fraction) {
            history.low_support_time_s += input.dt_s;
            if (history.low_support_time_s >=
                parameters_.contact_loss_delay_s) {
                history.contact = false;
            }
        } else {
            history.low_support_time_s = 0.;
        }
        assessment.contact = assessment.contact_known && history.contact;
        assessment.low_support_time_s = history.low_support_time_s;
        if (!input.yaw_valid) {
            assessment.evidence |= kYawUnavailable;
        }
        const bool wheel_valid = input.wheel_valid[side] &&
                                 std::isfinite(input.center_speed_mps[side]);
        if (!wheel_valid) {
            assessment.evidence |= kWheelUnavailable;
            continue;
        }
        assessment.prediction_residual_mps =
            input.center_speed_mps[side] - input.predicted_speed_mps;
        if (!assessment.contact_known) {
            assessment.status = WheelSpeedStatus::kUnknownContact;
        } else if (!assessment.contact) {
            assessment.status = WheelSpeedStatus::kAirborne;
            assessment.evidence |= kWheelAirborne;
        } else {
            assessment.status = WheelSpeedStatus::kTrusted;
            assessment.confidence = 1.;
        }
    }
    // Keep useful residual logging without turning it into a slip detector.
    const bool pair_valid = input.yaw_valid && result[0].confidence == 1. &&
                            result[1].confidence == 1.;
    if (pair_valid) {
        const double residual =
            input.center_speed_mps[0] - input.center_speed_mps[1];
        for (auto& assessment : result) {
            assessment.pair_comparison_valid = true;
            assessment.pair_residual_mps = residual;
        }
    }
    return result;
}
}  // namespace dust_control::chassis
