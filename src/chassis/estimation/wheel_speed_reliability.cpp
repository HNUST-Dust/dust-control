/* SPDX-License-Identifier: Apache-2.0 */
#include "wheel_speed_reliability.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dust_control::chassis {
void ValidateWheelReliabilityParameters(const WheelReliabilityParameters& p) {
    for (double value :
         {p.contact_off_fraction, p.contact_on_fraction, p.contact_loss_delay_s,
          p.recovery_time_s, p.pair_residual_limit_mps, p.innovation_floor_mps,
          p.innovation_sigma, p.rejection_ratio, p.suspect_confidence}) {
        if (!std::isfinite(value) || value <= 0.) {
            throw std::invalid_argument(
                "Wheel reliability parameters must be finite and positive");
        }
    }
    if (p.contact_off_fraction >= p.contact_on_fraction ||
        p.rejection_ratio <= 1. || p.suspect_confidence >= 1.) {
        throw std::invalid_argument("Invalid wheel reliability thresholds");
    }
}
WheelSpeedReliability::WheelSpeedReliability(
    WheelReliabilityParameters parameters)
    : parameters_(parameters) {
    ValidateWheelReliabilityParameters(parameters_);
}
void WheelSpeedReliability::Reset() {
    history_ = {};
}

std::array<WheelSpeedAssessment, 2> WheelSpeedReliability::Update(
    const WheelReliabilityInput& input) {
    std::array<WheelSpeedAssessment, 2> result{};
    if (!std::isfinite(input.dt_s) || input.dt_s <= 0. || input.dt_s > .02) {
        return result;
    }
    const auto& p = parameters_;
    const bool prediction_valid =
        input.inertial_prediction_valid &&
        std::isfinite(input.predicted_speed_mps) &&
        std::isfinite(input.predicted_speed_variance) &&
        input.predicted_speed_variance >= 0.;
    const double gate =
        prediction_valid
            ? std::max(p.innovation_floor_mps,
                       p.innovation_sigma *
                           std::sqrt(input.predicted_speed_variance))
            : p.innovation_floor_mps;

    // First establish usable contact; an airborne wheel must not discredit its
    // peer.
    std::array<bool, 2> usable{};
    std::array<bool, 2> contact_known{};
    for (int i = 0; i < 2; ++i) {
        auto& h = history_[i];
        contact_known[i] =
            input.support_valid && std::isfinite(input.support_fraction[i]);
        if (!contact_known[i]) {
            h.contact = false;
            h.low_support_time_s = 0.;
        } else if (input.support_fraction[i] >= p.contact_on_fraction) {
            h.contact = true;
            h.low_support_time_s = 0.;
        } else if (input.support_fraction[i] < p.contact_off_fraction) {
            h.low_support_time_s += input.dt_s;
            if (h.low_support_time_s >= p.contact_loss_delay_s) {
                h.contact = false;
            }
        } else {
            h.low_support_time_s = 0.;
        }
        usable[i] = input.wheel_valid[i] &&
                    std::isfinite(input.center_speed_mps[i]) &&
                    (!contact_known[i] || h.contact);
    }
    const bool compare_pair = usable[0] && usable[1] && input.yaw_valid;
    const double pair_residual =
        compare_pair ? input.center_speed_mps[0] - input.center_speed_mps[1]
                     : 0.;
    const bool pair_bad =
        compare_pair && std::abs(pair_residual) > p.pair_residual_limit_mps;

    for (int i = 0; i < 2; ++i) {
        auto& h = history_[i];
        auto& out = result[i];
        out.pair_residual_mps = pair_residual;
        out.pair_comparison_valid = compare_pair;
        out.prediction_gate_active = prediction_valid;
        out.prediction_gate_mps = gate;
        out.contact_known = contact_known[i];
        out.contact = contact_known[i] && h.contact;
        out.low_support_time_s = h.low_support_time_s;
        if (!contact_known[i]) {
            out.evidence |= kContactUnknown;
        }
        if (!input.yaw_valid) {
            out.evidence |= kYawUnavailable;
        }
        if (pair_bad) {
            out.evidence |= kPairDisagreement;
        }
        out.prediction_residual_mps =
            std::isfinite(input.center_speed_mps[i])
                ? input.center_speed_mps[i] - input.predicted_speed_mps
                : 0.;
        if (!input.wheel_valid[i] ||
            !std::isfinite(input.center_speed_mps[i])) {
            h.confidence = 0.;
            out.evidence |= kWheelUnavailable;
            continue;
        }
        if (!usable[i]) {
            h.confidence = 0.;
            out.status = WheelSpeedStatus::kAirborne;
            out.evidence |= kWheelAirborne;
            continue;
        }
        const double ratio = prediction_valid
                                 ? std::abs(out.prediction_residual_mps) / gate
                                 : 0.;
        out.prediction_ratio = ratio;
        if (ratio > 1.) {
            out.evidence |= kPredictionDisagreement;
        }
        // Pair disagreement alone cannot identify the guilty side. With
        // inertia, preserve a side consistent with prediction while downgrading
        // its peer.
        const bool peer_matches =
            prediction_valid && std::abs(input.center_speed_mps[1 - i] -
                                         input.predicted_speed_mps) <= gate;
        const bool this_side_matches = prediction_valid && ratio <= 1.;
        const bool preserve_this_side = this_side_matches && !peer_matches;
        const bool inconsistent =
            ratio > 1. || (pair_bad && !preserve_this_side);
        if (ratio > p.rejection_ratio) {
            h.confidence = 0.;
            out.status = WheelSpeedStatus::kRejected;
            out.evidence |= kHardRejection;
        } else if (inconsistent) {
            h.confidence = std::min(h.confidence, p.suspect_confidence);
            out.status = WheelSpeedStatus::kInconsistent;
        } else {
            // Unknown contact or missing yaw cannot earn full confidence.
            const double ceiling =
                contact_known[i] && input.yaw_valid ? 1. : 0.5;
            h.confidence = std::min(
                ceiling, h.confidence + input.dt_s / p.recovery_time_s);
            out.status = !contact_known[i]   ? WheelSpeedStatus::kUnknownContact
                         : h.confidence < 1. ? WheelSpeedStatus::kRecovering
                                             : WheelSpeedStatus::kTrusted;
        }
        out.confidence = h.confidence;
    }
    return result;
}
}  // namespace dust_control::chassis
