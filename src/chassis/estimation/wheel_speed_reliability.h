/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

namespace dust_control::chassis {
enum class WheelSpeedStatus : uint8_t {
    kUnavailable = 0,
    kAirborne = 1,
    kUnknownContact = 2,
    kRecovering = 3,
    kTrusted = 4,
    kInconsistent = 5,
    kRejected = 6
};
struct WheelReliabilityParameters {
    double contact_off_fraction = 0.06;
    double contact_on_fraction = 0.12;
    double contact_loss_delay_s = 0.005;
    double recovery_time_s = 0.10;
    double pair_residual_limit_mps = 0.4;
    double innovation_floor_mps = 0.4;
    double innovation_sigma = 3.0;
    double rejection_ratio = 3.0;
    double suspect_confidence = 0.05;
};
// Independent evidence bits, so simultaneous causes remain visible in logs.
enum WheelSpeedEvidence : uint16_t {
    kWheelUnavailable = 1u << 0,
    kContactUnknown = 1u << 1,
    kWheelAirborne = 1u << 2,
    kYawUnavailable = 1u << 3,
    kPairDisagreement = 1u << 4,
    kPredictionDisagreement = 1u << 5,
    kHardRejection = 1u << 6,
};
struct WheelSpeedAssessment {
    WheelSpeedStatus status = WheelSpeedStatus::kUnavailable;
    double confidence = 0.0;  // reliability score, not a calibrated probability
    double prediction_residual_mps = 0.0;
    double pair_residual_mps = 0.0;
    bool pair_comparison_valid = false;
    bool prediction_gate_active = false;
    bool contact_known = false;
    bool contact = false;
    double low_support_time_s = 0.0;
    double prediction_gate_mps = 0.0;
    double prediction_ratio = 0.0;
    uint16_t evidence = 0;
};
struct WheelReliabilityInput {
    std::array<double, 2> center_speed_mps{};
    std::array<bool, 2> wheel_valid{};
    std::array<double, 2> support_fraction{};
    bool support_valid = false;
    bool yaw_valid = false;
    // Enable absolute residual gating only with a usable inertial observation.
    bool inertial_prediction_valid = false;
    double predicted_speed_mps = 0.0;
    double predicted_speed_variance = 1.0;
    double dt_s = 0.001;
};
// Assesses observation consistency; does not measure physical slip ratio.
class WheelSpeedReliability {
   public:
    explicit WheelSpeedReliability(WheelReliabilityParameters parameters = {});
    void Reset();
    std::array<WheelSpeedAssessment, 2> Update(
        const WheelReliabilityInput& input);

   private:
    struct SideHistory {
        bool contact = false;
        double low_support_time_s = 0.0;
        double confidence = 0.0;
    };
    const WheelReliabilityParameters parameters_;
    std::array<SideHistory, 2> history_{};
};
void ValidateWheelReliabilityParameters(
    const WheelReliabilityParameters& parameters);
}  // namespace dust_control::chassis
