/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

namespace dust_control::chassis {
// Codes 3,5,6 and evidence bits 4..6 are reserved for reading historical logs.
enum class WheelSpeedStatus : uint8_t {
    kUnavailable = 0,
    kAirborne = 1,
    kUnknownContact = 2,
    kRecovering = 3,
    kTrusted = 4,
    kInconsistent = 5,
    kRejected = 6
};
struct WheelContactParameters {
    double contact_off_fraction = 0.06;
    double contact_on_fraction = 0.12;
    double contact_loss_delay_s = 0.005;
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
    double confidence =
        0.0;  // binary observation availability, not a probability
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
struct WheelContactInput {
    std::array<double, 2> center_speed_mps{};
    std::array<bool, 2> wheel_valid{};
    std::array<double, 2> support_fraction{};
    bool support_valid = false;
    bool yaw_valid = false;
    double predicted_speed_mps = 0.0;
    double dt_s = 0.001;
};
// Binary contact/validity gate only. Residuals are diagnostics, never gates.
class WheelContactGate {
   public:
    explicit WheelContactGate(WheelContactParameters parameters = {});
    void Reset();
    std::array<WheelSpeedAssessment, 2> Update(const WheelContactInput& input);

   private:
    struct SideHistory {
        bool contact = false;
        double low_support_time_s = 0.0;
    };
    const WheelContactParameters parameters_;
    std::array<SideHistory, 2> history_{};
};
void ValidateWheelContactParameters(const WheelContactParameters& parameters);
}  // namespace dust_control::chassis
