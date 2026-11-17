/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>

#include "chassis/estimation/wheel_contact_gate.h"

namespace dust_control::chassis {
struct BodyMotionState {
    double speed = 0.0;              // m/s, body forward
    double acceleration = 0.0;       // m/s², derivative of forward speed
    double acceleration_bias = 0.0;  // m/s²
    double position = 0.0;  // m, integral of forward speed, not world X
};

struct BodyMotionParameters {
    // Per-cycle acceleration-increment driving variance, NOT continuous jerk
    // PSD.
    double acceleration_process_noise = 3000.0;
    double bias_random_walk_psd = 0.1;
    double wheel_speed_variance = 10.0;  // (m/s)², nominal combined observation
    double acceleration_variance = 3.0;  // (m/s²)²
    WheelContactParameters contact;
};

struct BodyMotionObservation {
    // Each side translated to body-center forward speed using IMU yaw.
    std::array<double, 2> center_speed_mps{};
    std::array<bool, 2> wheel_valid{};
    std::array<double, 2> support_fraction{};  // vertical force / total weight
    bool support_valid = false;
    bool yaw_valid = false;
    // Caller owns axis mapping, gravity/rotation compensation and time
    // alignment.
    double forward_acceleration_mps2 = 0.0;
    bool acceleration_valid = false;
    bool acceleration_new_sample =
        true;  // do not correct repeatedly with one IMU frame
    double dt_s = 0.001;
};

struct BodyMotionDiagnostics {
    bool valid = false;
    bool acceleration_used = false;
    bool wheel_speed_used = false;
    BodyMotionParameters parameters;
    std::array<double, 2> center_speed_mps{};
    std::array<bool, 2> wheel_valid{};
    std::array<double, 2> support_fraction{};
    bool support_valid = false;
    bool yaw_valid = false;
    bool acceleration_valid = false;
    bool acceleration_new_sample = false;
    double forward_acceleration_mps2 = 0.0;
    double dt_s = 0.0;
    // Prior AFTER optional IMU correction, BEFORE all wheel corrections.
    double prior_speed_mps = 0.0;
    double prior_speed_variance = 0.0;
    double fused_wheel_speed_mps = 0.0;
    double wheel_speed_variance = 0.0;
    std::array<double, 3> covariance_diagonal{1.0, 1.0, 1.0};
    std::array<WheelSpeedAssessment, 2> wheels{};
};

// Three-state KF [v,a,b]. Fixed-size, allocation-free, no device or mode
// access. Missing observations skip their correction, including covariance
// correction.
class BodyMotionEstimator {
   public:
    explicit BodyMotionEstimator(BodyMotionParameters parameters = {});
    void Reset();
    const BodyMotionState& Update(const BodyMotionObservation& observation);
    const BodyMotionDiagnostics& diagnostics() const {
        return diagnostics_;
    }

   private:
    const BodyMotionParameters parameters_;
    BodyMotionState state_;
    std::array<double, 9> covariance_{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    WheelContactGate contact_gate_;
    BodyMotionDiagnostics diagnostics_;
};
void ValidateBodyMotionParameters(const BodyMotionParameters& parameters);
}  // namespace dust_control::chassis
