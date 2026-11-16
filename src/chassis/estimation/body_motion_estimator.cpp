/* SPDX-License-Identifier: Apache-2.0 */
#include "body_motion_estimator.h"

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace dust_control::chassis {
namespace {
using Vector = Eigen::Matrix<double, 3, 1>;
using Matrix = Eigen::Matrix<double, 3, 3>;
using Row = Eigen::Matrix<double, 1, 3>;

void Correct(Vector& state, Matrix& covariance, const Row& observation,
             double measurement, double variance) {
    const Vector gain =
        covariance * observation.transpose() /
        ((observation * covariance * observation.transpose())(0, 0) + variance);
    state += gain * (measurement - (observation * state)(0));
    // Joseph form preserves symmetry/positive semidefiniteness under roundoff.
    const Matrix residual = Matrix::Identity() - gain * observation;
    covariance = (residual * covariance * residual.transpose() +
                  variance * gain * gain.transpose())
                     .eval();
    covariance = (0.5 * (covariance + covariance.transpose())).eval();
}
}  // namespace
void ValidateBodyMotionParameters(const BodyMotionParameters& p) {
    for (double value : {p.acceleration_process_noise, p.bias_random_walk_psd,
                         p.wheel_speed_variance, p.acceleration_variance}) {
        if (!std::isfinite(value) || value <= 0.) {
            throw std::invalid_argument(
                "Motion KF variances must be finite and positive");
        }
    }
    ValidateWheelReliabilityParameters(p.reliability);
}
BodyMotionEstimator::BodyMotionEstimator(BodyMotionParameters parameters)
    : parameters_(parameters), reliability_(parameters.reliability) {
    ValidateBodyMotionParameters(parameters_);
}
void BodyMotionEstimator::Reset() {
    state_ = {};
    covariance_ = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    reliability_.Reset();
    diagnostics_ = {};
}
const BodyMotionState& BodyMotionEstimator::Update(
    const BodyMotionObservation& input) {
    diagnostics_ = {};
    diagnostics_.parameters = parameters_;
    diagnostics_.center_speed_mps = input.center_speed_mps;
    diagnostics_.wheel_valid = input.wheel_valid;
    diagnostics_.support_fraction = input.support_fraction;
    diagnostics_.support_valid = input.support_valid;
    diagnostics_.yaw_valid = input.yaw_valid;
    diagnostics_.acceleration_valid = input.acceleration_valid;
    diagnostics_.acceleration_new_sample = input.acceleration_new_sample;
    diagnostics_.forward_acceleration_mps2 = input.forward_acceleration_mps2;
    diagnostics_.dt_s = input.dt_s;
    const double dt = input.dt_s;
    if (!std::isfinite(dt) || dt <= 0. || dt > .02) {
        return state_;
    }
    Vector state;
    state << state_.speed, state_.acceleration, state_.acceleration_bias;
    Matrix covariance = Eigen::Map<const Matrix>(covariance_.data());
    Matrix transition = Matrix::Identity();
    transition(0, 1) = dt;
    // One random acceleration increment per cycle: G=[dt²/2,dt,0].
    Vector driving;
    driving << .5 * dt * dt, dt, 0.;
    Matrix process =
        parameters_.acceleration_process_noise * driving * driving.transpose();
    process(2, 2) = dt * parameters_.bias_random_walk_psd;
    state = (transition * state).eval();
    covariance =
        (transition * covariance * transition.transpose() + process).eval();

    const bool imu_valid = input.acceleration_valid &&
                           std::isfinite(input.forward_acceleration_mps2);
    if (imu_valid && input.acceleration_new_sample) {
        Correct(state, covariance, (Row() << 0., 1., 1.).finished(),
                input.forward_acceleration_mps2,
                parameters_.acceleration_variance);
        diagnostics_.acceleration_used = true;
    }
    diagnostics_.prior_speed_mps = state[0];
    diagnostics_.prior_speed_variance = covariance(0, 0);
    // All wheel residuals use the SAME prior, before any wheel correction.
    WheelReliabilityInput reliability_input;
    reliability_input.center_speed_mps = input.center_speed_mps;
    reliability_input.wheel_valid = input.wheel_valid;
    reliability_input.support_fraction = input.support_fraction;
    reliability_input.support_valid = input.support_valid;
    reliability_input.yaw_valid = input.yaw_valid;
    reliability_input.inertial_prediction_valid = imu_valid;
    reliability_input.predicted_speed_mps = state[0];
    reliability_input.predicted_speed_variance = covariance(0, 0);
    reliability_input.dt_s = dt;
    diagnostics_.wheels = reliability_.Update(reliability_input);

    double weight_sum = 0., weighted_speed = 0.;
    for (int side = 0; side < 2; ++side) {
        const double weight = diagnostics_.wheels[side].confidence;
        if (weight <= 0.) {
            continue;  // Never multiply NaN by zero.
        }
        weight_sum += weight;
        weighted_speed += weight * input.center_speed_mps[side];
    }
    if (weight_sum > 0.) {
        // Fuse once: shared yaw/kinematic errors must not be counted twice.
        // One trusted wheel has twice nominal variance; two trusted wheels R_v.
        const double variance =
            parameters_.wheel_speed_variance * 2. / weight_sum;
        diagnostics_.fused_wheel_speed_mps = weighted_speed / weight_sum;
        Correct(state, covariance, (Row() << 1., 0., 0.).finished(),
                diagnostics_.fused_wheel_speed_mps, variance);
        diagnostics_.wheel_speed_used = true;
        diagnostics_.wheel_speed_variance = variance;
    }
    if (!state.allFinite() || !covariance.allFinite()) {
        Reset();
        return state_;
    }
    state_.speed = state[0];
    state_.acceleration = state[1];
    state_.acceleration_bias = state[2];
    state_.position += state_.speed * dt;
    Eigen::Map<Matrix>(covariance_.data()) = covariance;
    diagnostics_.valid = true;
    for (int i = 0; i < 3; ++i) {
        diagnostics_.covariance_diagonal[i] = covariance(i, i);
    }
    return state_;
}
}  // namespace dust_control::chassis
