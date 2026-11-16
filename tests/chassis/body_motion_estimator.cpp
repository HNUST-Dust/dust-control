/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis/estimation/body_motion_estimator.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "chassis/balance/balance_controller.h"
#include "chassis/control/chassis_input_reader.h"

namespace c = dust_control::chassis;
namespace {
c::BodyMotionObservation Grounded() {
    c::BodyMotionObservation input;
    input.wheel_valid = {true, true};
    input.support_valid = input.yaw_valid = input.acceleration_valid = true;
    input.support_fraction = {.5, .5};
    return input;
}
void BiasAndAcceleration() {
    c::BodyMotionEstimator estimator;
    auto input = Grounded();
    double speed = 0.;
    double squared_error = 0.;
    for (int k = 0; k < 30000; ++k) {
        const double acceleration = .4 * std::sin(k * .001);
        speed += acceleration * input.dt_s;
        input.center_speed_mps = {speed + .15 * std::sin(k * .9),
                                  speed + .15 * std::sin(k * .9)};
        input.forward_acceleration_mps2 = acceleration + .3;
        const auto state = estimator.Update(input);
        if (k > 20000) {
            squared_error += std::pow(state.speed - speed, 2);
        }
    }
    assert(std::sqrt(squared_error / 9999) < .08);
    const auto state = estimator.Update(input);
    assert(std::abs(state.acceleration_bias - .3) < .08);
    assert(estimator.diagnostics().valid);
}
void AirborneAndLanding() {
    c::BodyMotionEstimator estimator;
    auto input = Grounded();
    for (int k = 0; k < 1000; ++k) {
        estimator.Update(input);
    }
    input.support_fraction = {0., 0.};
    input.center_speed_mps = {20., -30.};
    for (int k = 0; k < 20; ++k) {
        estimator.Update(input);
    }
    const double before = estimator.diagnostics().covariance_diagonal[0];
    for (int k = 0; k < 500; ++k) {
        estimator.Update(input);
    }
    assert(!estimator.diagnostics().wheel_speed_used);
    assert(estimator.diagnostics().wheels[0].status ==
           c::WheelSpeedStatus::kAirborne);
    assert(std::abs(estimator.Update(input).speed) < .05);
    assert(estimator.diagnostics().covariance_diagonal[0] > before);
    input.support_fraction = {.5, .5};
    input.center_speed_mps = {0., 0.};
    estimator.Update(input);
    assert(estimator.diagnostics().wheels[0].confidence < .1);
    for (int k = 0; k < 200; ++k) {
        estimator.Update(input);
    }
    assert(estimator.diagnostics().wheels[0].status ==
           c::WheelSpeedStatus::kTrusted);
}
void SlippingWheelAndCommonSlip() {
    c::BodyMotionEstimator estimator;
    auto input = Grounded();
    for (int k = 0; k < 3000; ++k) {
        estimator.Update(input);
    }
    input.center_speed_mps = {4., 0.};
    for (int k = 0; k < 100; ++k) {
        estimator.Update(input);
    }
    auto diagnostics = estimator.diagnostics();
    assert(diagnostics.wheels[0].confidence == 0.);
    assert(diagnostics.wheels[0].prediction_gate_active);
    assert(diagnostics.wheels[0].prediction_ratio > 3.);
    assert(diagnostics.wheels[0].evidence & c::kHardRejection);
    assert(diagnostics.wheels[0].pair_comparison_valid);
    assert(diagnostics.prior_speed_variance > 0.);
    assert(diagnostics.center_speed_mps[0] == 4.);
    assert(diagnostics.parameters.reliability.pair_residual_limit_mps == .4);
    assert(diagnostics.wheels[1].confidence == 1.);
    assert(std::abs(estimator.Update(input).speed) < .05);
    input.center_speed_mps = {4., 4.};
    for (int k = 0; k < 100; ++k) {
        estimator.Update(input);
    }
    assert(!estimator.diagnostics().wheel_speed_used);
    assert(std::abs(estimator.Update(input).speed) < .05);
    input.center_speed_mps = {0., 0.};
    for (int k = 0; k < 200; ++k) {
        estimator.Update(input);
    }
    assert(estimator.diagnostics().wheels[0].confidence == 1.);
}
void NoImuAndInvalidInputs() {
    c::BodyMotionEstimator estimator;
    auto input = Grounded();
    input.acceleration_valid = false;
    input.center_speed_mps = {2., 0.};
    for (int k = 0; k < 200; ++k) {
        estimator.Update(input);
    }
    assert(estimator.diagnostics().wheels[0].status ==
           c::WheelSpeedStatus::kInconsistent);
    assert(estimator.diagnostics().wheels[1].status ==
           c::WheelSpeedStatus::kInconsistent);
    // With no inertia, pair agreement must allow recovery even far from old
    // speed.
    input.center_speed_mps = {2., 2.};
    for (int k = 0; k < 5000; ++k) {
        estimator.Update(input);
    }
    const auto previous = estimator.Update(input);
    assert(std::abs(previous.speed - 2.) < .1);
    input.dt_s = std::numeric_limits<double>::quiet_NaN();
    assert(estimator.Update(input).position == previous.position);
    assert(!estimator.diagnostics().valid);
    input.dt_s = .001;
    input.center_speed_mps = {std::numeric_limits<double>::quiet_NaN(), 2.};
    assert(std::isfinite(estimator.Update(input).speed));
    assert(estimator.diagnostics().wheels[0].confidence == 0.);
    estimator.Reset();
    input = {};
    auto state = estimator.Update(input);
    assert(state.speed == 0. && state.position == 0. &&
           state.acceleration_bias == 0.);
    assert(!estimator.diagnostics().wheel_speed_used);
    c::BodyMotionParameters bad;
    bad.wheel_speed_variance = -1.;
    bool threw = false;
    try {
        c::BodyMotionEstimator rejected(bad);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}
void DuplicateImuDoesNotCorrectTwice() {
    c::BodyMotionEstimator with_sample, without_sample;
    c::BodyMotionObservation input;
    input.acceleration_valid = true;
    input.forward_acceleration_mps2 = 1.;
    with_sample.Update(input);
    without_sample.Update(input);
    input.acceleration_new_sample = false;
    with_sample.Update(input);
    input.acceleration_valid = false;
    without_sample.Update(input);
    const auto a = with_sample.diagnostics().covariance_diagonal;
    const auto b = without_sample.diagnostics().covariance_diagonal;
    for (int i = 0; i < 3; ++i) {
        assert(std::abs(a[i] - b[i]) < 1e-12);
    }
    assert(!with_sample.diagnostics().acceleration_used);
}
void ContactHysteresisAndYawCompensation() {
    c::WheelSpeedReliability reliability;
    c::WheelReliabilityInput input;
    input.wheel_valid = {true, true};
    input.yaw_valid = input.support_valid = true;
    input.support_fraction = {.5, .5};
    input.center_speed_mps = {.7, .7};
    for (int k = 0; k < 200; ++k) {
        reliability.Update(input);
    }
    assert(reliability.Update(input)[0].status ==
           c::WheelSpeedStatus::kTrusted);
    input.support_fraction[0] = 0.;
    assert(reliability.Update(input)[0].status !=
           c::WheelSpeedStatus::kAirborne);
    input.support_fraction[0] =
        .08;  // dead band does not toggle established contact
    for (int k = 0; k < 20; ++k) {
        reliability.Update(input);
    }
    assert(reliability.Update(input)[0].status ==
           c::WheelSpeedStatus::kTrusted);
    input.support_fraction[0] = 0.;
    for (int k = 0; k < 10; ++k) {
        reliability.Update(input);
    }
    assert(reliability.Update(input)[0].status ==
           c::WheelSpeedStatus::kAirborne);
}
void ControlBoundaryAndAccelerationCalibration() {
    c::ChassisConfig config;
    c::BalanceController balance(c::chassis_config::kWheelTorqueLimit, config);
    c::ChassisCycleInput input;
    input.feedback_valid = input.imu_fresh = input.support_force_valid = true;
    input.measured_support_force_n = {60., 60.};
    input.imu.yaw_rate_rad_s = 4.;
    // Pure CCW spin: physical left speed -d*w, right +d*w. Raw motor
    // signs differ; both raw rpm values therefore have the same sign.
    const double rpm =
        config.half_wheel_track * 4. * config.wheel_reduction /
        (config.wheel_radius * c::chassis_config::kRpmToRadPerSec);
    input.wheel_motor_speed_rpm = {rpm, rpm};
    c::ChassisControlOutput output;
    for (int k = 0; k < 200; ++k) {
        balance.Update(input, output);
    }
    assert(std::abs(output.estimated_body_speed) < 1e-10);
    assert(output.motion.estimation.wheels[0].confidence == 1.);
    assert(std::abs(output.motion.estimation.wheels[0].pair_residual_mps) <
           1e-10);

    config.forward_acceleration_axis = 1;
    config.forward_acceleration_sign = -1.;
    c::ChassisInputReader reader({}, config);
    c::ControllerInputs sources;
    sources.imu.valid = true;
    sources.imu.sequence = 1;
    sources.imu.precise_timestamp_us = 1000;
    sources.imu.pitch_deg = 30.;
    sources.imu.accel_g[1] =
        .5;  // static tilt, selected axis has negative sign
    reader.Read(sources, input, 1000, .001, false);
    assert(input.forward_acceleration_valid);
    assert(std::abs(input.forward_acceleration_mps2) < 1e-6);
    sources.imu.accel_g[1] = .5 - .8 / c::chassis_config::kGravity;
    reader.Read(sources, input, 1000, .001, false);
    assert(std::abs(input.forward_acceleration_mps2 - .8) < 1e-6);
    sources.imu.yaw_rate_rad_s = 2.;
    reader.Read(sources, input, 1000, .001, false);
    assert(!input.forward_acceleration_valid);
    c::ChassisInputReader uncalibrated;
    sources.imu.yaw_rate_rad_s = 0.;
    uncalibrated.Read(sources, input, 1000, .001, false);
    assert(!input.forward_acceleration_valid);
}
}  // namespace
int main() {
    Eigen::internal::set_is_malloc_allowed(false);
    BiasAndAcceleration();
    AirborneAndLanding();
    SlippingWheelAndCommonSlip();
    NoImuAndInvalidInputs();
    DuplicateImuDoesNotCorrectTwice();
    ContactHysteresisAndYawCompensation();
    ControlBoundaryAndAccelerationCalibration();
}
