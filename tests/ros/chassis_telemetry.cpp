/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_telemetry.hpp"

#include <cassert>
#include <cmath>

int main() {
    dust_control::chassis::ChassisObservation sample;
    sample.sequence = 17;
    sample.monotonic_ns = 1000000000;
    sample.control_duration_ns = 12500;
    sample.input.imu_fresh = true;
    sample.input.imu.pitch_deg = 12.5;
    sample.input.imu_age_us = 123;
    sample.input.leg.left.length = .25;
    sample.input.leg.left.hz = -.23;
    sample.input.leg.right.length = .26;
    const rclcpp::Time now(2000000000LL);
    auto message =
        dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(message.sequence == 17 && message.version == 1);
    assert(message.header.stamp.sec == 1 &&
           message.header.stamp.nanosec == 990000000);
    assert(message.pitch_deg == 12.5 && message.imu_age_us == 123);
    assert(message.left_length_m == .25 && message.left_height_m == .23);
    assert(message.transport_age_ms == 10 &&
           message.control_duration_us == 12.5);
    sample.input.motor_stale_mask = 0x0c;
    sample.input.imu_fresh = false;
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(std::isnan(message.pitch_deg));
    assert(!message.left_leg_fresh && std::isnan(message.left_length_m));
    assert(message.right_leg_fresh && message.right_length_m == .26);
    sample.input.feedback_invalid_mask = 8;
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1010000000);
    assert(!message.right_leg_fresh && std::isnan(message.right_length_m));
    message = dust_control::ros::MakeChassisTelemetry(sample, now, 1300000000);
    assert(!message.source_connected && !message.imu_fresh);
    assert(message.motor_stale_mask == 0x3f && std::isnan(message.imu_age_us));
    assert(!message.right_leg_fresh && !message.left_leg_fresh);
}
