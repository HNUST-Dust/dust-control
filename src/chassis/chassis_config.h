/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <chassis_params_generated.h>

#include <cstddef>
#include <cstdint>

#include "chassis_types.h"

namespace modules::chassis_config {

using namespace chassis_params;

struct LegHardwareMap {
    int kinematic_branch;
    double leg_angle_offset;
    double leg_coordinate_sign;
    double wheel_feedback_sign;
    double vmc_torque_sign;
    double wheel_command_sign;
};

inline constexpr SidePair<LegHardwareMap> kLegHardware = {
    .left = {kLeftKinematicBranch, kLeftLegAngleOffset, kLeftLegCoordinateSign,
             kLeftWheelFeedbackSign, kLeftVmcTorqueSign, kLeftWheelCommandSign},
    .right = {kRightKinematicBranch, kRightLegAngleOffset,
              kRightLegCoordinateSign, kRightWheelFeedbackSign,
              kRightVmcTorqueSign, kRightWheelCommandSign},
};

inline constexpr double kTwoPi = 6.28318530717958647692;
inline constexpr double kDegToRad = 0.01745329251994329577;
inline constexpr double kDpsToRadPerSec = kDegToRad;
inline constexpr double kRpmToRadPerSec = 0.10471975511965977;
inline constexpr double kMaxYawRate = kMaxTurnWheelSpeed / kHalfWheelTrack;
inline constexpr double kYawAccelerationLimit =
    kTurnAccelerationLimit / kHalfWheelTrack;
inline constexpr double kYawTrackingErrorLimit =
    kYawTrackingErrorLimitDeg * kDegToRad;
inline constexpr double kWheelTorqueLimit =
    static_cast<double>(kDjiProtocolCurrentLimit) / kDjiCurrentPerNm;
inline constexpr double kThetaBalanceBias = kThetaBalanceBiasDeg * kDegToRad;

}  // namespace modules::chassis_config
