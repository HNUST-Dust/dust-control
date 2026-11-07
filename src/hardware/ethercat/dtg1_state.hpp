/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "common/device_sensor_page.hpp"
namespace dust_control::host {
struct Dtg1State {
    std::array<uint8_t,60> held{};
    std::array<uint8_t,6> motor_sequence{};
    std::array<bool,6> have_motor{};
    uint32_t command_source_sequence=0, hi91_sequence=0, sbus_ack=0;
    uint8_t sequence=0, ack_repeats=0;
    bool have_command=false, have_hi91=false;
    dust::device_sensor_page::Assembler sbus{dust::device_sensor_page::Kind::Sbus,20000};
    dust::device_sensor_page::Assembler imu{dust::device_sensor_page::Kind::ImuCompact,20000};
};
}
