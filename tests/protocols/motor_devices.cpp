/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "../support/motor_fixture.hpp"
#include "hardware/motors/dji_protocol.h"
#include "hardware/motors/motor_protocol.hpp"

namespace {
std::array<uint8_t, 8> EncodeTarget(unsigned index,
                                    dust_control::devices::MotorTarget target) {
    dust_control::devices::MotorTargets targets{};
    targets[index] = target;
    const auto batch =
        dust_control::devices::MotorEncoder{dust_control::test::MotorFixture()}
            .Encode(targets);
    assert(batch && batch->count == 1);
    const auto& address = dust_control::test::kMotorAddresses[index];
    assert(batch->frames[0].bus == address.bus);
    assert(batch->frames[0].id == address.command_id);
    return batch->frames[0].bytes;
}
}  // namespace

int main() {
    using namespace dust_control::devices;
    const MotorConfig config = dust_control::test::MotorFixture();
    for (unsigned index = 0; index < 2; ++index) {
        const double sign = config.wheel_command_sign[index];
        for (double torque : {0.0, 0.123, -0.456, 1e6, -1e6,
                              std::numeric_limits<double>::quiet_NaN()}) {
            const auto bytes = EncodeTarget(
                index, {dust_control::devices::MotorMode::kTorque, torque});
            const auto& address = dust_control::test::kMotorAddresses[index];
            const double requested = sign * torque;
            const double current =
                std::isfinite(requested)
                    ? std::clamp(requested * config.dji_current_per_nm,
                                 -double(config.dji_protocol_current_limit),
                                 double(config.dji_protocol_current_limit))
                    : 0.0;
            const auto encoded = static_cast<int16_t>(current);
            int16_t currents[4]{};
            currents[address.command_slot] = encoded;
            std::array<uint8_t, 8> expected{};
            assert(dust_control::protocols::EncodeDjiCurrentFrame0x200(
                       currents, expected.data()) == 0);
            assert(bytes == expected);
        }
    }
    for (unsigned index = 2; index < 6; ++index) {
        for (const auto mode : {dust_control::devices::MotorMode::kDisable,
                                dust_control::devices::MotorMode::kClearError,
                                dust_control::devices::MotorMode::kEnable}) {
            const auto bytes = EncodeTarget(index, {mode, 0});
            const uint8_t expected =
                mode == dust_control::devices::MotorMode::kDisable      ? 0xfd
                : mode == dust_control::devices::MotorMode::kClearError ? 0xfb
                                                                        : 0xfc;
            assert(bytes[7] == expected);
            for (unsigned byte = 0; byte < 7; ++byte) {
                assert(bytes[byte] == 0xff);
            }
        }
        for (double torque : {-2.5, 0.0, 2.5}) {
            const auto bytes = EncodeTarget(
                index, {dust_control::devices::MotorMode::kTorque, torque});
            dust_control::protocols::DmMitCommand command{};
            command.torque = float(torque);
            std::array<uint8_t, 8> expected{};
            assert(dust_control::protocols::PackDmMitCommand(
                       &command, &config.dm_range, expected.data()) == 0);
            assert(bytes == expected);
        }
    }
    const uint8_t raw[8] = {0x10, 0x80, 0x01, 0x23, 0x45, 0x67, 30, 31};
    dust_control::protocols::DmMotorFeedbackNormal decoded{};
    assert(dust_control::protocols::DecodeDmFeedbackNormal(raw, 8, &decoded) ==
           0);
    const auto feedback = DecodeMotorFeedback(2, raw, 8, 12345, config);
    assert(feedback.valid && feedback.precise_timestamp_us == 12345);
    assert(feedback.position_rad == dust_control::protocols::DmFeedbackPosition(
                                        decoded, config.dm_range));
    assert(
        feedback.velocity_rad_s ==
        dust_control::protocols::DmFeedbackVelocity(decoded, config.dm_range));
    assert(feedback.torque_nm ==
           dust_control::protocols::DmFeedbackTorque(decoded, config.dm_range));
    assert(feedback.enabled == (decoded.control_status == 1));
    const auto wheel = DecodeMotorFeedback(0, raw, 8, 100, config);
    assert(wheel.valid && wheel.speed_rpm == 0x123);
    assert(!DecodeMotorFeedback(6, raw, 8, 0, config).valid);
    assert(!DecodeMotorFeedback(0, raw, 7, 0, config).valid);
    assert(!DecodeMotorFeedback(0, nullptr, 8, 0, config).valid);
    MotorEncoder encoder(dust_control::test::MotorFixture());
    dust_control::devices::MotorTargets targets{};
    targets[0] = {dust_control::devices::MotorMode::kEnable, 0};
    assert(!encoder.Encode(targets));
    targets = {};
    targets[5].mode = static_cast<dust_control::devices::MotorMode>(255);
    assert(!encoder.Encode(targets));
    targets = {};
    const auto empty = encoder.Encode(targets);
    assert(empty && empty->count == 0);
    // 相同 CAN 地址可属于不同板，但不能把多板配置交给一个会话编解码器。
    MotorConfig multi = dust_control::test::MotorFixture();
    multi.adapters[1] = "other";
    multi.addresses[1] = multi.addresses[0];
    ValidateMotorConfig(multi);
    bool rejected_encoder = false;
    bool rejected_decoder = false;
    try {
        MotorEncoder invalid(multi);
    } catch (const std::invalid_argument&) {
        rejected_encoder = true;
    }
    try {
        MotorFeedbackDecoder invalid(multi);
    } catch (const std::invalid_argument&) {
        rejected_decoder = true;
    }
    assert(rejected_encoder && rejected_decoder);
    const auto base = MotorConfigForAdapter(multi, "base");
    const auto other = MotorConfigForAdapter(multi, "other");
    MotorFeedbackDecoder base_decoder(base);
    MotorFeedbackDecoder other_decoder(other);
    base_decoder.Consume(3, 0x201, raw, 100, true);
    other_decoder.Consume(3, 0x201, raw, 200, true);
    assert(base_decoder.Read()[0].valid && !base_decoder.Read()[1].valid);
    assert(other_decoder.Read()[1].valid && !other_decoder.Read()[0].valid);
    assert(base_decoder.Read()[0].precise_timestamp_us == 100);
    assert(other_decoder.Read()[1].precise_timestamp_us == 200);
}
