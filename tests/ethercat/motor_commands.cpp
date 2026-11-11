/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <cassert>

#include "../support/motor_fixture.hpp"
#include "chassis/control/chassis_output_policy.h"
#include "concurrency/realtime_loop.hpp"
#include "hardware/ethercat/device_pdo.hpp"
#include "hardware/ethercat/session.hpp"
#include "hardware/ethercat/wire.hpp"

namespace devices = dust_control::devices;
namespace chassis = dust_control::chassis;
namespace host = dust_control::host;
namespace ec = dust_control::ethercat;

namespace {
void EqualFrames(const devices::CanTransmitBatch& actual,
                 const devices::CanTransmitBatch& expected) {
    assert(actual.count == expected.count);
    for (size_t i = 0; i < actual.count; ++i) {
        assert(actual.frames[i].bus == expected.frames[i].bus);
        assert(actual.frames[i].id == expected.frames[i].id);
        assert(actual.frames[i].bytes == expected.frames[i].bytes);
    }
}

void PolicyAndBatchFormats() {
    const auto motors = dust_control::test::MotorFixture();
    const devices::MotorEncoder wire_encoder(motors);
    const host::MotorCommandEncoder fixed(motors);
    chassis::ChassisOutputPolicy policy;
    chassis::ChassisCycleInput input{};
    chassis::ChassisControlOutput output{};

    // 任一安全状态都须从本周期策略生成完整安全目标，即使前一周期有力矩。
    for (const auto state : {chassis::ChassisControlState::kDisabled,
                             chassis::ChassisControlState::kSafetyStop,
                             chassis::ChassisControlState::kActionFault,
                             chassis::ChassisControlState::kTiltFault}) {
        for (uint32_t tick : {0U, 1U, 9U, 10U, 99U, 100U}) {
            output.physical_wheel_torque = {1.0, -1.0};
            policy.Apply(chassis::ChassisControlState::kBalance, tick, input,
                         output);
            policy.Apply(state, tick, input, output);
            devices::MotorTargets expected{};
            for (size_t i = 0; i < expected.size(); ++i) {
                const auto mode = i < 2 ? devices::MotorMode::kTorque
                                        : devices::MotorMode::kDisable;
                expected[i] = {mode, 0};
                assert(output.idle_motor_targets[i].mode == mode);
                assert(output.idle_motor_targets[i].torque_nm == 0);
            }
            const auto full =
                fixed.Encode(output.motor_targets, output.idle_motor_targets);
            const auto expected_full = wire_encoder.Encode(expected);
            assert(full && expected_full && full->count == 6);
            EqualFrames(*full, *expected_full);
        }
    }

    // 显式模式命令优先；间隔周期补零力矩，不重发清错或使能命令。
    policy.ResetArming();
    for (uint32_t tick = 0; tick < 300; ++tick) {
        policy.Apply(chassis::ChassisControlState::kDmArming, tick, input,
                     output);
        devices::MotorTargets expected{};
        for (size_t i = 0; i < expected.size(); ++i) {
            auto mode = devices::MotorMode::kTorque;
            if (i >= 2 && tick % 10 == 0) {
                mode = tick < 50 ? devices::MotorMode::kClearError
                                 : devices::MotorMode::kEnable;
            }
            expected[i] = {mode, 0};
        }
        const auto batch =
            fixed.Encode(output.motor_targets, output.idle_motor_targets);
        const auto expected_batch = wire_encoder.Encode(expected);
        assert(batch && expected_batch);
        EqualFrames(*batch, *expected_batch);
    }

    // 连续控制的显式力矩保持原样，不被空槽目标覆盖。
    output.physical_wheel_torque = {0.123, -0.456};
    policy.Apply(chassis::ChassisControlState::kBalance, 301, input, output);
    const auto batch =
        fixed.Encode(output.motor_targets, output.idle_motor_targets);
    const auto expected = wire_encoder.Encode(output.motor_targets);
    assert(batch && expected);
    EqualFrames(*batch, *expected);
}

void SessionOwnedEncoder() {
    namespace dw = dust_control::ethercat::device;
    namespace ds = dust_control::ethercat::sensor;
    const auto motors = dust_control::test::MotorFixture();
    const ec::AdapterPdoConfig capacity;
    ec::PdoConfig config;
    config.cycle_time_us = 125;
    config.adapters["base"] = capacity;
    const auto layout = ec::BuildPdoLayout(config).adapters.at("base");
    host::EthercatSession session(layout, motors);
    std::array<host::CanCommandGroup, 6> groups{};
    for (size_t i = 0; i < groups.size(); ++i) {
        groups[i] = {dw::kRoutes[i].bus, dw::kRoutes[i].command};
    }
    assert(session.transmitter().ConfigureCan(groups, 20000000));
    session.transmitter().SealRegistration();
    const auto now = dust_control::realtime::MonotonicTimeNs();
    constexpr uint32_t nonce = 77;
    dw::Payload token{};
    ds::Put32(token.data(), nonce);
    token[4] = 2;
    const auto status = dw::Encode(dw::Type::Status, 1, token);
    std::array<uint8_t, 180> input{};
    std::copy(status.begin(), status.end(), input.begin());
    assert(session.Receive(input, now, nonce));
    session.Activate(now);

    chassis::ChassisOutputPolicy policy;
    chassis::ChassisControlOutput output{};
    policy.Apply(chassis::ChassisControlState::kSafetyStop, 1, {}, output);
    const auto batch = session.motor_encoder().Encode(
        output.motor_targets, output.idle_motor_targets);
    assert(batch && batch->count == 6);
    assert(session.transmitter().SubmitCan({batch->frames.data(), batch->count},
                                           1, now));
    std::array<uint8_t, 60> image{};
    assert(session.Encode(image, now, nonce, ec::wire::kArm));
    uint8_t sequence = 0;
    assert(dw::DecodeMotorBatch(image.data(), image.size(), sequence));
    for (size_t i = 0; i < groups.size(); ++i) {
        dw::View record{};
        assert(
            dw::Decode(dw::Direction::Rx, image.data() + i * 10, 10, record));
        const auto frame = std::find_if(
            batch->frames.begin(), batch->frames.begin() + batch->count,
            [&](const auto& value) {
                return value.bus == groups[i].bus && value.id == groups[i].id;
            });
        assert(frame != batch->frames.begin() + batch->count);
        assert(record.payload == frame->bytes);
    }
}

void RejectIncompleteOrInvalidTargets() {
    const auto motors = dust_control::test::MotorFixture();
    const host::MotorCommandEncoder fixed(motors);
    devices::MotorTargets empty{};
    assert(!fixed.Encode(empty, empty));
    devices::MotorTargets idle{};
    for (auto& target : idle) {
        target = {devices::MotorMode::kTorque, 0};
    }
    auto invalid = empty;
    invalid[2] = {static_cast<devices::MotorMode>(99), 0};
    assert(!fixed.Encode(invalid, idle));
    idle[2] = invalid[2];
    assert(!fixed.Encode(empty, idle));
    idle[2] = {devices::MotorMode::kTorque, 0};
    auto partial = motors;
    partial.present[5] = false;
    const host::MotorCommandEncoder incomplete(partial);
    assert(!incomplete.Encode(empty, idle));
}
}  // namespace

int main() {
    PolicyAndBatchFormats();
    RejectIncompleteOrInvalidTargets();
    SessionOwnedEncoder();
}
