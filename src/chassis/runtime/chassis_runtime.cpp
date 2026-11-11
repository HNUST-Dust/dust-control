/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis/runtime/chassis_runtime.hpp"

#include <pthread.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "concurrency/realtime_loop.hpp"
#include "hardware/ethercat/commands.hpp"
#include "hardware/imu/hi91_imu.hpp"
#include "hardware/motors/motor_protocol.hpp"
#include "hardware/remote/wfly_sbus_protocol.h"

namespace dust_control::chassis {
ChassisRuntime::ChassisRuntime(ChassisRuntimeOptions options,
                               const devices::MotorConfig& motors,
                               dust_control::chassis::InputDeadlines deadlines,
                               ChassisDevices devices,
                               dust_control::chassis::ChassisConfig config)
    : options_(options),
      devices_(std::move(devices)),
      wheel_command_sign_(motors.wheel_command_sign),
      controller_(deadlines, motors.wheel_torque_limit_nm(), config) {
    concurrency::ValidateThreadOptions(options_.thread);
    const bool any =
        !devices_.motor_adapters.empty() || devices_.remote || devices_.imu;
    const bool all =
        !devices_.motor_adapters.empty() && devices_.remote && devices_.imu;
    if ((any && !all) || (!all && options_.enable_output)) {
        throw std::invalid_argument(
            "Incomplete chassis devices or offline output request");
    }
    std::array<bool, 6> owned{};
    for (const auto& adapter : devices_.motor_adapters) {
        if (!adapter.feedback || !adapter.transmitter || !adapter.encoder) {
            throw std::invalid_argument("Incomplete chassis motor adapter: " +
                                        adapter.name);
        }
        std::array<dust_control::host::CanCommandGroup, 6> groups{};
        size_t count = 0;
        for (size_t i = 0; i < owned.size(); ++i) {
            if (!adapter.motors.present[i]) {
                continue;
            }
            if (owned[i]) {
                throw std::invalid_argument("Duplicate chassis motor owner");
            }
            owned[i] = true;
            const auto& motor = adapter.motors.addresses[i];
            const dust_control::host::CanCommandGroup group{motor.bus,
                                                            motor.command_id};
            if (std::find(groups.begin(), groups.begin() + count, group) ==
                groups.begin() + count) {
                groups[count++] = group;
            }
        }
        if (!count || !adapter.transmitter->ConfigureCan(
                          {groups.data(), count},
                          dust_control::host::kCommandTimeoutUs * 1000ULL)) {
            throw std::runtime_error("Chassis CAN registration rejected: " +
                                     adapter.name);
        }
    }
    if (all && !std::all_of(owned.begin(), owned.end(),
                            [](bool value) { return value; })) {
        throw std::invalid_argument(
            "Chassis requires all six motors across its adapters");
    }
}
ChassisRuntime::~ChassisRuntime() {
    Stop();
}
void ChassisRuntime::Start() {
    if (worker_.joinable() || finished_ || stop_) {
        throw std::logic_error("Chassis runtime cannot be restarted");
    }
    worker_ = std::thread([this] {
        try {
            Run();
        } catch (const std::exception& error) {
            error_ = error.what();
        }
        finished_ = true;
    });
}
void ChassisRuntime::Stop() {
    stop_ = true;
    if (worker_.joinable()) {
        worker_.join();
    }
}
void ChassisRuntime::Run() {
    std::string error;
    if (!concurrency::ConfigureThread(options_.thread, error)) {
        throw std::runtime_error(error);
    }
    pthread_setname_np(pthread_self(), "dust-chassis");
    std::cout << "Chassis: 1 kHz, "
              << (devices_.motor_adapters.empty() ? "offline"
                                                  : "IgH EtherCAT / CAN")
              << ", output="
              << (options_.enable_output ? "remote-gated" : "disabled")
              << std::endl;
    const uint64_t start = dust_control::realtime::MonotonicTimeNs();
    uint64_t release = start;
    bool disabled_observed = false;
    unsigned late_cycles = 0;
    uint64_t observation_sequence = 0;
    uint64_t command_sequence = 0;
    uint64_t observation_drops = 0;
    ChassisObservation sample;
    while (!stop_.load()) {
        if (!WaitForNextCycle(release, late_cycles)) {
            break;
        }
        const auto input = ReadInputs();
        disabled_observed |= input.remote_disable_pose;
        const auto tick_start = dust_control::realtime::MonotonicTimeNs();
        const auto output = controller_.Tick(
            input, tick_start / 1000,
            options_.enable_output && disabled_observed &&
                (!options_.startup_ready || options_.startup_ready->load()),
            &sample.input);
        sample.motor_targets = output.motor_targets;
        sample.wheel_command_sign = wheel_command_sign_;
        sample.motion = output.motion;
        sample.lqr = output.lqr;
        sample.lqr.output_wheel_torque = output.physical_wheel_torque;
        SubmitMotorTargets(output, ++command_sequence, tick_start);
        PublishObservation(sample, tick_start, ++observation_sequence,
                           late_cycles, observation_drops);
    }
    std::cout << "Stopped; missed control releases=" << late_cycles
              << std::endl;
}

bool ChassisRuntime::WaitForNextCycle(uint64_t& release,
                                      unsigned& late_cycles) {
    std::string error;
    release += chassis_config::kControlPeriodNs;
    if (!dust_control::realtime::SleepUntilNs(release, error)) {
        if (stop_.load()) {
            return false;
        }
        throw std::runtime_error(error);
    }
    const uint64_t now = dust_control::realtime::MonotonicTimeNs();
    // 实机停顿20ms已超过MCU看门狗期限，必须建立新会话。
    // 无硬件离线运行没有MCU会话，只统计漏周期，不因宿主机调度暂停退出。
    if (!devices_.motor_adapters.empty() &&
        now - release >= dust_control::host::kCommandTimeoutUs * 1000ULL) {
        throw std::runtime_error("Control deadline exceeded bridge watchdog");
    }
    if (now - release >= chassis_config::kControlPeriodNs) {
        const auto missed = (now - release) / chassis_config::kControlPeriodNs;
        release += missed * chassis_config::kControlPeriodNs;
        late_cycles += missed;
    }
    return true;
}

ControllerInputs ChassisRuntime::ReadInputs() const {
    ControllerInputs input{};
    for (const auto& adapter : devices_.motor_adapters) {
        if (adapter.started && !adapter.started->load()) {
            continue;
        }
        if (!adapter.transmitter->healthy()) {
            throw std::runtime_error("Adapter " + adapter.name + ": " +
                                     adapter.transmitter->error());
        }
        const auto feedback = adapter.feedback->Read();
        for (size_t i = 0; i < feedback.size(); ++i) {
            if (adapter.motors.present[i]) {
                input.motors[i] = feedback[i];
            }
        }
    }
    if (devices_.remote) {
        const auto remote = devices_.remote->Read();
        input.remote = remote.remote;
        input.remote_disable_pose = remote.disable_pose;
        input.imu = devices_.imu->Read();
    }
    return input;
}

void ChassisRuntime::SubmitMotorTargets(const ChassisControlOutput& output,
                                        uint64_t command_sequence,
                                        uint64_t tick_start) {
    // 先编码全部板；任一编码失败时，不提交本周期的部分目标。
    std::array<devices::CanTransmitBatch, 6> batches{};
    for (size_t i = 0; i < devices_.motor_adapters.size(); ++i) {
        const auto& adapter = devices_.motor_adapters[i];
        const auto encoded = adapter.encoder->Encode(output.motor_targets,
                                                     output.idle_motor_targets);
        if (!encoded) {
            throw std::runtime_error("Invalid motor control targets");
        }
        batches[i] = *encoded;
    }
    for (size_t i = 0; i < devices_.motor_adapters.size(); ++i) {
        const auto& adapter = devices_.motor_adapters[i];
        if (adapter.started && !adapter.started->load()) {
            continue;
        }
        const auto& batch = batches[i];
        if (!adapter.transmitter->SubmitCan({batch.frames.data(), batch.count},
                                            command_sequence, tick_start)) {
            throw std::runtime_error(
                "Adapter " + adapter.name +
                " CAN submission rejected: " + adapter.transmitter->error());
        }
    }
}

void ChassisRuntime::PublishObservation(ChassisObservation& sample,
                                        uint64_t tick_start,
                                        uint64_t observation_sequence,
                                        unsigned late_cycles,
                                        uint64_t& observation_drops) {
    sample.monotonic_ns = tick_start;
    sample.control_duration_ns =
        dust_control::realtime::MonotonicTimeNs() - tick_start;
    sample.sequence = observation_sequence;
    sample.missed_releases = late_cycles;
    sample.queue_drops = observation_drops;
    sample.state = controller_.state();
    sample.simulated = devices_.motor_adapters.empty();
    if (!observations_.Push(sample)) {
        ++observation_drops;
    }
}
}  // namespace dust_control::chassis
