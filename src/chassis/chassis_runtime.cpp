/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_runtime.hpp"

#include <pthread.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "chassis/runtime/realtime_loop.hpp"
#include "hardware/imu/hi91_decoder.hpp"
#include "hardware/motors/motor_protocol.hpp"
#include "hardware/remote/sbus_decoder.hpp"
#include "hardware/usb/bridge_protocol.hpp"
#include "hardware/usb/usb_transmitter.hpp"

namespace dust::chassis {
ChassisRuntime::ChassisRuntime(ChassisRuntimeOptions options,
                               const devices::MotorConfig& motors,
                               modules::InputDeadlines deadlines,
                               ChassisDevices devices,
                               modules::ChassisConfig config)
    : config_(config),
      options_(options),
      devices_(std::move(devices)),
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
        if (!adapter.feedback || !adapter.transmitter) {
            throw std::invalid_argument("Incomplete chassis motor adapter: " +
                                        adapter.name);
        }
        encoders_.emplace_back(adapter.motors);
        std::array<wbr::host::CanCommandGroup, 6> groups{};
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
            const wbr::host::CanCommandGroup group{motor.bus, motor.command_id};
            if (std::find(groups.begin(), groups.begin() + count, group) ==
                groups.begin() + count) {
                groups[count++] = group;
            }
        }
        if (!count ||
            !adapter.transmitter->ConfigureCan(
                wbr::host::CanProducer::kChassis, {groups.data(), count},
                wbr::bridge::kCommandTimeoutUs * 1000ULL)) {
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
                                                  : "USB Bulk / CAN WBD")
              << ", output="
              << (options_.enable_output ? "remote-gated" : "disabled")
              << std::endl;
    const uint64_t start = wbr::realtime::MonotonicTimeNs();
    uint64_t release = start;
    bool disabled_observed = false;
    unsigned late_cycles = 0;
    uint64_t observation_sequence = 0;
    uint64_t command_sequence = 0;
    uint64_t observation_drops = 0;
    ChassisObservation sample;
    while (!stop_.load()) {
        release += modules::chassis_config::kControlPeriodNs;
        if (!wbr::realtime::SleepUntilNs(release, error)) {
            if (stop_.load()) {
                break;
            }
            throw std::runtime_error(error);
        }
        const uint64_t now = wbr::realtime::MonotonicTimeNs();
        // 不补发错过的控制周期。停顿达到 20 ms 时已超过 MCU 看门狗期限，
        // 必须建立新会话，不能自动重新 Arm。
        if (now - release >= wbr::bridge::kCommandTimeoutUs * 1000ULL) {
            throw std::runtime_error(
                "Control deadline exceeded bridge watchdog");
        }
        if (now - release >= modules::chassis_config::kControlPeriodNs) {
            const auto missed =
                (now - release) / modules::chassis_config::kControlPeriodNs;
            release += missed * modules::chassis_config::kControlPeriodNs;
            late_cycles += missed;
        }
        modules::ControllerInputs input{};
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
        disabled_observed |= input.remote_disable_pose;
        const auto tick_start = wbr::realtime::MonotonicTimeNs();
        const auto output = controller_.Tick(
            input, tick_start / 1000,
            options_.enable_output && disabled_observed &&
                (!options_.startup_ready || options_.startup_ready->load()),
            &sample.input);
        ++command_sequence;
        // 先编码全部板；任一编码失败时，不提交本周期的部分目标。
        std::array<devices::CanTransmitBatch, 6> batches{};
        for (size_t i = 0; i < encoders_.size(); ++i) {
            auto targets = output.motor_targets;
            const auto& motors = devices_.motor_adapters[i].motors;
            const auto state = controller_.state();
            const bool safe =
                state == modules::ChassisControlState::kDisabled ||
                state == modules::ChassisControlState::kSafetyStop ||
                state == modules::ChassisControlState::kActionFault ||
                state == modules::ChassisControlState::kTiltFault;
            // 仅关节板也必须在 20 ms 租约内收到真实安全命令。
            // 根据当前安全状态重新生成失能指令，不重放历史目标或发送假心跳。
            if (safe && !motors.present[0] && !motors.present[1] &&
                command_sequence % 10 == 0) {
                for (size_t joint = 2; joint < targets.size(); ++joint) {
                    if (motors.present[joint]) {
                        targets[joint] = {devices::MotorMode::kDisable, 0};
                    }
                }
            }
            const auto encoded = encoders_[i].Encode(targets);
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
            if (!adapter.transmitter->SubmitCan(
                    wbr::host::CanProducer::kChassis,
                    {batch.frames.data(), batch.count}, command_sequence,
                    tick_start)) {
                throw std::runtime_error("Adapter " + adapter.name +
                                         " CAN submission rejected: " +
                                         adapter.transmitter->error());
            }
        }
        sample.monotonic_ns = tick_start;
        sample.control_duration_ns =
            wbr::realtime::MonotonicTimeNs() - tick_start;
        sample.sequence = ++observation_sequence;
        sample.missed_releases = late_cycles;
        sample.queue_drops = observation_drops;
        sample.state = controller_.state();
        sample.simulated = devices_.motor_adapters.empty();
        if (!observations_.Push(sample)) {
            ++observation_drops;
        }
    }
    std::cout << "Stopped; missed control releases=" << late_cycles
              << std::endl;
}
}  // namespace dust::chassis
