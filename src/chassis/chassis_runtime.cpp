/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_runtime.hpp"

#include <pthread.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "chassis/runtime/realtime_loop.hpp"
#include "command_arbiter.hpp"
#include "hardware/imu/hi91_decoder.hpp"
#include "hardware/motors/motor_feedback.hpp"
#include "hardware/remote/sbus_decoder.hpp"
#include "hardware/usb/bridge_protocol.hpp"
#include "hardware/usb/usb_transmitter.hpp"

namespace dust::chassis {
ChassisRuntime::ChassisRuntime(ChassisRuntimeOptions options,
                               const devices::MotorConfig& motors,
                               modules::InputDeadlines deadlines,
                               ChassisDevices devices,
                               wbr::host::TelemetryPublisher* telemetry,
                               modules::ChassisConfig config)
    : config_(config),
      options_(options),
      devices_(devices),
      telemetry_(telemetry),
      encoder_(motors),
      controller_(deadlines, motors.wheel_torque_limit_nm(), config) {
    const bool any = devices_.motors || devices_.remote || devices_.imu ||
                     devices_.transmitter;
    const bool all = devices_.motors && devices_.remote && devices_.imu &&
                     devices_.transmitter;
    if ((any && !all) || (!all && options_.enable_output)) {
        throw std::invalid_argument(
            "Incomplete chassis devices or offline output request");
    }
    if (all) {
        std::array<wbr::host::CanCommandGroup, 6> groups{};
        size_t count = 0;
        for (const auto& motor : motors.addresses) {
            const wbr::host::CanCommandGroup group{motor.bus, motor.command_id};
            if (std::find(groups.begin(), groups.begin() + count, group) ==
                groups.begin() + count) {
                groups[count++] = group;
            }
        }
        if (!devices_.transmitter->ConfigureCan(
                wbr::host::CanProducer::kChassis, {groups.data(), count},
                wbr::bridge::kCommandTimeoutUs * 1000ULL)) {
            throw std::runtime_error(
                "Chassis CAN command group registration rejected");
        }
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
    if (options_.realtime && !wbr::realtime::ConfigureCurrentThread(
                                 options_.cpu, options_.priority, error)) {
        throw std::runtime_error(error);
    }
    pthread_setname_np(pthread_self(), "dust-chassis");
    std::cout << "chassis_controller: 1 kHz, "
              << (!devices_.motors ? "offline" : "USB Bulk / CAN WBD")
              << ", output="
              << (options_.enable_output ? "remote-gated" : "disabled")
              << std::endl;
    const uint64_t start = wbr::realtime::MonotonicTimeNs();
    uint64_t release = start;
    const uint64_t finish = options_.seconds
                                ? start + uint64_t(options_.seconds * 1e9)
                                : UINT64_MAX;
    bool disabled_observed = false;
    unsigned late_cycles = 0;
    uint64_t next_telemetry = start;
    uint64_t telemetry_sequence = 0;
    uint64_t command_sequence = 0;
    uint64_t telemetry_drops = 0;
    wbr::host::TelemetrySample sample;
    MotionCommand command{};
    while (!stop_.load() && wbr::realtime::MonotonicTimeNs() < finish) {
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
        if (devices_.transmitter && !devices_.transmitter->healthy()) {
            throw std::runtime_error(devices_.transmitter->error().empty()
                                         ? "Bridge sensor stream timeout"
                                         : devices_.transmitter->error());
        }
        modules::ControllerInputs input{};
        if (devices_.motors) {
            input.motors = devices_.motors->Read();
            const auto remote = devices_.remote->Read();
            input.remote = remote.remote;
            input.remote_disable_pose = remote.disable_pose;
            input.imu = devices_.imu->Read();
        }
        ApplyMotionCommand(input, command, options_.commands, commands_, now,
                           config_);
        disabled_observed |= input.remote_disable_pose;
        const auto tick_start = wbr::realtime::MonotonicTimeNs();
        const bool observe = tick_start >= next_telemetry;
        const auto output =
            controller_.Tick(input, tick_start / 1000,
                             options_.enable_output && disabled_observed,
                             observe ? &sample.input : nullptr);
        const auto encoded = encoder_.Encode(output.motor_targets);
        if (!encoded) {
            throw std::runtime_error("Invalid motor control targets");
        }
        if (devices_.transmitter &&
            !devices_.transmitter->SubmitCan(
                wbr::host::CanProducer::kChassis,
                {encoded->frames.data(), encoded->count}, ++command_sequence,
                tick_start)) {
            throw std::runtime_error("CAN submission rejected: " +
                                     devices_.transmitter->error());
        }
        if (observe) {
            sample.monotonic_ns = tick_start;
            sample.control_duration_ns =
                wbr::realtime::MonotonicTimeNs() - tick_start;
            sample.sequence = ++telemetry_sequence;
            sample.missed_releases = late_cycles;
            sample.queue_drops = telemetry_drops;
            sample.state = controller_.state();
            sample.simulated = !devices_.motors;
            if (telemetry_ && !telemetry_->Push(sample)) {
                ++telemetry_drops;
            }
            observations_.Push(sample);
            // 遥测频率为 100 Hz；跳过延迟样本，不补发积压周期。
            next_telemetry +=
                ((tick_start - next_telemetry) / 10000000 + 1) * 10000000;
        }
    }
    std::cout << "Stopped; missed control releases=" << late_cycles
              << std::endl;
}
}  // namespace dust::chassis
