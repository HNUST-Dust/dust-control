/* SPDX-License-Identifier: Apache-2.0 */
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <thread>
#include <vector>

#include "concurrency/published_snapshot.hpp"
#include "hardware/imu/onboard_imu.hpp"
#include "hardware/motors/motor_feedback.hpp"
#include "hardware/motors/motor_map.h"
#include "hardware/remote/sbus_decoder.hpp"

namespace {
struct Sample {
    uint64_t sequence = 0;
    std::array<uint64_t, 64> values{};
};
}  // namespace

int main() {
    // 多读取者必须得到完整版本；大样本用于暴露逐字段复制的撕裂。
    dust::concurrency::PublishedSnapshot<Sample> published;
    std::atomic<bool> start{false};
    std::atomic<bool> done{false};
    std::atomic<unsigned> reads{0};
    std::vector<std::thread> readers;
    for (unsigned i = 0; i < 3; ++i) {
        readers.emplace_back([&] {
            while (!start.load()) {
                std::this_thread::yield();
            }
            do {
                const auto sample = published.Read();
                for (const auto value : sample.values) {
                    assert(value == sample.sequence);
                }
                ++reads;
            } while (!done.load());
        });
    }
    start = true;
    for (uint64_t sequence = 1; sequence <= 30000; ++sequence) {
        Sample sample;
        sample.sequence = sequence;
        sample.values.fill(sequence);
        published.Publish(sample);
    }
    done = true;
    for (auto& reader : readers) {
        reader.join();
    }
    assert(reads > 0 && published.Read().sequence == 30000);

    // 真正的设备解析线程与控制读取线程并发，Read 不能暴露解析状态。
    dust::devices::MotorFeedbackDecoder motor;
    dust::devices::OnboardImu onboard;
    dust::devices::SbusDecoder remote;
    const auto& address = platform::motor_map::kDefaultMotors[0];
    done = false;
    std::thread control([&] {
        do {
            const auto feedback = motor.Read()[0];
            if (feedback.valid) {
                assert(feedback.speed_rpm == 123);
                assert(feedback.precise_timestamp_us > 0);
            }
            const auto imu = onboard.Read();
            if (imu.sequence) {
                assert(imu.sample_us == imu.sequence);
                assert(imu.temperature_c == float(imu.sequence));
                for (const auto byte : imu.raw) {
                    assert(byte == uint8_t(imu.sequence));
                }
            }
            const auto sample = remote.Read();
            if (sample.remote.sequence) {
                assert(sample.remote.robot_enable != sample.disable_pose);
            }
        } while (!done.load());
    });
    for (uint32_t sequence = 1; sequence <= 10000; ++sequence) {
        const std::array<uint8_t, 8> can{0, 0, 0, 123, 0, 0, 0, 0};
        motor.Consume(address.bus, address.feedback_id, can, sequence, true);
        std::array<uint8_t, 25> raw{};
        raw.fill(uint8_t(sequence));
        onboard.Consume(raw, float(sequence), sequence, sequence);
        std::array<uint8_t, 25> sbus{};
        sbus[0] = 15;
        sbus[7] = sequence % 2 ? 0x40 : 0;
        remote.Consume(sbus, sequence);
        if (sequence % 100 == 0) {
            motor.Reset();
            onboard.Reset();
            remote.Reset();
        }
    }
    done = true;
    control.join();
    auto copy = onboard.Read();
    copy.sequence = 123;
    assert(onboard.Read().sequence == 0);
}
