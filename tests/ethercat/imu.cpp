/* SPDX-License-Identifier: Apache-2.0 */
#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <limits>

#include "hardware/ethercat/device_pdo.hpp"
#include "hardware/ethercat/session.hpp"
#include "hardware/ethercat/wire.hpp"

namespace ec = dust_control::ethercat;
namespace host = dust_control::host;
namespace devices = dust_control::devices;
namespace dw = dust_control::ethercat::device;
namespace ds = dust_control::ethercat::sensor;
namespace w = dust_control::ethercat::wire;

namespace {
void PutFloat(uint8_t* bytes, float value) {
    w::Put32(bytes, std::bit_cast<uint32_t>(value));
}
void SameSample(const devices::ImuSample& a, const devices::ImuSample& b) {
    assert(a.valid == b.valid && a.sequence == b.sequence);
    assert(a.precise_timestamp_us == b.precise_timestamp_us);
    assert(a.pitch_deg == b.pitch_deg && a.roll_deg == b.roll_deg);
    assert(a.yaw_deg == b.yaw_deg);
    assert(a.pitch_rate_rad_s == b.pitch_rate_rad_s);
    assert(a.roll_rate_rad_s == b.roll_rate_rad_s);
    assert(a.yaw_rate_rad_s == b.yaw_rate_rad_s);
    assert(a.vertical_accel_g == b.vertical_accel_g);
    for (size_t i = 0; i < 3; ++i) {
        assert(a.accel_g[i] == b.accel_g[i]);
    }
}
void MeasurementConversion() {
    devices::Hi91Measurement decoded{};
    decoded.accel_g[0] = -1.25F;
    decoded.accel_g[1] = -0.25F;
    decoded.accel_g[2] = 0.75F;
    decoded.gyro_dps[0] = -15;
    decoded.gyro_dps[1] = -5;
    decoded.gyro_dps[2] = 5;
    decoded.roll_deg = -12;
    decoded.pitch_deg = 23;
    decoded.yaw_deg = 45;
    devices::Hi91Imu direct;
    assert(direct.Consume(decoded, 1000));
    const auto converted = direct.Read();
    assert(converted.valid && converted.pitch_deg == 23);
    assert(converted.roll_deg == -12 && converted.yaw_deg == 45);
    assert(std::abs(converted.pitch_rate_rad_s + 0.2617994F) < 0.000001F);
    assert(std::abs(converted.roll_rate_rad_s + 0.08726646F) < 0.000001F);
    assert(std::abs(converted.yaw_rate_rad_s - 0.08726646F) < 0.000001F);
    assert(converted.vertical_accel_g == 0.75F);
    assert(direct.Consume(decoded, 2000));
    assert(direct.Read().sequence == 2);
    const auto valid = direct.Read();
    for (size_t field = 0; field < 9; ++field) {
        auto invalid = decoded;
        float* values[] = {
            &invalid.accel_g[0],  &invalid.accel_g[1],  &invalid.accel_g[2],
            &invalid.gyro_dps[0], &invalid.gyro_dps[1], &invalid.gyro_dps[2],
            &invalid.roll_deg,    &invalid.pitch_deg,   &invalid.yaw_deg};
        *values[field] = std::numeric_limits<float>::quiet_NaN();
        assert(!direct.Consume(invalid, 3000));
        SameSample(valid, direct.Read());
    }
    decoded.pitch_deg = 101;
    assert(!direct.Consume(decoded, 3000));
    SameSample(valid, direct.Read());
    direct.Reset();
    assert(!direct.Read().sequence && !direct.Read().valid);
}
void CompactSession() {
    const ec::AdapterPdoConfig capacity;
    ec::PdoConfig config;
    config.cycle_time_us = 125;
    config.adapters["base"] = capacity;
    const auto layout = ec::BuildPdoLayout(config).adapters.at("base");
    host::EthercatSession session(layout);
    std::array<uint8_t, 180> input{};
    dw::Payload token{};
    ds::Put32(token.data(), 77);
    token[4] = 2;
    const auto status = dw::Encode(dw::Type::Status, 1, token);
    std::copy(status.begin(), status.end(), input.begin());
    assert(session.Receive(input, 1000000, 77));
    input.fill(0);
    std::array<uint8_t, 16> onboard{};
    const std::array<int16_t, 6> axes{4096,  -4096,  -32768,
                                      16384, -16384, 32767};
    for (size_t i = 0; i < axes.size(); ++i) {
        const auto value = uint16_t(axes[i]);
        onboard[i * 2] = uint8_t(value >> 8);
        onboard[i * 2 + 1] = uint8_t(value);
    }
    PutFloat(onboard.data() + 12, 25.5F);
    ds::Page page{};
    assert(ds::Encode(ds::Kind::ImuCompact, 1, 10, false, onboard.data(),
                      onboard.size(), page));
    std::copy_n(page.begin(), 28, input.begin() + 60);
    auto* hi91 = input.data() + 120;
    hi91[0] = ec::pdo::kHi91PageType;
    w::Put32(hi91 + 1, UINT32_MAX);
    auto* fields = hi91 + ec::pdo::kHi91FieldsOffset;
    PutFloat(fields, 1.0F);
    PutFloat(fields + 12, 90.0F);
    PutFloat(fields + 24, -3.0F);
    PutFloat(fields + 28, 4.0F);
    PutFloat(fields + 32, 5.0F);
    assert(session.Receive(input, 2000000, 77));
    const auto sample = session.hi91_imu().Read();
    assert(sample.valid && sample.pitch_deg == 4 && sample.roll_deg == -3);
    assert(std::abs(sample.pitch_rate_rad_s - 1.5707963F) < 0.00001F);
    const auto board = session.onboard_imu().Read();
    assert(board.temperature_c == 25.5F && board.sample_us == 2000);
    for (size_t i = 0; i < 3; ++i) {
        assert(board.accel_raw[i] == axes[i]);
        assert(board.gyro_raw[i] == axes[i + 3]);
    }
    assert(session.Receive(input, 3000000, 77));
    SameSample(sample, session.hi91_imu().Read());
    assert(session.onboard_imu().Read().sample_us == 2000);
    w::Put32(hi91 + 1, 0);  // 网关序号回绕，不是设备时钟复位。
    assert(session.Receive(input, 4000000, 77));
    assert(session.hi91_imu().Read().sequence == sample.sequence + 1);
    const auto wrapped = session.hi91_imu().Read();
    PutFloat(fields + 28, std::numeric_limits<float>::infinity());
    w::Put32(hi91 + 1, 1);
    assert(session.Receive(input, 5000000, 77));
    SameSample(wrapped, session.hi91_imu().Read());
    PutFloat(fields + 28, 6);
    w::Put32(hi91 + 1, 2);
    assert(session.Receive(input, 6000000, 77));
    assert(session.hi91_imu().Read().pitch_deg == 6);
    w::Put32(hi91 + 1, 1);
    assert(!session.Receive(input, 7000000, 77));
}
}  // namespace
int main() {
    MeasurementConversion();
    CompactSession();
}
