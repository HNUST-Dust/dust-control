/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/hardware/imu/hi91_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现 HI91 IMU 串口协议的帧解析。
 * @details
 * 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// HI91 对外报文格式约定。

#include <stddef.h>
#include <stdint.h>

#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/byte_stream.hpp"
#include "imu_config.hpp"

namespace dust_control::protocols {

constexpr uint8_t kHi91FrameSof0 =
    0x5AU;  ///< HI91 帧起始标志的第一个固定字节。
constexpr uint8_t kHi91FrameSof1 =
    0xA5U;  ///< HI91 帧起始标志的第二个固定字节。
constexpr uint8_t kHi91DataTag = 0x91U;  ///< 标识标准姿态数据载荷的协议标签。
constexpr uint16_t kHi91DataLength =
    76U;  ///< HI91 数据帧标准载荷长度，单位为字节。
constexpr uint16_t kHi91MaxPayloadLength =
    256U;  ///< 解析器接受的最大载荷长度，单位为字节。
constexpr size_t kHi91FrameHeaderSize =
    6U;  ///< 帧头长度，包含双字节 SOF、长度和 CRC 字段，单位为字节。

/** @brief HI91 协议帧解码后的物理量。 */
struct Hi91Sample {
    uint16_t main_status;     ///< 上位机协议报告的主要工作状态。
    int8_t temperature_c;     ///< 温度，单位为摄氏度。
    float air_pressure;       ///< 气压测量值，单位为帕。
    uint32_t system_time_ms;  ///< 时间长度，单位为毫秒。
    float accel_g[3];  ///< 以标准重力加速度为单位的三轴加速度。
    float gyro_dps[3];  ///< 三轴陀螺仪角速度，单位为度每秒。
    float mag_ut[3];    ///< 三轴磁场测量值，单位为微特斯拉。
    float roll_deg;     ///< 机体横滚角，单位为度。
    float pitch_deg;    ///< 机体俯仰角，单位为度。
    float yaw_deg;      ///< 归一化航向角，单位为度。
    float quat[4];  ///< 按 `[w, x, y, z]` 排列的姿态单位四元数。
};

/**
 * @brief 校验并解码一帧 HI91 协议数据。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param strict_crc 为 `true` 时要求协议 CRC 校验通过。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；CRC 不符返回 -EILSEQ，参数/格式错误返回负 errno。
 */
int DecodeHi91Frame(const uint8_t *data, size_t len, bool strict_crc,
                    Hi91Sample *out);

/**
 * @brief 计算 HI91 帧头与载荷的协议校验值。
 * @param payload_length 协议载荷长度，单位为字节。
 * @param[in] payload 协议载荷字节缓冲区。
 * @return 按 HI91 帧格式计算的 16 位 CRC。
 */
uint16_t CalculateHi91FrameCrc(uint16_t payload_length, const uint8_t *payload);

/* HiPNUC CRC-16/CCITT update with the manual's initial value 0x0000. */
/**
 * @brief 计算 CRC-16/CCITT-FALSE 校验值。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param size 输入数据的字节数。
 * @return 从初值 0x0000 开始迭代得到的 CRC-16 值。
 */
uint16_t Hi91Crc16CcittFalse(const uint8_t *data, size_t size);

}  // namespace dust_control::protocols

namespace dust_control::devices {
// HI91 发布的控制用物理量样本，时间戳使用主机单调时钟。
struct ImuSample {
    uint32_t sequence = 0U;
    uint64_t precise_timestamp_us = 0U;
    bool valid = false;
    float pitch_deg = 0.0F;
    float roll_deg = 0.0F;
    float yaw_deg = 0.0F;
    float pitch_rate_rad_s = 0.0F;
    float roll_rate_rad_s = 0.0F;
    float yaw_rate_rad_s = 0.0F;
    float accel_g[3] = {};
    float vertical_accel_g = 0.0F;
};

// 确认传感器进入新的时间周期，避免把单帧乱序误判为重启。
class Hi91TimeGuard {
   public:
    void Reset() {
        *this = {};
    }

    void Reject() {
        candidates_ = 0;
    }

    bool Accept(uint32_t device_ms, uint64_t sample_us, uint64_t timeout_us,
                bool crc_valid) {
        if (!has_time_ || int32_t(device_ms - accepted_ms_) > 0) {
            Commit(device_ms, sample_us);
            return true;
        }
        if (!crc_valid || device_ms == accepted_ms_ ||
            sample_us <= accepted_us_ ||
            sample_us - accepted_us_ <= timeout_us) {
            Reject();
            return false;
        }
        const bool consecutive =
            candidates_ && sample_us >= candidate_us_ &&
            sample_us - candidate_us_ <= kMaximumCandidateGapUs &&
            int32_t(device_ms - candidate_ms_) > 0 &&
            uint32_t(device_ms - candidate_ms_) <= kMaximumDeviceStepMs;
        if (!consecutive) {
            candidates_ = 1;
            first_candidate_us_ = sample_us;
        } else if (candidates_ < kRecoveryFrameCount) {
            ++candidates_;
        }
        candidate_ms_ = device_ms;
        candidate_us_ = sample_us;
        // 必须经过实际观测时间，不能将一次到达的三个缓冲帧当作恢复依据。
        if (candidates_ < kRecoveryFrameCount ||
            sample_us - first_candidate_us_ < kMinimumRecoverySpanUs) {
            return false;
        }
        Commit(device_ms, sample_us);
        return true;
    }

   private:
    static constexpr unsigned kRecoveryFrameCount = 3;
    static constexpr uint64_t kMinimumRecoverySpanUs = 2000;
    static constexpr uint64_t kMaximumCandidateGapUs = 20000;
    static constexpr uint32_t kMaximumDeviceStepMs = 20;
    void Commit(uint32_t device_ms, uint64_t sample_us) {
        has_time_ = true;
        accepted_ms_ = device_ms;
        accepted_us_ = sample_us;
        Reject();
    }

    bool has_time_ = false;
    uint32_t accepted_ms_ = 0;
    uint32_t candidate_ms_ = 0;
    unsigned candidates_ = 0;
    uint64_t accepted_us_ = 0;
    uint64_t candidate_us_ = 0;
    uint64_t first_candidate_us_ = 0;
};

// Read 可跨线程调用；拼帧、CRC 和时钟恢复状态只由接收线程访问。
class Hi91Decoder {
   public:
    explicit Hi91Decoder(bool strict_crc = false, ImuConfig config = {})
        : config_(config), strict_crc_(strict_crc) {}
    void Reset();
    void Discontinuity();
    void Consume(std::span<const uint8_t> bytes, uint64_t sample_us);
    ImuSample Read() const {
        return published_.Read();
    }

   private:
    const ImuConfig config_;
    void DecodeFrames(uint64_t sample_us);
    ByteStream<512> stream_;
    ImuSample latest_;
    concurrency::PublishedSnapshot<ImuSample> published_;
    Hi91TimeGuard time_guard_;
    uint32_t sequence_ = 0;
    bool strict_crc_;
};
}  // namespace dust_control::devices
