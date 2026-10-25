/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file src/hardware/remote/wfly_sbus_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现天地飞 SBUS 遥控协议解码。
 * @details
 * 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once
#include <stddef.h>
#include <stdint.h>

#include <array>
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/byte_stream.hpp"
#include "hardware/remote/remote_input_state.hpp"

namespace dust_control::protocols {

constexpr size_t kWflySbusFrameLength =
    25;  ///< 天地飞 SBUS 固定帧长度，单位为字节。
constexpr uint8_t kWflySbusStartByte = 0x0f;  ///< 天地飞 SBUS 帧起始字节。
constexpr uint8_t kWflySbusEndByte = 0x00;  ///< 天地飞 SBUS 帧结束字节。
constexpr size_t kWflySbusChannelCount = 16;  ///< SBUS 模拟通道数量。

/** @brief 天地飞 SBUS 帧的原始通道解码结果。 */
struct WflySbusFrame {
    std::array<uint16_t, kWflySbusChannelCount> msg;  ///< 遥测或遥控通道数组。
    bool channel17;   ///< SBUS 数字通道 17 的开关状态。
    bool channel18;   ///< SBUS 数字通道 18 的开关状态。
    bool frame_lost;  ///< 接收机报告的遥控帧丢失标志。
    bool failsafe;    ///< 接收机进入失控保护状态的标志。
};

/**
 * @brief 解码一帧天地飞 SBUS 数据。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 操作成功或条件成立时返回 `true`，否则返回 `false`。
 */
bool DecodeWflySbusFrame(const uint8_t *data, size_t len, WflySbusFrame *out);

}  // namespace dust_control::protocols

namespace dust_control::devices {
struct RemoteSample {
    dust_control::input::RemoteInputState remote{};
    bool disable_pose = false;
};

// Read 可跨线程调用；其余操作由接收线程独占，输入字节仅在调用期间借用。
class WflySbusDecoder {
   public:
    void Reset();
    void Discontinuity();
    void Consume(std::span<const uint8_t> bytes, uint64_t receive_us);
    bool PollTimeout(uint64_t now_us);
    bool timeout_pending() const {
        return lost_active_ && !lost_disabled_;
    }
    RemoteSample Read() const {
        return published_.Read();
    }

   private:
    void DecodeFrame(uint64_t receive_us);
    ByteStream<512> stream_;
    RemoteSample latest_;
    concurrency::PublishedSnapshot<RemoteSample> published_;
    uint32_t sequence_ = 0;
    bool lost_active_ = false;
    bool lost_disabled_ = false;
    uint64_t lost_start_us_ = 0;
};
}  // namespace dust_control::devices
