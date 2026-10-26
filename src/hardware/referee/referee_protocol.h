/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/hardware/referee/referee_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现机器人裁判系统协议的帧解析。
 * @details
 * 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// RoboMaster 裁判系统对外报文格式约定。

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <span>

#include "concurrency/published_snapshot.hpp"
#include "hardware/byte_stream.hpp"

namespace dust_control::protocols {

constexpr uint8_t kRefereeSof = 0xA5U;  ///< 裁判系统协议帧起始字节。

/*
 * 线上布局：
 *   字节 0      SOF (0xa5)
 *   字节 1-2    数据长度，小端序
 *   字节 3      帧序号
 *   字节 4      帧头 CRC8
 *   字节 5-6    命令 ID，小端序
 *   字节 7 起   数据载荷，长度由字节 1-2 指定
 *   最后 2 字节 整帧 CRC16
 */
constexpr size_t kRefereeFrameHeaderSize =
    7U;  ///< 裁判系统帧头长度，单位为字节。
constexpr size_t kRefereeFrameTrailerSize =
    2U;  ///< 裁判系统帧尾 CRC-16 长度，单位为字节。
constexpr size_t kRefereeMaxFrameSize =
    160U;  ///< 解析器接受的裁判系统最大完整帧长度。
constexpr size_t kRefereeMaxDataLength =
    kRefereeMaxFrameSize - kRefereeFrameHeaderSize -
    kRefereeFrameTrailerSize;  ///< 单帧允许的最大数据区长度，单位为字节。

constexpr uint16_t kRefereeCmdGameStatus = 0x0001U;  ///< 比赛状态命令标识符。
constexpr uint16_t kRefereeCmdRobotStatus =
    0x0201U;  ///< 机器人状态命令标识符。
constexpr uint16_t kRefereeCmdShootData = 0x0207U;  ///< 射击事件命令标识符。

/** @brief 裁判系统比赛状态数据。 */
struct RefereeGameStatus {
    uint8_t game_type;  ///< 比赛类型，来自状态字节的低 4 位。
    uint8_t game_progress;  ///< 当前比赛阶段，来自状态字节的高 4 位。
    uint16_t stage_remain_time;  ///< 当前比赛阶段剩余时间，单位为秒。
};

/** @brief 裁判系统机器人状态数据。 */
struct RefereeRobotStatus {
    uint16_t current_hp;           ///< 机器人当前生命值。
    uint16_t max_hp;               ///< 机器人最大生命值。
    uint16_t chassis_power_limit;  ///< 裁判系统报告的底盘功率上限。
    uint8_t gimbal_power_on;  ///< 裁判系统报告的云台供电状态。
};

/** @brief 裁判系统射击事件数据。 */
struct RefereeShootData {
    uint8_t bullet_type;  ///< 裁判系统报告的弹丸类型。
    uint8_t launching_frequency;  ///< 发射机构目标或实测频率，单位为赫兹。
    float initial_speed;  ///< 线速度，单位为米每秒。
};

/** @brief 裁判系统协议帧的通用解码结果。 */
struct RefereeFrame {
    uint16_t cmd_id;  ///< 裁判系统命令标识符。
    const uint8_t *data;  ///< 借用输入帧内的载荷，不复制或延长其寿命。
    uint16_t data_len;  ///< 协议数据有效长度，单位为字节。
};

/**
 * @brief 校验帧边界并拆解载荷；当前不校验协议中的 CRC8/CRC16。
 * @param[in] frame 输入帧；out->data 借用此缓冲区，使用结果前须保持其有效。
 * @param frame_len 完整协议帧长度，单位为字节。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int DecodeRefereeFrame(const uint8_t *frame, size_t frame_len,
                       RefereeFrame *out);

// 以下接口分别解码当前支持的裁判系统命令载荷。
/**
 * @brief 解码裁判系统比赛状态载荷。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int DecodeRefereeGameStatus(const uint8_t *data, size_t len,
                            RefereeGameStatus *out);
/**
 * @brief 解码裁判系统机器人状态载荷。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int DecodeRefereeRobotStatus(const uint8_t *data, size_t len,
                             RefereeRobotStatus *out);
/**
 * @brief 解码裁判系统射击事件载荷。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int DecodeRefereeShootData(const uint8_t *data, size_t len,
                           RefereeShootData *out);

}  // namespace dust_control::protocols

namespace dust_control::devices {
struct RefereeSample {
    // 最近接收块的前 40 字节；Consume 仍解析传入的全部字节。
    std::array<uint8_t, 40> chunk{};
    uint8_t chunk_size = 0;
    uint64_t receive_us = 0;
    // 沿用现有裁判协议解析器：只校验帧边界，尚不校验 CRC8/CRC16。
    std::array<uint8_t, dust_control::protocols::kRefereeMaxDataLength>
        payload{};
    uint16_t payload_size = 0;
    uint16_t command_id = 0;
    uint64_t frame_us = 0;
    uint32_t frame_sequence = 0;
};

// Read 可跨线程调用；其余操作仅由接收线程调用。输出不参与控制许可。
class RefereeDecoder {
   public:
    void Reset();
    void Discontinuity();
    void Consume(std::span<const uint8_t> bytes, uint64_t receive_us);
    RefereeSample Read() const {
        return published_.Read();
    }

   private:
    void DecodeFrames(uint64_t receive_us);
    ByteStream<dust_control::protocols::kRefereeMaxFrameSize> stream_;
    RefereeSample latest_;
    concurrency::PublishedSnapshot<RefereeSample> published_;
};
}  // namespace dust_control::devices
