/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file src/hardware/motors/dji_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现大疆电机 CAN 协议编解码。
 * @details
 * 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// DJI 电机对外报文格式约定。

#include <stdint.h>

namespace protocols {

/** @brief 大疆电机标准反馈帧的解码结果。 */
struct DjiMotorFeedback {
    uint16_t encoder;     ///< 电机编码器原始计数值。
    int16_t omega;        ///< 电机协议反馈的原始角速度。
    int16_t current;      ///< 电机协议反馈或控制的原始电流值。
    uint8_t temperature;  ///< 电机内部温度，单位为摄氏度。
};

/**
 * @brief 判断 CAN 标识符是否为大疆电机标准反馈 ID。
 * @param can_id 11 位标准 CAN 标识符。
 * @return ID 位于 0x201..0x204 时返回 `true`。
 */
bool IsDjiStandardFeedbackId(uint16_t can_id);
/**
 * @brief 解码大疆电机标准反馈帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param dlc 输入的有效字节数；至少 7 字节，正常 CAN 数据帧为 8 字节。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int DecodeDjiFeedback(const uint8_t *data, uint8_t dlc, DjiMotorFeedback *out);

// 将四个电机电流打包为 CAN ID 0x200 报文（对应 0x201~0x204）。
/**
 * @brief 编码 CAN ID 0x200 的四路电流指令。
 * @param[in] current_cmd 目标电流指令，单位由对应电机协议规定。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int EncodeDjiCurrentFrame0x200(const int16_t current_cmd[4], uint8_t out[8]);
// 将四个电机电流打包为 CAN ID 0x1FF 报文。
/**
 * @brief 编码 CAN ID 0x1FF 的四路电流指令。
 * @param[in] current_cmd 目标电流指令，单位由对应电机协议规定。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int EncodeDjiCurrentFrame0x1ff(const int16_t current_cmd[4], uint8_t out[8]);

// 将单个电机电流命令写入 0x200/0x1FF 报文的对应槽位。
/**
 * @brief 将单个大疆电机电流指令写入对应载荷槽位。
 * @param motor_can_id 目标电机的标准 CAN 标识符。
 * @param current_cmd 目标电流指令，单位由对应电机协议规定。
 * @param frame_payload 待写入的 8 字节 CAN 载荷。
 * @return 成功返回 0；参数或帧格式无效时返回负 errno。
 */
int WriteDjiCurrentCommandToSlot(uint16_t motor_can_id, int16_t current_cmd,
                                 uint8_t frame_payload[8]);

}  // namespace protocols
