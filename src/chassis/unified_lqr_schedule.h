/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
namespace dust_control::modules {

/**
 * @brief 计算四路输出、十维状态的完整 LQR 增益矩阵。
 *
 * 状态顺序为位置、速度、yaw、yaw 角速度、左右腿相对角
 * theta = alpha - pitch 及其角速度、机体 pitch 和 pitch 角速度。
 * 输出顺序为左轮、右轮、左虚拟腿力矩、右虚拟腿力矩。
 */
void EvaluateUnifiedLqrGain(double leg_length, double gain[4][10]);

}  // namespace dust_control::modules
