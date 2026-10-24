/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <limits>

#include "chassis/command_arbiter.hpp"

int main() {
    using namespace dust::chassis;
    MotionCommandOptions options;
    options.external_commands = true;
    options.timeout_ns = 100;
    MotionCommandQueue queue;
    MotionCommand command{};
    modules::ControllerInputs input{};
    input.remote.robot_enable = true;
    input.remote.sequence = 7;
    assert(queue.Push({0.5, 0.3, 0.01, 1000}));
    ApplyMotionCommand(input, command, options, queue, 1050);
    assert(input.remote.chassis_x > 0 && input.remote.chassis_rotate < 0);
    assert(input.remote.leg_length_delta > 0);
    assert(input.remote.sequence == 7 && input.remote.robot_enable);
    // 新周期从物理快照开始，ROS 不能续期遥控 sequence。
    input = {};
    input.remote.robot_enable = true;
    ApplyMotionCommand(input, command, options, queue, 1100);
    assert(input.remote.chassis_x == 0 && input.remote.chassis_rotate == 0);
    assert(queue.Push({0.5, 0, 0, 1200}));
    input.remote.chassis_x = -0.8;
    ApplyMotionCommand(input, command, options, queue, 1201);
    assert(input.remote.chassis_x == -0.8F && command.received_ns == 0);
    input.remote.chassis_x = 0;
    ApplyMotionCommand(input, command, options, queue, 1202);
    assert(input.remote.chassis_x == 0);  // 手动释放不能恢复旧目标。
    input.remote.robot_enable = false;
    assert(queue.Push({1, 1, 0, 1300}));
    ApplyMotionCommand(input, command, options, queue, 1301);
    assert(!input.remote.robot_enable && input.remote.sequence == 0);
    assert(command.received_ns == 0);
    assert(!queue.Push({std::numeric_limits<double>::quiet_NaN(), 0, 0, 1}));
    assert(!queue.Push({0, std::numeric_limits<double>::infinity(), 0, 1}));
    for (unsigned i = 0; i < 31; ++i) {
        assert(queue.Push({double(i), 0, 0, i + 1}));
    }
    assert(!queue.Push({99, 0, 0, 1}));
    assert(queue.TakeLatest(command) && command.forward_m_s == 30);
    assert(!queue.TakeLatest(command));
    assert(queue.Push({1, 1, 1, 1400}));
    options.external_commands = false;
    input.remote.chassis_x = -0.2;
    ApplyMotionCommand(input, command, options, queue, 1401);
    assert(input.remote.chassis_x == -0.2F);
}
