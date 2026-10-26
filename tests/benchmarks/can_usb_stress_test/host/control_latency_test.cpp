/* SPDX-License-Identifier: Apache-2.0 */
#include "control_latency.hpp"

#include <iostream>
#define CHECK(c)                                      \
    do {                                              \
        if (!(c)) {                                   \
            std::cerr << "line " << __LINE__ << '\n'; \
            return 1;                                 \
        }                                             \
    } while (false)
int main() {
    using namespace dust_control::benchmarks::bridge_wire;
    // 与固件一致使用静态存储，避免将记录器放在小容量栈上。
    static ControlLatencyRecorder recorder;
    auto command = StreamCommand(0, 0, 0);
    WriteU32(command.data(), kControlCommand);
    WriteU32(command.data() + 12, 7);
    for (unsigned i = 0; i < 3; ++i) {
        WriteU32(command.data() + FrameOffset(i) + 12, 0xffffffe0U);
    }

    recorder.Stage(command, 0, 0xfffffff0U);
    for (unsigned i = 0; i < 3; ++i) {
        recorder.Driver(i, 0xfffffff8U);
    }

    // 旧命令在途时暂存新命令，其元数据不能改变旧命令的完成时间和首个后续反馈。
    auto newer = command;
    WriteU32(newer.data() + 4, 2);  // 不采样。
    recorder.Stage(newer, 0, 1);
    // 即使元数据已更新，触发时间仍属于在途的采样组。
    for (unsigned i = 0; i < 3; ++i) {
        recorder.HardwareStart(i, 0);  // 时钟环绕后的零时间戳有效。
    }

    auto hardware = recorder.QueryHardware(0);
    CHECK(ValidHardwareReply(hardware, 0));
    CHECK(ReadU32(hardware.data() + 8) == 7);
    CHECK(ReadU32(hardware.data() + 16) == 0);
    CHECK(uint32_t(ReadU32(hardware.data() + 16) - 0xfffffff8U) == 8);
    CHECK(ReadU32(recorder.QueryHardware(2).data() + 8) == kControlUnavailable);
    for (unsigned i = 0; i < 3; ++i) {
        recorder.BatchDone(i, 8);
    }

    recorder.Receive(0, 7);  // 时间戳早于完成时间，应拒绝。
    CHECK(!HasControlTime(recorder.Query(0), 0, ControlPoint::NextFeedback));
    for (unsigned i = 0; i < 3; ++i) {
        recorder.Driver(i, 10);  // 未采样的在途传输。
        recorder.BatchDone(i, 20);
        recorder.Receive(i, 24);
        recorder.Receive(i, 30);  // 保留首个反馈，不能覆盖为最新反馈。
    }

    auto reply = recorder.Query(0);
    CHECK(ValidControlReply(reply, 0));
    CHECK(!ValidControlReply(reply, 1));
    for (unsigned i = 0; i < 3; ++i) {
        CHECK(HasControlTime(reply, i, ControlPoint::Origin));
        CHECK(HasControlTime(reply, i, ControlPoint::Driver));
        CHECK(HasControlTime(reply, i, ControlPoint::BatchDone));
        CHECK(HasControlTime(reply, i, ControlPoint::NextFeedback));
        CHECK(ControlTime(reply, i, ControlPoint::BatchDone) == 8);
        CHECK(ControlTime(reply, i, ControlPoint::NextFeedback) == 24);
        CHECK(uint32_t(ControlTime(reply, i, ControlPoint::BatchDone) -
                       ControlTime(reply, i, ControlPoint::Origin)) == 40);
    }

    CHECK(ReadU32(recorder.Query(2).data() + 8) == kControlUnavailable);
    CHECK(ReadU32(recorder.Query(64 * kTimingCapacity).data() + 8) ==
          kControlUnavailable);
    // 另一条总线独立记录，零时间戳也有效。
    WriteU32(command.data() + 4, 1);
    WriteU32(command.data() + 12, 1);
    WriteU32(command.data() + FrameOffset(0) + 12, 0);
    recorder.Stage(command, 1, 50);
    recorder.Driver(3, 60);
    recorder.HardwareStart(3, 70);
    CHECK(ReadU32(recorder.QueryHardware(1).data() + 8) == 1);
    CHECK(ReadU32(recorder.QueryHardware(1).data() + 16) == 70);
    CHECK(ReadU32(recorder.QueryHardware(0).data() + 8) == 7);
    recorder.BatchDone(3, 90);
    auto right = recorder.Query(1);
    CHECK(HasControlTime(right, 0, ControlPoint::Origin));
    CHECK(ControlTime(right, 0, ControlPoint::Origin) == 0);
    CHECK(!HasControlTime(right, 1,
                          ControlPoint::Origin));  // 预热阶段不能伪造状态。
    CHECK(!HasControlTime(
        right, 0, ControlPoint::NextFeedback));  // 缺失状态必须显式保留。
    CHECK(ControlTime(recorder.Query(0), 0, ControlPoint::BatchDone) == 8);
    WriteU32(right.data() + 8, 1U << 7);
    CHECK(!ValidControlReply(right, 1));
    std::cout << "In-flight identity, missing feedback, independent buses and "
                 "timestamp wrap "
                 "passed\n";
}
