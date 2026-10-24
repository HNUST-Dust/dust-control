/* SPDX-License-Identifier: Apache-2.0 */
#include <iostream>

#include "bridge_observation.hpp"
#define CHECK(condition)                              \
    do {                                              \
        if (!(condition)) {                           \
            std::cerr << "line " << __LINE__ << '\n'; \
            return 1;                                 \
        }                                             \
    } while (false)
int main() {
    using namespace bridge_wire;
    const auto request = EncodeRequest(17, 123, 0);
    const unsigned char header[] = {0x43, 0x41, 0x4e, 0x31, 17, 0,
                                    0,    0,    123,  0,    0,  0};
    for (unsigned i = 0; i < sizeof(header); ++i) {
        CHECK(request[i] == header[i]);
    }

    CHECK(request.size() == 64);
    CHECK(ReadId(request.data() + 16) == kLayout[0][0].command_id);
    CHECK(request[17] == 8 && request[16] == kLayout[0][0].bus);
    // 使用生产代码的零 MIT 命令样本，MCU 还会通过其编码器独立校验。
    const unsigned char zero[] = {0x80, 0, 0x80, 0, 0, 0, 8, 0};
    for (unsigned i = 0; i < 8; ++i) {
        CHECK(request[36 + i] == zero[i]);
    }

    FeedbackObservation observation{};
    observation.active = true;
    observation.receive_us = 0xfffffff0U;  // 覆盖单调时间戳环绕。
    WriteU32(observation.reply.data(), kReplyMagic);
    WriteU32(observation.reply.data() + 4, 17);
    const uint8_t feedback[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    Packet reply{};
    for (unsigned i = 0; i < 3; ++i) {
        const auto &motor = kLayout[0][i];
        CHECK(!observation.OnFeedback(i, 1, motor.bus, motor.feedback_id, 8,
                                      feedback));
        observation.OnTx(i, 0xffffffe0U);  // 忽略过期完成事件。
        CHECK(observation.tx_mask == ((1U << i) - 1));
        observation.OnTx(i, 0xfffffff8U);
        CHECK(observation.OnFeedback(i, 2 + i, motor.bus, motor.feedback_id, 8,
                                     feedback));
        CHECK(!observation.OnFeedback(i, 22, motor.bus, motor.feedback_id, 8,
                                      feedback));
        CHECK(ReadU32(observation.reply.data() + FrameOffset(i) + 12) ==
              18 + i);
    }

    CHECK(observation.Finish(10, 5000, reply));
    CHECK(ValidateReply(reply, 0));
    CHECK(ReadU32(reply.data() + 8) == 0);
    CHECK(!observation.Finish(11, 5000,
                              reply));  // 反馈窗口不能复用。
    reply[18] ^= 1;
    CHECK(!ValidateReply(reply, 0));
    observation = {};
    observation.active = true;
    observation.receive_us = 100;
    WriteU32(observation.reply.data(), kReplyMagic);
    for (unsigned i = 0; i < 3; ++i) {
        SetAddress(observation.reply.data() + FrameOffset(i), kLayout[1][i].bus,
                   kLayout[1][i].feedback_id, 0);
    }

    CHECK(!observation.Finish(5099, 5000, reply));
    CHECK(observation.Finish(5100, 5000, reply));
    CHECK(ReadU32(reply.data() + 8) == 7 && ValidateReply(reply, 1));
    WriteU32(reply.data() + 8, kBusy);
    CHECK(ValidateReply(reply, 1));
    WriteU32(reply.data() + 8, 1U << 31);
    CHECK(!ValidateReply(reply, 1));
    CHECK(!ValidateReply(request, 0));  // USB 回显固件不属于桥接固件。
    std::cout << "CAN wire fixtures, feedback freshness, timeout and wrap "
                 "rules passed\n";
}
