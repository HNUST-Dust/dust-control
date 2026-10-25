/* SPDX-License-Identifier: Apache-2.0 */
// 仅供测试的 libusb 替代后端，不能打开任何真实设备。
#include <libusb.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "bridge_profile.hpp"
#include "merged_control.hpp"
struct libusb_context {};
struct libusb_device {};
struct libusb_device_handle {};
namespace {
using dust_control::benchmarks::bridge_wire::Packet;
struct Mock {
    std::mutex queue_mutex, event_mutex;
    std::condition_variable ready;
    std::deque<libusb_transfer *> inputs;
    std::deque<libusb_transfer *> outputs;
    std::deque<libusb_transfer *> cancelled;
    std::deque<Packet> replies;
    std::unique_ptr<
        dust_control::benchmarks::bridge_wire::ControlLatencyRecorder>
        recorder;
    uint32_t commands[2]{}, feedback_sequence = 0;
    static uint32_t Now() {
        return uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
    }

    void Command(const Packet &request, unsigned group, bool merged,
                 uint32_t out_us) {
        recorder->Stage(request, group, out_us);
        Packet ack{};
        dust_control::benchmarks::bridge_wire::WriteU32(
            ack.data(), dust_control::benchmarks::bridge_wire::kStreamAck);
        dust_control::benchmarks::bridge_wire::WriteU32(
            ack.data() + 4,
            dust_control::benchmarks::bridge_wire::ReadU32(request.data() + 4));
        dust_control::benchmarks::bridge_wire::WriteU32(ack.data() + 12,
                                                        out_us);
        dust_control::benchmarks::bridge_wire::WriteU32(ack.data() + 20, 2);
        dust_control::benchmarks::bridge_wire::WriteU32(
            ack.data() + 24,
            dust_control::benchmarks::bridge_wire::kControlReply);
        if (merged) {
            dust_control::benchmarks::bridge_wire::WriteU32(
                ack.data() + 28, dust_control::benchmarks::bridge_wire::
                                     kMergedControlCapability);
        }
        replies.push_back(ack);
        Packet feedback{};
        dust_control::benchmarks::bridge_wire::WriteU32(
            feedback.data(),
            dust_control::benchmarks::bridge_wire::kStreamFeedback);
        dust_control::benchmarks::bridge_wire::WriteU32(feedback.data() + 4,
                                                        feedback_sequence++);
        dust_control::benchmarks::bridge_wire::WriteU32(feedback.data() + 8, 3);
        dust_control::benchmarks::bridge_wire::WriteU32(feedback.data() + 12,
                                                        out_us + 4);
        for (unsigned i = 0; i < 3; ++i) {
            recorder->Driver(group * 3 + i, out_us + 1);
            recorder->HardwareStart(group * 3 + i, out_us + 1);
            recorder->BatchDone(group * 3 + i, out_us + 2);
            recorder->Receive(group * 3 + i, out_us + 3);
            const auto &layout =
                dust_control::benchmarks::bridge_wire::kLayout[group][i];
            auto *frame = feedback.data() +
                          dust_control::benchmarks::bridge_wire::FrameOffset(i);
            dust_control::benchmarks::bridge_wire::SetAddress(
                frame, layout.bus, layout.feedback_id, 8);
            dust_control::benchmarks::bridge_wire::WriteU32(frame + 12,
                                                            out_us + 3);
        }
        replies.push_back(feedback);
        ++commands[group];
    }

    void Process(libusb_transfer *transfer) {
        const uint32_t receive_us = Now();
        const int length = transfer->length;
        Packet request{};
        std::copy_n(transfer->buffer, 64, request.begin());
        const auto magic =
            dust_control::benchmarks::bridge_wire::ReadU32(request.data());
        if (magic ==
            dust_control::benchmarks::bridge_wire::kMergedCapabilityQuery) {
            Packet response{};
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data(),
                dust_control::benchmarks::bridge_wire::kMergedCapabilityReply);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 4,
                dust_control::benchmarks::bridge_wire::ReadU32(request.data() +
                                                               4));
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 8, dust_control::benchmarks::bridge_wire::
                                         kMergedControlCapability);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 12,
                std::getenv("RMCS_MOCK_CAPACITY_64") ? 64 : 128);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 16, 64);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 24,
                dust_control::benchmarks::bridge_wire::kProfileReply);
            if (!std::getenv("RMCS_MOCK_NO_HARDWARE")) {
                dust_control::benchmarks::bridge_wire::WriteU32(
                    response.data() + 20,
                    dust_control::benchmarks::bridge_wire::kHardwareReply);
            }
            replies.push_back(response);
        } else if (magic ==
                   dust_control::benchmarks::bridge_wire::kProfileQuery) {
            Packet response = request;
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data(),
                dust_control::benchmarks::bridge_wire::kProfileReply);
            const auto order = dust_control::benchmarks::bridge_wire::ReadU32(
                request.data() + 8);
            const auto fast = dust_control::benchmarks::bridge_wire::ReadU32(
                request.data() + 12);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 16,
                (!dust_control::benchmarks::bridge_wire::ValidProfile(order,
                                                                      fast) ||
                 commands[0] || commands[1])
                    ? 1
                    : 0);
            replies.push_back(response);
        } else if (magic ==
                   dust_control::benchmarks::bridge_wire::kControlCommand) {
            if (length == 128) {
                if (std::getenv("RMCS_MOCK_CAPACITY_64")) {
                    std::abort();
                }
                dust_control::benchmarks::bridge_wire::MergedControl raw{};
                std::copy_n(transfer->buffer, 128, raw.begin());
                Packet groups[2]{};
                if (!dust_control::benchmarks::bridge_wire::SplitMergedControl(
                        raw, groups)) {
                    std::abort();
                }
                Command(groups[0], 0, true, receive_us);
                Command(groups[1], 1, true, receive_us);
            } else {
                const auto bus = request[16];
                Command(
                    request,
                    bus == dust_control::benchmarks::bridge_wire::kLayout[0][0]
                                .bus
                        ? 0
                        : 1,
                    false, receive_us);
            }
        } else if (magic ==
                   dust_control::benchmarks::bridge_wire::kHardwareQuery) {
            if (std::getenv("RMCS_MOCK_NO_HARDWARE")) {
                std::abort();
            }
            replies.push_back(recorder->QueryHardware(
                dust_control::benchmarks::bridge_wire::ReadU32(request.data() +
                                                               4)));
        } else if (magic ==
                   dust_control::benchmarks::bridge_wire::kControlQuery) {
            replies.push_back(
                recorder->Query(dust_control::benchmarks::bridge_wire::ReadU32(
                    request.data() + 4)));
        } else if (magic ==
                   dust_control::benchmarks::bridge_wire::kStreamStatsQuery) {
            const auto page = dust_control::benchmarks::bridge_wire::ReadU32(
                request.data() + 8);
            Packet response{};
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data(),
                dust_control::benchmarks::bridge_wire::kStreamStatsReply);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 4,
                dust_control::benchmarks::bridge_wire::ReadU32(request.data() +
                                                               4));
            dust_control::benchmarks::bridge_wire::WriteU32(response.data() + 8,
                                                            page);
            dust_control::benchmarks::bridge_wire::WriteU32(
                response.data() + 12, commands[0] * 1000);
            if (page < 2) {
                for (unsigned i = 0; i < 3; ++i) {
                    dust_control::benchmarks::bridge_wire::WriteU32(
                        response.data() + 16 + i * 16, commands[page]);
                    dust_control::benchmarks::bridge_wire::WriteU32(
                        response.data() + 20 + i * 16, commands[page]);
                }
            } else {
                dust_control::benchmarks::bridge_wire::WriteU32(
                    response.data() + 24, commands[page - 2]);
                dust_control::benchmarks::bridge_wire::WriteU32(
                    response.data() + 36, commands[page - 2]);
                dust_control::benchmarks::bridge_wire::WriteU32(
                    response.data() + 40,
                    dust_control::benchmarks::bridge_wire::kLayout[page - 2][0]
                        .bus);
            }
            replies.push_back(response);
        } else {
            std::abort();
        }
    }
} mock;
libusb_context context;
libusb_device_handle handle;
libusb_device device;
}  // namespace
extern "C" {
int LIBUSB_CALL libusb_init(libusb_context **out) {
    mock.recorder = std::make_unique<
        dust_control::benchmarks::bridge_wire::ControlLatencyRecorder>();
    mock.commands[0] = mock.commands[1] = mock.feedback_sequence = 0;
    *out = &context;
    return 0;
}

void LIBUSB_CALL libusb_exit(libusb_context *) {
    // 销毁上下文前，所有已提交传输都必须完成其回调。
    std::lock_guard<std::mutex> lock(mock.queue_mutex);
    if (!mock.inputs.empty() || !mock.outputs.empty() ||
        !mock.cancelled.empty()) {
        std::abort();
    }
}

libusb_device_handle *LIBUSB_CALL
libusb_open_device_with_vid_pid(libusb_context *, uint16_t, uint16_t) {
    return &handle;
}

libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *) {
    return &device;
}

int LIBUSB_CALL libusb_get_device_speed(libusb_device *) {
    return LIBUSB_SPEED_HIGH;
}

int LIBUSB_CALL libusb_get_configuration(libusb_device_handle *, int *value) {
    *value = 1;
    return 0;
}

int LIBUSB_CALL libusb_set_configuration(libusb_device_handle *, int) {
    return 0;
}

int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *, int) {
    return 0;
}

int LIBUSB_CALL libusb_release_interface(libusb_device_handle *, int) {
    return 0;
}

void LIBUSB_CALL libusb_close(libusb_device_handle *) {}
int LIBUSB_CALL libusb_get_active_config_descriptor(
    libusb_device *, libusb_config_descriptor **out) {
    static libusb_endpoint_descriptor endpoints[2]{};
    endpoints[0].bEndpointAddress = 0x81;
    endpoints[1].bEndpointAddress = 0x01;
    for (auto &ep : endpoints) {
        ep.bmAttributes = LIBUSB_TRANSFER_TYPE_BULK;
        ep.wMaxPacketSize = 512;
    }

    static libusb_interface_descriptor alt{};
    alt.bNumEndpoints = 2;
    alt.endpoint = endpoints;
    static libusb_interface interface {};
    interface.altsetting = &alt;
    interface.num_altsetting = 1;
    auto *config = new libusb_config_descriptor{};
    config->bNumInterfaces = 1;
    config->interface = &interface;
    *out = config;
    return 0;
}

void LIBUSB_CALL
libusb_free_config_descriptor(libusb_config_descriptor *config) {
    delete config;
}

libusb_transfer *LIBUSB_CALL libusb_alloc_transfer(int) {
    return static_cast<libusb_transfer *>(
        std::calloc(1, sizeof(libusb_transfer)));
}

void LIBUSB_CALL libusb_free_transfer(libusb_transfer *transfer) {
    std::free(transfer);
}

void LIBUSB_CALL libusb_lock_events(libusb_context *) {
    // 关闭回归不能依赖低优先级控制线程获取事件线程的锁。
    if (std::getenv("RMCS_MOCK_FORBID_EVENT_LOCK")) {
        std::abort();
    }

    mock.event_mutex.lock();
}

void LIBUSB_CALL libusb_unlock_events(libusb_context *) {
    mock.event_mutex.unlock();
}

int LIBUSB_CALL libusb_submit_transfer(libusb_transfer *transfer) {
    std::lock_guard<std::mutex> lock(mock.queue_mutex);
    if (transfer->endpoint == 0x81) {
        mock.inputs.push_back(transfer);
    } else {
        mock.outputs.push_back(transfer);
    }

    mock.ready.notify_all();
    return 0;
}

int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer *transfer) {
    std::lock_guard<std::mutex> lock(mock.queue_mutex);
    for (auto *queue : {&mock.inputs, &mock.outputs}) {
        const auto found = std::find(queue->begin(), queue->end(), transfer);
        if (found != queue->end()) {
            queue->erase(found);
            mock.cancelled.push_back(transfer);
            mock.ready.notify_all();
            return 0;
        }
    }

    return LIBUSB_ERROR_NOT_FOUND;
}

int LIBUSB_CALL libusb_handle_events_timeout_completed(libusb_context *,
                                                       timeval *timeout,
                                                       int *) {
    {
        std::unique_lock<std::mutex> lock(mock.queue_mutex);
        mock.ready.wait_for(
            lock,
            std::chrono::seconds(timeout->tv_sec) +
                std::chrono::microseconds(timeout->tv_usec),
            [] {
                return !mock.cancelled.empty() || !mock.outputs.empty() ||
                       (!mock.inputs.empty() && !mock.replies.empty());
            });
    }

    std::lock_guard<std::mutex> event_lock(mock.event_mutex);
    libusb_transfer *transfer = nullptr;
    {
        std::lock_guard<std::mutex> lock(mock.queue_mutex);
        if (!mock.cancelled.empty()) {
            transfer = mock.cancelled.front();
            mock.cancelled.pop_front();
            transfer->status = LIBUSB_TRANSFER_CANCELLED;
            transfer->actual_length = 0;
        } else if (!mock.outputs.empty()) {
            transfer = mock.outputs.front();
            mock.outputs.pop_front();
            mock.Process(transfer);
            transfer->status = LIBUSB_TRANSFER_COMPLETED;
            transfer->actual_length = transfer->length;
        } else if (!mock.inputs.empty() && !mock.replies.empty()) {
            transfer = mock.inputs.front();
            mock.inputs.pop_front();
            std::copy(mock.replies.front().begin(), mock.replies.front().end(),
                      transfer->buffer);
            mock.replies.pop_front();
            transfer->status = LIBUSB_TRANSFER_COMPLETED;
            transfer->actual_length = 64;
        }
    }

    if (transfer) {
        transfer->callback(transfer);
    }

    return 0;
}
}  // extern C
