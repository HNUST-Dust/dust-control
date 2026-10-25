#include <hardware/usb/protocol/usb_session_policy.hpp>
/* SPDX-License-Identifier: Apache-2.0 */
// 仅供测试的后端，不打开硬件，也不链接到生产程序。
#include <hardware/imu/hi91_protocol.h>
#include <libusb.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <hardware/usb/bridge_protocol.hpp>
#include <mutex>
#include <thread>

#include "../support/motor_fixture.hpp"

struct libusb_context {
    void* state = nullptr;
};
struct libusb_device {
    int index = 0;
    libusb_context* context = nullptr;
};
struct libusb_device_handle {
    int index = 0;
    libusb_context* context = nullptr;
    libusb_device device;
};

namespace {
namespace wire = wbr::bridge;
// 契约失败仍立即终止，仅补充位置，便于区分时序失败和协议错误。
[[noreturn]] void MockAbort(int line) {
    std::fprintf(stderr, "USB mock contract failed at usb_mock.cpp:%d\n", line);
    std::abort();
}

struct MockState {
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<libusb_transfer*> inputs, outputs, cancelled;
    std::deque<wire::UsbInPacket> replies;
    uint32_t session = 0, command_count = 0, last_sequence = 0;
    bool ready_delivered = false, stop_received = false, disconnected = false;
    bool configured = false, armed = false;
    wire::SessionPolicy policy;
    uint32_t uart_sequence[2]{};
    uint32_t last_sbus_us = 0;
    int device_index = 0;
};
MockState& State(libusb_context* context) {
    return *static_cast<MockState*>(context->state);
}
uint32_t Now() {
    return uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
}

template <typename Message>
void Enqueue(MockState& state, const Message& message) {
    wire::UsbInPacket packet{};
    if (!wire::Encode(message, packet)) {
        MockAbort(__LINE__);
    }
    state.replies.push_back(packet);
}

void FullLoadSensors(MockState& state, const wire::CanTransmitRequest& packet) {
    if (packet.frame_count != 6) {
        MockAbort(__LINE__);
    }

    for (unsigned i = 0; i < 6; ++i) {
        const auto& frame = packet.frames[i];
        const bool wheel = frame.id == 0x200;
        for (unsigned byte = 0; byte < 8; ++byte) {
            const uint8_t expected = wheel ? 0 : (byte == 7 ? 0xfd : 0xff);
            if (frame.data[byte] != expected) {
                MockAbort(__LINE__);
            }
        }
    }

    const auto uart = [&](unsigned port, const uint8_t* data, size_t size) {
        while (size) {
            const auto count = std::min(size, size_t(40));
            wire::UartReceiveReport report{};
            const auto stamp = Now();
            report.header = {stamp, state.session};
            report.channel = port;
            report.byte_count = count;
            report.block_sequence = state.uart_sequence[port - 1]++;
            report.receive_timestamp_us = stamp - 7;
            std::copy_n(data, count, report.bytes.begin());
            Enqueue(state, report);
            data += count;
            size -= count;
        }
    };

    const uint32_t now_us = Now();
    if (!state.last_sbus_us || uint32_t(now_us - state.last_sbus_us) >= 14000) {
        state.last_sbus_us = now_us;
        uint8_t sbus[25]{};
        sbus[0] = 0x0f;
        uart(1, sbus, sizeof(sbus));
    }

    uint8_t hi91[82]{};
    hi91[0] = 0x5a;
    hi91[1] = 0xa5;
    wire::Put16(hi91 + 2, 76);
    hi91[6] = 0x91;
    wire::Put32(hi91 + 14, state.command_count);
    wire::Put16(hi91 + 4, protocols::CalculateHi91FrameCrc(76, hi91 + 6));
    uart(2, hi91, sizeof(hi91));
    if (!std::getenv("WBR_MOCK_FULL_DROP_IMU")) {
        wire::ImuSampleReport imu{};
        const auto stamp = Now();
        imu.header = {stamp, state.session};
        imu.sample_timestamp_us = stamp - 7;
        Enqueue(state, imu);
    }
}

// MCU 的传感器上报独立于控制命令，不依赖本周期是否有待发送帧。
void SplitSensors(MockState& state) {
    wire::CanReceiveReport feedback{};
    feedback.header = {Now(), state.session};
    feedback.frame_count =
        state.device_index == 1 && std::atoi(std::getenv("WBR_MOCK_SPLIT")) == 2
            ? 4
            : 2;
    for (unsigned i = 0; i < feedback.frame_count; ++i) {
        const bool wheel = state.device_index == 0;
        const auto command_id = i == 0 ? 0 : (i == 1 ? 3 : (i == 2 ? 1 : 2));
        auto& received = feedback.frames[i];
        received.frame.channel = wheel ? (i == 0 ? 3 : 2) : (i < 2 ? 3 : 2);
        received.frame.id = wheel ? 0x201 : 0x10 + command_id;
        received.frame.length = 8;
        received.frame.data[1] = static_cast<uint8_t>(state.device_index + 1);
        received.receive_timestamp_us = feedback.header.transmit_timestamp_us;
        received.bus_sequence = state.command_count;
    }
    Enqueue(state, feedback);
}

void Process(MockState& state, libusb_transfer* transfer) {
    state.device_index = transfer->dev_handle->index;
    if (transfer->length != wire::kOutBytes) {
        MockAbort(__LINE__);
    }
    wire::UsbOutRequest packet{};
    std::copy_n(transfer->buffer, packet.size(), packet.begin());
    const wire::UsbOutRequestView view(packet);
    const auto kind = view.MessageKind();
    if (kind == wire::Kind::Hello) {
        wire::HelloRequest hello{};
        if (!wire::Decode(packet, hello)) {
            MockAbort(__LINE__);
        }
        if (const auto* delayed = std::getenv("WBR_MOCK_DELAY_INDEX")) {
            if (state.device_index == std::atoi(delayed)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(35));
            }
        }
        state.session = hello.header.session_id;
        state.configured = state.armed = false;
        state.policy.Open(state.session);
        state.ready_delivered = false;
        state.stop_received = false;
        state.last_sequence = 0;
        state.replies.clear();
        wire::ReadyReport reply{};
        reply.header = {Now(), state.session};
        reply.capabilities =
            std::getenv("WBR_MOCK_BAD_CAPS")
                ? 0
                : (std::getenv("WBR_MOCK_ALL_UARTS") ? 255 : 15);
        reply.can_channel_mask =
            std::getenv("WBR_MOCK_SPLIT") && state.device_index != 0 &&
                    std::atoi(std::getenv("WBR_MOCK_SPLIT")) == 3
                ? 8
                : 12;
        reply.protection_frame_count = 6;
        reply.protection_signature = 0x12345678;
        reply.max_lease_us = wire::kCommandTimeoutUs;
        if (std::getenv("WBR_MOCK_PROFILE_MISMATCH")) {
            reply.protection_signature ^= 1;
        }
        if (std::getenv("WBR_MOCK_NO_PROFILE")) {
            reply.protection_flags = 0;
        }
        Enqueue(state, reply);
        // 故障注入在序列化后修改线上字节。
        if (std::getenv("WBR_MOCK_OLD_FIRMWARE")) {
            wire::Put32(state.replies.back().data(), 0x314b4341);
        }
        if (std::getenv("WBR_MOCK_V1_FIRMWARE")) {
            state.replies.back()[8] = 1;
        }
        if (std::getenv("WBR_MOCK_STALE_SESSION")) {
            wire::CanReceiveReport stale{};
            stale.header = {Now(), state.session ^ 0xffffffffU};
            stale.frame_count = 1;
            stale.frames[0] = {{3, 8, 0, 0x201, {}}, Now(), 0};
            Enqueue(state, stale);
        }
        return;
    }
    if (!state.ready_delivered || view.Session() != state.session) {
        MockAbort(__LINE__);
    }
    if (kind == wire::Kind::ConfigureSafety || kind == wire::Kind::Arm) {
        int status = 0;
        if (kind == wire::Kind::ConfigureSafety) {
            wire::ConfigureSafetyRequest request{};
            if (state.armed || !wire::Decode(packet, request) ||
                request.uart_stop_count != 0 ||
                request.lease_us != wire::kCommandTimeoutUs) {
                MockAbort(__LINE__);
            }
            // 零动作表示选择固件默认保护，不能清空保护基线。
            state.configured = state.policy.Configure(request.lease_us);
            state.policy.Observe(request.header.request_sequence);
            if (std::getenv("WBR_MOCK_SAFETY_REJECT")) {
                status = -22;
            }
        } else {
            wire::ArmRequest request{};
            if (!state.configured || !wire::Decode(packet, request) ||
                !state.policy.Arm(request.header.request_sequence, Now())) {
                MockAbort(__LINE__);
            }
            state.armed = true;
            const auto* rejected_index =
                std::getenv("WBR_MOCK_ARM_REJECT_INDEX");
            if (std::getenv("WBR_MOCK_ARM_REJECT") ||
                (rejected_index &&
                 state.device_index == std::atoi(rejected_index))) {
                status = -22;
                state.armed = false;
                state.policy.Close();
            }
        }
        Enqueue(state, wire::RequestResultReport{{Now(), state.session},
                                                 view.Sequence(),
                                                 kind,
                                                 state.armed,
                                                 status});
    } else if (kind == wire::Kind::CanTransmitRequest) {
        wire::CanTransmitRequest request{};
        if (!state.armed || !wire::Decode(packet, request)) {
            MockAbort(__LINE__);
        }
        // 拒绝通知到达前，已排队的 OUT 仍可能到达关闭的会话。
        if (std::getenv("WBR_MOCK_DATA_REJECT") && !state.policy.Session()) {
            return;
        }
        const auto sequence = request.header.request_sequence;
        if (!state.policy.Accept(sequence, Now()) ||
            (state.command_count &&
             !wire::NewSequence(sequence, state.last_sequence))) {
            MockAbort(__LINE__);
        }
        state.last_sequence = sequence;
        ++state.command_count;
        if (std::getenv("WBR_MOCK_DATA_REJECT")) {
            Enqueue(
                state,
                wire::RequestResultReport{
                    {Now(), state.session}, sequence, kind, state.armed, -28});
            state.policy.Close();
            return;
        }
        if (std::getenv("WBR_MOCK_FULL_LOAD")) {
            FullLoadSensors(state, request);
        }
        if (std::getenv("WBR_MOCK_UART")) {
            if (request.frame_count) {
                MockAbort(__LINE__);
            }
            for (unsigned port = 1; port <= 2; ++port) {
                wire::UartReceiveReport uart{};
                const auto stamp = Now();
                uart.header = {stamp, state.session};
                uart.channel = port;
                uart.byte_count = 8;
                uart.block_sequence =
                    state.command_count + (state.command_count >= 3 ? 1 : 0);
                uart.receive_timestamp_us = stamp - 7;
                Enqueue(state, uart);
            }
        }
        if (std::getenv("WBR_MOCK_SPLIT")) {
            for (unsigned i = 0; i < request.frame_count; ++i) {
                const auto& command = request.frames[i];
                const bool wheel = command.id == 0x200;
                const bool two_boards =
                    std::atoi(std::getenv("WBR_MOCK_SPLIT")) == 2;
                const bool joint_route = command.channel == 3 &&
                                         (command.id == 0 || command.id == 3);
                const bool right_route = two_boards && command.channel == 2 &&
                                         (command.id == 1 || command.id == 2);
                if ((state.device_index == 0 &&
                     (!wheel ||
                      (command.channel != 2 && command.channel != 3))) ||
                    (state.device_index != 0 &&
                     (wheel || (!joint_route && !right_route)))) {
                    MockAbort(__LINE__);
                }
                for (unsigned byte = 0; byte < 8; ++byte) {
                    const uint8_t expected =
                        wheel ? 0 : (byte == 7 ? 0xfd : 0xff);
                    if (command.data[byte] != expected)
                        MockAbort(__LINE__);
                }
            }
            SplitSensors(state);
        } else {
            wire::CanReceiveReport feedback{};
            feedback.header = {Now(), state.session};
            feedback.frame_count = 6;
            for (unsigned slot = 0; slot < 6; ++slot) {
                const auto& motor = dust::test::kMotorAddresses[slot];
                auto& received = feedback.frames[slot];
                received.frame.channel = motor.bus;
                received.frame.id = motor.feedback_id;
                received.frame.length = 8;
                if (slot == 2 && std::getenv("WBR_MOCK_FULL_FAULT")) {
                    received.frame.data[0] = 0xa0;
                }
                received.receive_timestamp_us =
                    feedback.header.transmit_timestamp_us;
                received.bus_sequence = state.command_count;
            }
            Enqueue(state, feedback);
        }

    } else if (kind == wire::Kind::UartTransmitRequest) {
        wire::UartTransmitRequest request{};
        if (!state.armed || !wire::Decode(packet, request) ||
            request.byte_count > 40 ||
            !state.policy.Accept(request.header.request_sequence, Now())) {
            MockAbort(__LINE__);
        }
        wire::UartReceiveReport echo{};
        echo.header = {Now(), state.session};
        echo.channel = request.channel;
        echo.byte_count = request.byte_count;
        echo.block_sequence = request.header.request_sequence;
        echo.receive_timestamp_us = echo.header.transmit_timestamp_us;
        std::copy_n(request.bytes.begin(), request.byte_count,
                    echo.bytes.begin());
        Enqueue(state, echo);
    } else if (kind == wire::Kind::Stop) {
        wire::StopRequest request{};
        if (!wire::Decode(packet, request)) {
            MockAbort(__LINE__);
        }
        state.stop_received = true;
    } else {
        MockAbort(__LINE__);
    }
}
}  // namespace
extern "C" {
int LIBUSB_CALL libusb_init(libusb_context** output) {
    *output = new libusb_context{new MockState};
    return 0;
}

void LIBUSB_CALL libusb_exit(libusb_context* context) {
    auto& state = State(context);
    std::unique_lock<std::mutex> lock(state.mutex);
    if (!state.inputs.empty() || !state.outputs.empty() ||
        !state.cancelled.empty()) {
        MockAbort(__LINE__);
    }

    if (state.command_count && !state.disconnected && !state.stop_received &&
        state.policy.Session()) {
        MockAbort(__LINE__);
    }
    lock.unlock();
    delete static_cast<MockState*>(context->state);
    delete context;
}

ssize_t LIBUSB_CALL libusb_get_device_list(libusb_context* context,
                                           libusb_device*** out) {
    const int count = std::getenv("WBR_MOCK_NO_DEVICE")
                          ? 0
                          : (std::getenv("WBR_MOCK_TRIPLE")
                                 ? 3
                                 : (std::getenv("WBR_MOCK_MULTIPLE") ? 2 : 1));
    auto** list = new libusb_device* [count + 1] {};
    for (int i = 0; i < count; ++i)
        list[i] = new libusb_device{i, context};
    *out = list;
    return count;
}
void LIBUSB_CALL libusb_free_device_list(libusb_device** list, int) {
    for (size_t i = 0; list[i]; ++i)
        delete list[i];
    delete[] list;
}
int LIBUSB_CALL libusb_get_device_descriptor(libusb_device* dev,
                                             libusb_device_descriptor* out) {
    *out = {};
    out->idVendor = 0x34b7;
    out->idProduct =
        std::getenv("WBR_MOCK_ZERO_PID")
            ? 0
            : (std::getenv("WBR_MOCK_LEGACY_PID")
                   ? 0xffff
                   : (dev->index && !std::getenv("WBR_MOCK_SAME_PID")
                          ? 0xabcd
                          : 0x1234));
    out->iManufacturer = 1;
    out->iProduct = 2;
    out->iSerialNumber = 3;
    return 0;
}

uint8_t LIBUSB_CALL libusb_get_bus_number(libusb_device*) {
    return 1;
}

uint8_t LIBUSB_CALL libusb_get_device_address(libusb_device* dev) {
    return 2 + dev->index;
}

int LIBUSB_CALL libusb_open(libusb_device* dev, libusb_device_handle** out) {
    const auto* failed = std::getenv("WBR_MOCK_FAIL_INDEX");
    if ((failed && dev->index == std::atoi(failed)) ||
        std::getenv("WBR_MOCK_NO_ACCESS") ||
        (dev->index && std::getenv("WBR_MOCK_SECOND_NO_ACCESS"))) {
        return LIBUSB_ERROR_ACCESS;
    }
    *out = new libusb_device_handle{dev->index, dev->context, *dev};
    return 0;
}

int LIBUSB_CALL libusb_get_string_descriptor_ascii(libusb_device_handle* dev,
                                                   uint8_t index,
                                                   unsigned char* out,
                                                   int capacity) {
    if (std::getenv("WBR_MOCK_STRING_FAIL")) {
        return LIBUSB_ERROR_IO;
    }

    const char* text =
        index == 1   ? (std::getenv("WBR_MOCK_OTHER_VENDOR") ? "Other" : "Dust")
        : index == 2 ? (std::getenv("WBR_MOCK_OTHER_BOARD") ? "dust-other"
                                                            : "dust-hpm6750")
        : dev->index == 2 ? "00112233445566778899AABBCCDDEEFF2"
        : dev->index == 1 ? "00112233445566778899AABBCCDDEEFF1"
                          : "00112233445566778899AABBCCDDEEFF0";
    const int length = std::min(capacity, int(std::strlen(text)));
    std::memcpy(out, text, length);
    return length;
}

libusb_device* LIBUSB_CALL libusb_get_device(libusb_device_handle* dev) {
    return &dev->device;
}

int LIBUSB_CALL libusb_get_device_speed(libusb_device*) {
    return LIBUSB_SPEED_HIGH;
}

int LIBUSB_CALL libusb_claim_interface(libusb_device_handle*, int) {
    return 0;
}

int LIBUSB_CALL libusb_release_interface(libusb_device_handle*, int) {
    return 0;
}

void LIBUSB_CALL libusb_close(libusb_device_handle* handle) {
    delete handle;
}
int LIBUSB_CALL libusb_get_active_config_descriptor(
    libusb_device*, libusb_config_descriptor** output) {
    static libusb_endpoint_descriptor endpoints[2]{};
    endpoints[0].bEndpointAddress = 1;
    endpoints[1].bEndpointAddress = 0x81;
    for (auto& endpoint : endpoints) {
        endpoint.bmAttributes = LIBUSB_TRANSFER_TYPE_BULK;
        endpoint.wMaxPacketSize = 512;
    }

    static libusb_interface_descriptor descriptor{};
    descriptor.bInterfaceClass =
        std::getenv("WBR_MOCK_BAD_INTERFACE") ? 0 : LIBUSB_CLASS_VENDOR_SPEC;
    descriptor.bNumEndpoints = 2;
    descriptor.endpoint = endpoints;
    static libusb_interface interface { &descriptor, 1 };
    auto* config = new libusb_config_descriptor{};
    config->bNumInterfaces = 1;
    config->interface = &interface;
    *output = config;
    return 0;
}

void LIBUSB_CALL
libusb_free_config_descriptor(libusb_config_descriptor* config) {
    delete config;
}

libusb_transfer* LIBUSB_CALL libusb_alloc_transfer(int) {
    return static_cast<libusb_transfer*>(
        std::calloc(1, sizeof(libusb_transfer)));
}

void LIBUSB_CALL libusb_free_transfer(libusb_transfer* transfer) {
    std::free(transfer);
}

int LIBUSB_CALL libusb_submit_transfer(libusb_transfer* transfer) {
    auto& state = State(transfer->dev_handle->context);
    std::lock_guard<std::mutex> lock(state.mutex);
    if (transfer->endpoint == 0x81 && std::getenv("WBR_MOCK_IN_FAIL")) {
        return LIBUSB_ERROR_IO;
    }

    if (transfer->endpoint == 0x01 &&
        (wire::HasHeader(transfer->buffer, transfer->length) &&
         transfer->buffer[9] == uint8_t(wire::Kind::Hello)) &&
        std::getenv("WBR_MOCK_REQUIRE_PREPOST") && state.inputs.size() != 4) {
        MockAbort(__LINE__);
    }
    (transfer->endpoint == 0x81 ? state.inputs : state.outputs)
        .push_back(transfer);
    state.changed.notify_all();
    return 0;
}

int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer* transfer) {
    auto& state = State(transfer->dev_handle->context);
    std::lock_guard<std::mutex> lock(state.mutex);
    for (auto* queue : {&state.inputs, &state.outputs}) {
        auto found = std::find(queue->begin(), queue->end(), transfer);
        if (found != queue->end()) {
            queue->erase(found);
            state.cancelled.push_back(transfer);
            state.changed.notify_all();
            return 0;
        }
    }

    return LIBUSB_ERROR_NOT_FOUND;
}

int LIBUSB_CALL libusb_handle_events_timeout_completed(libusb_context* context,
                                                       timeval*, int*) {
    auto& state = State(context);
    libusb_transfer* transfer = nullptr;
    {
        std::unique_lock<std::mutex> lock(state.mutex);
        state.changed.wait_for(
            lock,
            std::chrono::milliseconds(std::getenv("WBR_MOCK_SPLIT") ? 1 : 10),
            [&state] {
                return !state.cancelled.empty() || !state.outputs.empty() ||
                       (!state.inputs.empty() && !state.replies.empty());
            });
        if (std::getenv("WBR_MOCK_SPLIT") && state.armed &&
            !state.stop_received && !state.disconnected &&
            state.replies.empty()) {
            SplitSensors(state);
        }
        if (!state.cancelled.empty()) {
            transfer = state.cancelled.front();
            state.cancelled.pop_front();
            transfer->status = LIBUSB_TRANSFER_CANCELLED;
            transfer->actual_length = 0;
        } else if (!state.outputs.empty()) {
            transfer = state.outputs.front();
            state.outputs.pop_front();
            Process(state, transfer);
            transfer->status = LIBUSB_TRANSFER_COMPLETED;
            transfer->actual_length = transfer->length;
        } else if (!state.inputs.empty() && !state.replies.empty()) {
            transfer = state.inputs.front();
            state.inputs.pop_front();
            if ((std::getenv("WBR_MOCK_DISCONNECT") ||
                 (std::getenv("WBR_MOCK_DISCONNECT_INDEX") &&
                  state.device_index ==
                      std::atoi(std::getenv("WBR_MOCK_DISCONNECT_INDEX")))) &&
                state.command_count >= 5 && !state.disconnected) {
                state.disconnected = true;
                transfer->status = LIBUSB_TRANSFER_NO_DEVICE;
                transfer->actual_length = 0;
            } else {
                const auto packet = state.replies.front();
                state.replies.pop_front();
                std::copy(packet.begin(), packet.end(), transfer->buffer);
                if (packet[9] == uint8_t(wire::Kind::Ready)) {
                    state.ready_delivered = true;
                }
                transfer->status = LIBUSB_TRANSFER_COMPLETED;
                transfer->actual_length = int(wire::UsbReportLength(packet));
            }
        }
    }

    if (transfer) {
        transfer->callback(transfer);
    }

    return 0;
}
}
