#include <hardware/usb/protocol/usb_session_policy.hpp>
/* SPDX-License-Identifier: Apache-2.0 */
// 仅供测试的后端，不打开硬件，也不链接到生产程序。
#include <hardware/imu/hi91_protocol.h>
#include <hardware/motors/motor_map.h>
#include <libusb.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <hardware/usb/bridge_protocol.hpp>
#include <mutex>

struct libusb_context {};
struct libusb_device_handle {
    int index = 0;
};

struct libusb_device {
    int index = 0;
};

namespace {
namespace wire = wbr::bridge;
libusb_context context;
libusb_device_handle handle;
libusb_device device;
libusb_device second_device{1};
libusb_device_handle second_handle{1};
std::mutex mutex;
std::condition_variable changed;
std::deque<libusb_transfer*> inputs;
std::deque<libusb_transfer*> outputs;
std::deque<libusb_transfer*> cancelled;
std::deque<wire::UsbInPacket> replies;
uint32_t session;
uint32_t command_count;
uint32_t last_sequence;
bool ready_delivered;
bool stop_received;
bool disconnected;
bool configured;
bool armed;
wire::SessionPolicy policy;
uint32_t Now() {
    return uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
}

template <typename Message>
void Enqueue(const Message& message) {
    wire::UsbInPacket packet{};
    if (!wire::Encode(message, packet)) {
        std::abort();
    }
    replies.push_back(packet);
}

void FullLoadSensors(const wire::CanTransmitRequest& packet) {
    if (packet.frame_count != 6) {
        std::abort();
    }

    for (unsigned i = 0; i < 6; ++i) {
        const auto& frame = packet.frames[i];
        const bool wheel = frame.id == 0x200;
        for (unsigned byte = 0; byte < 8; ++byte) {
            const uint8_t expected = wheel ? 0 : (byte == 7 ? 0xfd : 0xff);
            if (frame.data[byte] != expected) {
                std::abort();
            }
        }
    }

    static uint32_t uart_sequence[2]{};
    const auto uart = [&](unsigned port, const uint8_t* data, size_t size) {
        while (size) {
            const auto count = std::min(size, size_t(40));
            wire::UartReceiveReport report{};
            const auto stamp = Now();
            report.header = {stamp, session};
            report.channel = port;
            report.byte_count = count;
            report.block_sequence = uart_sequence[port - 1]++;
            report.receive_timestamp_us = stamp - 7;
            std::copy_n(data, count, report.bytes.begin());
            Enqueue(report);
            data += count;
            size -= count;
        }
    };

    static uint32_t last_sbus_us = 0;
    const uint32_t now_us = Now();
    if (!last_sbus_us || uint32_t(now_us - last_sbus_us) >= 14000) {
        last_sbus_us = now_us;
        uint8_t sbus[25]{};
        sbus[0] = 0x0f;
        uart(1, sbus, sizeof(sbus));
    }

    uint8_t hi91[82]{};
    hi91[0] = 0x5a;
    hi91[1] = 0xa5;
    wire::Put16(hi91 + 2, 76);
    hi91[6] = 0x91;
    wire::Put32(hi91 + 14, command_count);
    wire::Put16(hi91 + 4, protocols::CalculateHi91FrameCrc(76, hi91 + 6));
    uart(2, hi91, sizeof(hi91));
    if (!std::getenv("WBR_MOCK_FULL_DROP_IMU")) {
        wire::ImuSampleReport imu{};
        const auto stamp = Now();
        imu.header = {stamp, session};
        imu.sample_timestamp_us = stamp - 7;
        Enqueue(imu);
    }
}

void Process(libusb_transfer* transfer) {
    if (transfer->length != wire::kOutBytes) {
        std::abort();
    }
    wire::UsbOutRequest packet{};
    std::copy_n(transfer->buffer, packet.size(), packet.begin());
    const wire::UsbOutRequestView view(packet);
    const auto kind = view.MessageKind();
    if (kind == wire::Kind::Hello) {
        wire::HelloRequest hello{};
        if (!wire::Decode(packet, hello)) {
            std::abort();
        }
        session = hello.header.session_id;
        configured = armed = false;
        policy.Open(session);
        ready_delivered = false;
        stop_received = false;
        last_sequence = 0;
        replies.clear();
        wire::ReadyReport reply{};
        reply.header = {Now(), session};
        reply.capabilities =
            std::getenv("WBR_MOCK_BAD_CAPS")
                ? 0
                : (std::getenv("WBR_MOCK_ALL_UARTS") ? 255 : 15);
        reply.can_channel_mask = 12;
        reply.protection_frame_count = 6;
        reply.protection_signature = 0x12345678;
        reply.max_lease_us = wire::kCommandTimeoutUs;
        if (std::getenv("WBR_MOCK_PROFILE_MISMATCH")) {
            reply.protection_signature ^= 1;
        }
        if (std::getenv("WBR_MOCK_NO_PROFILE")) {
            reply.protection_flags = 0;
        }
        Enqueue(reply);
        // 故障注入在序列化后修改线上字节。
        if (std::getenv("WBR_MOCK_OLD_FIRMWARE")) {
            wire::Put32(replies.back().data(), 0x314b4341);
        }
        if (std::getenv("WBR_MOCK_V1_FIRMWARE")) {
            replies.back()[8] = 1;
        }
        if (std::getenv("WBR_MOCK_STALE_SESSION")) {
            wire::CanReceiveReport stale{};
            stale.header = {Now(), session ^ 0xffffffffU};
            stale.frame_count = 1;
            stale.frames[0] = {{3, 8, 0, 0x201, {}}, Now(), 0};
            Enqueue(stale);
        }
        return;
    }
    if (!ready_delivered || view.Session() != session) {
        std::abort();
    }
    if (kind == wire::Kind::ConfigureSafety || kind == wire::Kind::Arm) {
        int status = 0;
        if (kind == wire::Kind::ConfigureSafety) {
            wire::ConfigureSafetyRequest request{};
            if (armed || !wire::Decode(packet, request) ||
                request.uart_stop_count != 0 ||
                request.lease_us != wire::kCommandTimeoutUs) {
                std::abort();
            }
            // 零动作表示选择固件默认保护，不能清空保护基线。
            configured = policy.Configure(request.lease_us);
            policy.Observe(request.header.request_sequence);
            if (std::getenv("WBR_MOCK_SAFETY_REJECT")) {
                status = -22;
            }
        } else {
            wire::ArmRequest request{};
            if (!configured || !wire::Decode(packet, request) ||
                !policy.Arm(request.header.request_sequence, Now())) {
                std::abort();
            }
            armed = true;
            if (std::getenv("WBR_MOCK_ARM_REJECT")) {
                status = -22;
                armed = false;
                policy.Close();
            }
        }
        Enqueue(wire::RequestResultReport{
            {Now(), session}, view.Sequence(), kind, armed, status});
    } else if (kind == wire::Kind::CanTransmitRequest) {
        wire::CanTransmitRequest request{};
        if (!armed || !wire::Decode(packet, request)) {
            std::abort();
        }
        // 拒绝通知到达前，已排队的 OUT 仍可能到达关闭的会话。
        if (std::getenv("WBR_MOCK_DATA_REJECT") && !policy.Session()) {
            return;
        }
        const auto sequence = request.header.request_sequence;
        if (!policy.Accept(sequence, Now()) ||
            (command_count && !wire::NewSequence(sequence, last_sequence))) {
            std::abort();
        }
        last_sequence = sequence;
        ++command_count;
        if (std::getenv("WBR_MOCK_DATA_REJECT")) {
            Enqueue(wire::RequestResultReport{
                {Now(), session}, sequence, kind, armed, -28});
            policy.Close();
            return;
        }
        if (std::getenv("WBR_MOCK_FULL_LOAD")) {
            FullLoadSensors(request);
        }
        if (std::getenv("WBR_MOCK_UART")) {
            if (request.frame_count) {
                std::abort();
            }
            for (unsigned port = 1; port <= 2; ++port) {
                wire::UartReceiveReport uart{};
                const auto stamp = Now();
                uart.header = {stamp, session};
                uart.channel = port;
                uart.byte_count = 8;
                uart.block_sequence =
                    command_count + (command_count >= 3 ? 1 : 0);
                uart.receive_timestamp_us = stamp - 7;
                Enqueue(uart);
            }
        }
        wire::CanReceiveReport feedback{};
        feedback.header = {Now(), session};
        feedback.frame_count = 6;
        for (unsigned slot = 0; slot < 6; ++slot) {
            const auto& motor = platform::motor_map::kDefaultMotors[slot];
            auto& received = feedback.frames[slot];
            received.frame.channel = motor.bus;
            received.frame.id = motor.feedback_id;
            received.frame.length = 8;
            if (slot == 2 && std::getenv("WBR_MOCK_FULL_FAULT")) {
                received.frame.data[0] = 0xa0;
            }
            received.receive_timestamp_us =
                feedback.header.transmit_timestamp_us;
            received.bus_sequence = command_count;
        }
        Enqueue(feedback);
    } else if (kind == wire::Kind::UartTransmitRequest) {
        wire::UartTransmitRequest request{};
        if (!armed || !wire::Decode(packet, request) ||
            request.byte_count > 40 ||
            !policy.Accept(request.header.request_sequence, Now())) {
            std::abort();
        }
        wire::UartReceiveReport echo{};
        echo.header = {Now(), session};
        echo.channel = request.channel;
        echo.byte_count = request.byte_count;
        echo.block_sequence = request.header.request_sequence;
        echo.receive_timestamp_us = echo.header.transmit_timestamp_us;
        std::copy_n(request.bytes.begin(), request.byte_count,
                    echo.bytes.begin());
        Enqueue(echo);
    } else if (kind == wire::Kind::Stop) {
        wire::StopRequest request{};
        if (!wire::Decode(packet, request)) {
            std::abort();
        }
        stop_received = true;
    } else {
        std::abort();
    }
}
}  // namespace
extern "C" {
int LIBUSB_CALL libusb_init(libusb_context** output) {
    *output = &context;
    return 0;
}

void LIBUSB_CALL libusb_exit(libusb_context*) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!inputs.empty() || !outputs.empty() || !cancelled.empty()) {
        std::abort();
    }

    if (command_count && !disconnected && !stop_received && policy.Session()) {
        std::abort();
    }
}

ssize_t LIBUSB_CALL libusb_get_device_list(libusb_context*,
                                           libusb_device*** out) {
    static libusb_device* list[3];
    list[0] = std::getenv("WBR_MOCK_NO_DEVICE") ? nullptr : &device;
    list[1] =
        list[0] && std::getenv("WBR_MOCK_MULTIPLE") ? &second_device : nullptr;
    list[2] = nullptr;
    *out = list;
    return !list[0] ? 0 : (list[1] ? 2 : 1);
}

void LIBUSB_CALL libusb_free_device_list(libusb_device**, int) {}
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
    if (std::getenv("WBR_MOCK_NO_ACCESS") ||
        (dev->index && std::getenv("WBR_MOCK_SECOND_NO_ACCESS"))) {
        return LIBUSB_ERROR_ACCESS;
    }
    *out = dev->index ? &second_handle : &handle;
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
        : dev->index ? "00112233445566778899AABBCCDDEEFF1"
                     : "00112233445566778899AABBCCDDEEFF0";
    const int length = std::min(capacity, int(std::strlen(text)));
    std::memcpy(out, text, length);
    return length;
}

libusb_device* LIBUSB_CALL libusb_get_device(libusb_device_handle* dev) {
    return dev->index ? &second_device : &device;
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

void LIBUSB_CALL libusb_close(libusb_device_handle*) {}
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
    static libusb_interface interface{&descriptor, 1};
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
    std::lock_guard<std::mutex> lock(mutex);
    if (transfer->endpoint == 0x81 && std::getenv("WBR_MOCK_IN_FAIL")) {
        return LIBUSB_ERROR_IO;
    }

    if (transfer->endpoint == 0x01 &&
        (wire::HasHeader(transfer->buffer, transfer->length) &&
         transfer->buffer[9] == uint8_t(wire::Kind::Hello)) &&
        std::getenv("WBR_MOCK_REQUIRE_PREPOST") && inputs.size() != 4) {
        std::abort();
    }
    (transfer->endpoint == 0x81 ? inputs : outputs).push_back(transfer);
    changed.notify_all();
    return 0;
}

int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer* transfer) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto* queue : {&inputs, &outputs}) {
        auto found = std::find(queue->begin(), queue->end(), transfer);
        if (found != queue->end()) {
            queue->erase(found);
            cancelled.push_back(transfer);
            changed.notify_all();
            return 0;
        }
    }

    return LIBUSB_ERROR_NOT_FOUND;
}

int LIBUSB_CALL libusb_handle_events_timeout_completed(libusb_context*,
                                                       timeval*, int*) {
    libusb_transfer* transfer = nullptr;
    {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait_for(lock, std::chrono::milliseconds(10), [] {
            return !cancelled.empty() || !outputs.empty() ||
                   (!inputs.empty() && !replies.empty());
        });
        if (!cancelled.empty()) {
            transfer = cancelled.front();
            cancelled.pop_front();
            transfer->status = LIBUSB_TRANSFER_CANCELLED;
            transfer->actual_length = 0;
        } else if (!outputs.empty()) {
            transfer = outputs.front();
            outputs.pop_front();
            Process(transfer);
            transfer->status = LIBUSB_TRANSFER_COMPLETED;
            transfer->actual_length = transfer->length;
        } else if (!inputs.empty() && !replies.empty()) {
            transfer = inputs.front();
            inputs.pop_front();
            if (std::getenv("WBR_MOCK_DISCONNECT") && command_count >= 5 &&
                !disconnected) {
                disconnected = true;
                transfer->status = LIBUSB_TRANSFER_NO_DEVICE;
                transfer->actual_length = 0;
            } else {
                const auto packet = replies.front();
                replies.pop_front();
                std::copy(packet.begin(), packet.end(), transfer->buffer);
                if (packet[9] == uint8_t(wire::Kind::Ready)) {
                    ready_delivered = true;
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
