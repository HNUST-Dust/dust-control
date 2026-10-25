/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_link.hpp"

#include <algorithm>
#include <chassis/runtime/realtime_loop.hpp>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "concurrency/thread.hpp"
#include "usb_transmitter.hpp"

namespace dust_control::host {
namespace rt = dust_control::realtime;
UsbLink::UsbLink(UartReceiveQueue& uart_queue, const UsbOptions& options,
                 const dust_control::devices::MotorConfig& motors,
                 const dust_control::devices::ImuConfig& imu)
    : options_(options),
      motor_config_(motors),
      uart_queue_(uart_queue),
      motor_decoder_(motors),
      hi91_decoder_(options.strict_hi91_crc, imu) {
    options_.uart_sensors.Validate();
}
UsbLink::~UsbLink() {
    Close();
}

bool UsbLink::FailSession(const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (error_.empty()) {
            error_ = reason;
        }
    }

    failed_ = true;
    ready_cv_.notify_all();
    return false;
}

std::string UsbLink::error() {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}

bool UsbLink::healthy() const {
    if (!ready_received_ || failed_) {
        return false;
    }

    // 先读取接收时间戳，再采样当前时间。若顺序相反，接收回调可能在两者之间
    // 发布更新的时间戳，导致无符号时间差下溢，把正常数据流误判为超时。
    const uint64_t received_us = last_receive_us_.load();
    const uint64_t now_us = rt::MonotonicTimeNs() / 1000;
    return now_us - received_us < kCommandTimeoutUs;
}

bool UsbLink::Open() {
    if (!OpenDevice()) {
        return false;
    }
    return StartSession();
}

bool UsbLink::OpenDevice() {
    if (libusb_init(&context_)) {
        return FailSession("Cannot initialize USB");
    }

    libusb_device** devices = nullptr;
    const auto count = libusb_get_device_list(context_, &devices);
    if (count < 0) {
        return FailSession(std::string("Cannot enumerate USB: ") +
                           libusb_error_name(int(count)));
    }

    std::vector<std::string> matches;
    std::string diagnostics;
    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor descriptor{};
        const int described =
            libusb_get_device_descriptor(devices[i], &descriptor);
        if (described) {
            diagnostics += "Cannot read USB device descriptor: " +
                           std::string(libusb_error_name(described)) + "; ";
            continue;
        }
        if (descriptor.idVendor != options_.vid ||
            (options_.pid && descriptor.idProduct != *options_.pid)) {
            continue;
        }
        std::ostringstream label;
        label << "bus=" << unsigned(libusb_get_bus_number(devices[i]))
              << " address=" << unsigned(libusb_get_device_address(devices[i]))
              << " VID:PID=" << std::hex << std::setfill('0') << std::setw(4)
              << descriptor.idVendor << ':' << std::setw(4)
              << descriptor.idProduct;
        libusb_device_handle* candidate = nullptr;
        const int opened = libusb_open(devices[i], &candidate);
        if (opened) {
            diagnostics += label.str() + " open: " + libusb_error_name(opened) +
                           " (check USB access permissions; run with sudo); ";
            continue;
        }
        const auto read = [&](uint8_t index, const char* field,
                              std::string& value) {
            unsigned char buffer[256];
            const int length =
                index ? libusb_get_string_descriptor_ascii(
                            candidate, index, buffer, sizeof(buffer))
                      : LIBUSB_ERROR_NOT_FOUND;
            if (length <= 0) {
                diagnostics +=
                    label.str() + " " + field + ": " +
                    libusb_error_name(length < 0 ? length : LIBUSB_ERROR_IO) +
                    "; ";
                return false;
            }
            value.assign(reinterpret_cast<char*>(buffer), size_t(length));
            return true;
        };
        std::string manufacturer;
        std::string product;
        std::string serial;
        const bool strings =
            read(descriptor.iManufacturer, "manufacturer", manufacturer) &&
            read(descriptor.iProduct, "product", product) &&
            read(descriptor.iSerialNumber, "serial", serial);
        if (!strings || manufacturer != options_.manufacturer ||
            product != options_.product ||
            (!options_.serial.empty() && serial != options_.serial)) {
            libusb_close(candidate);
            continue;
        }
        label << " manufacturer=" << manufacturer << " product=" << product
              << " serial=" << serial;
        libusb_config_descriptor* config = nullptr;
        const int configured =
            libusb_get_active_config_descriptor(devices[i], &config);
        bool in = false;
        bool out = false;
        if (!configured) {
            for (unsigned n = 0; n < config->bNumInterfaces; ++n) {
                for (int alt = 0; alt < config->interface[n].num_altsetting;
                     ++alt) {
                    const auto& interface =
                        config->interface[n].altsetting[alt];
                    if (interface.bInterfaceNumber != 0 ||
                        interface.bAlternateSetting != 0 ||
                        interface.bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC) {
                        continue;
                    }
                    for (unsigned e = 0; e < interface.bNumEndpoints; ++e) {
                        const auto& ep = interface.endpoint[e];
                        if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) !=
                                LIBUSB_TRANSFER_TYPE_BULK ||
                            ep.wMaxPacketSize != 512) {
                            continue;
                        }
                        in |= ep.bEndpointAddress == 0x81;
                        out |= ep.bEndpointAddress == 0x01;
                    }
                }
            }
            libusb_free_config_descriptor(config);
        }
        if (configured || !in || !out ||
            libusb_get_device_speed(devices[i]) != LIBUSB_SPEED_HIGH) {
            diagnostics +=
                label.str() +
                " requires 480 Mbps, vendor interface 0 alt 0, Bulk 0x01/0x81 "
                "MPS 512" +
                (configured ? std::string(": ") + libusb_error_name(configured)
                            : "") +
                "; ";
            libusb_close(candidate);
            continue;
        }
        matches.push_back(label.str());
        if (!device_) {
            device_ = candidate;
        } else {
            libusb_close(candidate);
        }
    }

    libusb_free_device_list(devices, 1);
    // 无法读取的候选也可能是匹配板卡；无法确认唯一性时，
    // 不能静默选取可访问的板卡。
    if (!diagnostics.empty()) {
        return FailSession("USB discovery incomplete: " + diagnostics);
    }

    if (matches.empty()) {
        return FailSession(
            "No matching USB bridge (manufacturer=" + options_.manufacturer +
            " product=" + options_.product + " serial=" + options_.serial +
            ")");
    }

    if (matches.size() != 1) {
        std::string message =
            "Multiple matching USB bridges; set an exact USB serial:";
        for (const auto& match : matches) {
            message += "\n  " + match;
        }
        return FailSession(message);
    }

    std::clog << "USB bridge " << matches.front() << std::endl;
    const int claimed = libusb_claim_interface(device_, 0);
    if (claimed) {
        return FailSession(std::string("Cannot claim USB bridge interface: ") +
                           libusb_error_name(claimed));
    }

    claimed_ = true;
    return true;
}

void UsbLink::SubmitReceive(ReceiveTransfer& input) {
    libusb_fill_bulk_transfer(input.transfer, device_, 0x81, input.data.data(),
                              dust_control::wire::kInBytes, CompleteReceive,
                              &input, 0);
    input.pending = true;
    ++active_transfers_;
    if (libusb_submit_transfer(input.transfer)) {
        input.pending = false;
        --active_transfers_;
        FailSession("USB IN submit");
    }
}

void LIBUSB_CALL UsbLink::CompleteReceive(libusb_transfer* transfer) {
    auto& input = *static_cast<ReceiveTransfer*>(transfer->user_data);
    auto& self = *input.owner;
    input.pending = false;
    --self.active_transfers_;
    if (self.closing_) {
        return;
    }

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
        transfer->actual_length < 16 ||
        transfer->actual_length !=
            int(dust_control::wire::UsbReportLength(input.data))) {
        self.FailSession(
            "USB IN status=" + std::to_string(int(transfer->status)) +
            " actual_length=" + std::to_string(transfer->actual_length) +
            " invalid report length");
        return;
    }

    self.HandleReport(input.data);

    if (!self.closing_ && !self.failed_) {
        self.SubmitReceive(input);
    }
}

void LIBUSB_CALL UsbLink::CompleteTransmit(libusb_transfer* transfer) {
    auto& output = *static_cast<TransmitTransfer*>(transfer->user_data);
    auto& self = *output.owner;
    --self.active_transfers_;
    if (!self.closing_ && (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
                           transfer->actual_length != transfer->length)) {
        self.FailSession(
            "USB OUT status=" + std::to_string(int(transfer->status)) +
            " actual_length=" + std::to_string(transfer->actual_length) +
            " expected_length=" + std::to_string(transfer->length));
    }

    output.free.store(true, std::memory_order_release);
}

bool UsbLink::QueueTransmit(const uint8_t* data, size_t length) {
    if (failed_ || closing_) {
        return false;
    }

    for (auto& output : transmit_transfers_) {
        bool available = true;
        if (!output.free.compare_exchange_strong(available, false)) {
            continue;
        }
        std::copy_n(data, length, output.data.begin());
        libusb_fill_bulk_transfer(output.transfer, device_, 0x01,
                                  output.data.data(), length, CompleteTransmit,
                                  &output, 100);
        ++active_transfers_;
        if (libusb_submit_transfer(output.transfer)) {
            --active_transfers_;
            output.free = true;
            return FailSession("USB OUT submit");
        }
        return true;
    }

    return FailSession(
        "USB command queue full; control is stopped instead of replaying "
        "stale commands");
}

bool UsbLink::Send(std::span<const dust_control::devices::CanFrame> frames) {
    if (!healthy()) {
        return false;
    }
    if (frames.size() > dust_control::wire::kMaxCanFramesPerPacket) {
        return FailSession("Invalid CAN transmit request");
    }
    dust_control::wire::CanTransmitRequest request{};
    request.header = {++request_sequence_, session_};
    request.frame_count = static_cast<uint8_t>(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        auto& record = request.frames[i];
        record.channel = frames[i].bus;
        record.id = frames[i].id;
        record.length = static_cast<uint8_t>(frames[i].bytes.size());
        std::copy(frames[i].bytes.begin(), frames[i].bytes.end(), record.data);
    }
    dust_control::wire::UsbOutRequest packet{};
    if (!dust_control::wire::Encode(request, packet)) {
        return FailSession("Invalid CAN transmit request");
    }
    return QueueTransmit(packet.data(), packet.size());
}

bool UsbLink::SendUart(uint8_t channel, const uint8_t* bytes, size_t size) {
    if (!healthy() || !bytes || !size ||
        size > dust_control::wire::kMaxUartWrite || channel < 1 ||
        channel > dust_control::wire::kUartPortCount) {
        return false;
    }
    const unsigned bit = channel < 3 ? channel : channel + 1;
    if (!(capabilities_ & (1U << bit))) {
        return false;
    }
    dust_control::wire::UartTransmitRequest request{};
    request.header = {++request_sequence_, session_};
    request.channel = channel;
    request.byte_count = uint8_t(size);
    std::copy_n(bytes, size, request.bytes.begin());
    dust_control::wire::UsbOutRequest packet{};
    if (!dust_control::wire::Encode(request, packet)) {
        return false;
    }
    return QueueTransmit(packet.data(), packet.size());
}

void UsbLink::RunEventLoop() {
    std::string error;
    pthread_setname_np(pthread_self(), options_.thread_name.c_str());
    if (!dust_control::concurrency::ConfigureThread(
            {options_.realtime, options_.cpu, options_.priority}, error)) {
        FailSession("USB receive thread setup: " + error);
    }

    if (!failed_) {
        for (auto& input : receive_transfers_) {
            SubmitReceive(input);
        }
    }

    if (!failed_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            receive_preposted_ = true;
        }
        ready_cv_.notify_all();
    }

    bool cancelled = false;
    for (;;) {
        if (failed_ && !closing_) {
            // 设备断开后 libusb 可能持续立即返回。阻塞 FIFO 事件线程，
            // 让控制线程有机会观察故障并调用 Close；Close 唤醒事件线程，
            // 由事件线程取消并排空未完成的传输。
            std::unique_lock<std::mutex> lock(mutex_);
            ready_cv_.wait(lock, [this] { return closing_.load(); });
        }
        if (closing_) {
            if (!cancelled) {
                for (auto& input : receive_transfers_) {
                    if (input.pending) {
                        libusb_cancel_transfer(input.transfer);
                    }
                }
                for (auto& output : transmit_transfers_) {
                    if (!output.free) {
                        libusb_cancel_transfer(output.transfer);
                    }
                }
                cancelled = true;
            }
            if (!active_transfers_) {
                break;
            }
        }
        // 即使 UART 没有流量，待处理的丢帧策略也必须定时检查。
        if (ready_received_ && !failed_ && !closing_) {
            wfly_sbus_decoder_.PollTimeout(rt::MonotonicTimeNs() / 1000);
        }
        timeval timeout{0, wfly_sbus_decoder_.timeout_pending() ? 1000 : 20000};
        const int result =
            libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
        if (result && result != LIBUSB_ERROR_INTERRUPTED) {
            FailSession("USB event handling error");
        }
    }
}

void UsbLink::Close() {
    if (!context_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closing_ = true;
    }

    ready_cv_.notify_all();
    // 由唯一事件线程取消并排空传输，避免低优先级线程争用事件锁。
    if (thread_.joinable()) {
        thread_.join();
    }

    for (auto& input : receive_transfers_) {
        if (input.transfer) {
            libusb_free_transfer(input.transfer);
            input.transfer = nullptr;
        }
    }

    for (auto& output : transmit_transfers_) {
        if (output.transfer) {
            libusb_free_transfer(output.transfer);
            output.transfer = nullptr;
        }
    }

    if (claimed_) {
        libusb_release_interface(device_, 0);
    }

    if (device_) {
        libusb_close(device_);
    }

    libusb_exit(context_);
    context_ = nullptr;
    device_ = nullptr;
}

unsigned UsbLink::FreeTransmitSlots() const {
    unsigned count = 0;
    for (const auto& transfer : transmit_transfers_) {
        count += transfer.free.load(std::memory_order_acquire);
    }
    return count;
}

}  // namespace dust_control::host
