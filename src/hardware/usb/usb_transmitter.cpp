/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_transmitter.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>

#include "chassis/runtime/realtime_loop.hpp"
#include "concurrency/thread.hpp"
#include "usb_link.hpp"

namespace dust_control::host {
namespace rt = dust_control::realtime;
namespace {
constexpr uint64_t kQueueRetryNs = 1000000;
constexpr uint64_t kMaximumCanAgeNs = 20000000;
constexpr unsigned kReservedCanTransfers = 2;
const char* ProducerName(CanProducer producer) {
    return producer == CanProducer::kChassis ? "chassis" : "gimbal";
}
}  // namespace

UsbTransmitter::UsbTransmitter(UsbLink& link, UsbTransmitOptions options)
    : link_(link), options_(options) {}

UsbTransmitter::~UsbTransmitter() {
    Stop();
}

bool UsbTransmitter::ConfigureCan(CanProducer producer,
                                  std::span<const CanCommandGroup> groups,
                                  uint64_t timeout_ns) {
    const std::lock_guard lock(mutex_);
    const auto index = static_cast<size_t>(producer);
    if (started_ || index >= sources_.size() || sources_[index].configured ||
        groups.empty() || groups.size() > kMaxGroups || !timeout_ns ||
        timeout_ns > kMaximumCanAgeNs) {
        return false;
    }
    for (size_t i = 0; i < groups.size(); ++i) {
        if ((groups[i].bus != 2 && groups[i].bus != 3) ||
            groups[i].id > 0x7ff) {
            return false;
        }
        for (size_t j = 0; j < i; ++j) {
            if (groups[i] == groups[j]) {
                return false;
            }
        }
        for (const auto& source : sources_) {
            for (size_t j = 0; j < source.group_count; ++j) {
                if (source.groups[j] == groups[i]) {
                    return false;
                }
            }
        }
    }
    auto& source = sources_[index];
    std::copy(groups.begin(), groups.end(), source.groups.begin());
    source.group_count = groups.size();
    source.timeout_ns = timeout_ns;
    source.configured = true;
    return true;
}

bool UsbTransmitter::HasCanProducers() const {
    const std::lock_guard lock(mutex_);
    return std::any_of(sources_.begin(), sources_.end(),
                       [](const auto& source) { return source.configured; });
}

bool UsbTransmitter::Start() {
    {
        const std::lock_guard lock(mutex_);
        if (started_ || stop_ || !options_.uart_bytes_per_second ||
            !link_.healthy()) {
            error_ =
                "USB transmitter needs a healthy session and valid options";
            return false;
        }
        started_ = true;
        started_ns_ = rt::MonotonicTimeNs();
        running_ = true;
    }
    try {
        thread_ = std::thread([this] { Run(); });
    } catch (const std::exception& exception) {
        Fail(exception.what());
        running_ = false;
        // 尚未创建发送线程，启动调用者仍是会话的唯一发送者。
        link_.Stop();
        return false;
    }
    return true;
}

void UsbTransmitter::RequestStop() {
    stop_ = true;
    changed_.notify_all();
}
void UsbTransmitter::Stop() {
    RequestStop();
    if (thread_.joinable()) {
        thread_.join();
    }
}

std::string UsbTransmitter::error() const {
    const std::lock_guard lock(mutex_);
    return error_;
}

void UsbTransmitter::Fail(const std::string& reason) {
    const std::lock_guard lock(mutex_);
    if (error_.empty()) {
        error_ = reason;
    }
    failed_ = true;
    changed_.notify_all();
}

bool UsbTransmitter::SubmitCan(
    CanProducer producer,
    std::span<const dust_control::devices::CanFrame> frames, uint64_t sequence,
    uint64_t generated_ns) {
    bool accepted;
    {
        const std::lock_guard lock(mutex_);
        accepted = StoreCan(producer, frames, sequence, generated_ns,
                            rt::MonotonicTimeNs());
    }
    if (accepted) {
        changed_.notify_one();
    }
    return accepted;
}

bool UsbTransmitter::StoreCan(
    CanProducer producer,
    std::span<const dust_control::devices::CanFrame> frames, uint64_t sequence,
    uint64_t generated_ns, uint64_t now_ns) {
    const auto index = static_cast<size_t>(producer);
    if (!started_ || stop_ || failed_ || index >= sources_.size() ||
        frames.size() > kMaxGroups || !sequence || generated_ns > now_ns) {
        return false;
    }
    auto& source = sources_[index];
    if (!source.configured ||
        now_ns - (source.has_value ? source.generated_ns : started_ns_) >=
            source.timeout_ns ||
        now_ns - generated_ns >= source.timeout_ns ||
        (source.has_value &&
         (sequence <= source.sequence || generated_ns < source.generated_ns))) {
        return false;
    }
    for (size_t i = 0; i < frames.size(); ++i) {
        const CanCommandGroup group{frames[i].bus, frames[i].id};
        if (std::find(source.groups.begin(),
                      source.groups.begin() + source.group_count,
                      group) == source.groups.begin() + source.group_count) {
            return false;
        }
        for (size_t j = 0; j < i; ++j) {
            if (frames[j].bus == frames[i].bus &&
                frames[j].id == frames[i].id) {
                return false;
            }
        }
    }
    std::copy(frames.begin(), frames.end(), source.frames.begin());
    source.frame_count = frames.size();
    source.cursor = 0;
    source.sequence = sequence;
    source.generated_ns = generated_ns;
    source.has_value = source.pending = true;
    return true;
}

bool UsbTransmitter::SubmitUart(uint8_t channel, std::span<const uint8_t> bytes,
                                uint64_t generated_ns, uint64_t lifetime_ns) {
    const unsigned bit = channel < 3 ? channel : channel + 1;
    if (channel < 1 || channel > dust_control::wire::kUartPortCount ||
        !(link_.capabilities() & (1U << bit))) {
        return false;
    }
    bool accepted;
    {
        const std::lock_guard lock(mutex_);
        accepted = StoreUart(channel, bytes, generated_ns, lifetime_ns,
                             rt::MonotonicTimeNs());
    }
    if (accepted) {
        changed_.notify_one();
    }
    return accepted;
}

bool UsbTransmitter::StoreUart(uint8_t channel, std::span<const uint8_t> bytes,
                               uint64_t generated_ns, uint64_t lifetime_ns,
                               uint64_t now_ns) {
    if (!started_ || stop_ || failed_ || channel < 1 ||
        channel > dust_control::wire::kUartPortCount || bytes.empty() ||
        bytes.size() > UartMessage{}.bytes.size() || !lifetime_ns ||
        lifetime_ns > 1000000000 || generated_ns > now_ns ||
        now_ns - generated_ns >= lifetime_ns ||
        uart_size_ == uart_queue_.size()) {
        return false;
    }
    auto& message = uart_queue_[(uart_head_ + uart_size_) % uart_queue_.size()];
    message = {};
    std::copy(bytes.begin(), bytes.end(), message.bytes.begin());
    message.size = bytes.size();
    message.channel = channel;
    message.generated_ns = generated_ns;
    message.lifetime_ns = lifetime_ns;
    ++uart_size_;
    return true;
}

UsbTransmitter::Work UsbTransmitter::NextWork(uint64_t now_ns,
                                              unsigned free_transfers) {
    Work work;
    work.wake_ns = UINT64_MAX;
    bool ready = true;
    bool has_can = false;
    for (size_t i = 0; i < sources_.size(); ++i) {
        const auto& source = sources_[i];
        if (!source.configured) {
            continue;
        }
        has_can = true;
        const uint64_t deadline =
            (source.has_value ? source.generated_ns : started_ns_) +
            source.timeout_ns;
        work.control_deadline_ns = std::min(work.control_deadline_ns, deadline);
        work.wake_ns = std::min(work.wake_ns, deadline);
        if (now_ns >= deadline) {
            work.kind = WorkKind::kExpired;
            work.producer = static_cast<CanProducer>(i);
            return work;
        }
        ready = ready && source.has_value;
    }
    // 所有控制源首次提交前，不让其他流量替尚未启动的控制器维持会话。
    if (!ready) {
        return work;
    }
    bool can_pending = false;
    for (size_t offset = 0; offset < sources_.size(); ++offset) {
        const size_t index = (next_source_ + offset) % sources_.size();
        auto& source = sources_[index];
        if (!source.pending) {
            continue;
        }
        can_pending = true;
        if (!free_transfers) {
            break;
        }
        work.kind = WorkKind::kCan;
        work.producer = static_cast<CanProducer>(index);
        work.count =
            std::min(source.frame_count - source.cursor, work.frames.size());
        std::copy_n(source.frames.begin() + source.cursor, work.count,
                    work.frames.begin());
        source.cursor += work.count;
        source.pending = source.cursor < source.frame_count;
        next_source_ = (index + 1) % sources_.size();
        return work;
    }
    while (uart_size_) {
        auto& message = uart_queue_[uart_head_];
        if (message.cursor == 0 &&
            now_ns - message.generated_ns >= message.lifetime_ns) {
            uart_head_ = (uart_head_ + 1) % uart_queue_.size();
            --uart_size_;
            continue;
        }
        const unsigned reserved = has_can ? kReservedCanTransfers : 0;
        if (now_ns >= next_uart_ns_ && free_transfers > reserved &&
            !can_pending) {
            work.kind = WorkKind::kUart;
            work.channel = message.channel;
            work.count =
                std::min(message.size - message.cursor, work.bytes.size());
            std::copy_n(message.bytes.begin() + message.cursor, work.count,
                        work.bytes.begin());
            message.cursor += work.count;
            if (message.cursor == message.size) {
                uart_head_ = (uart_head_ + 1) % uart_queue_.size();
                --uart_size_;
            }
            const uint64_t interval = (work.count * 1000000000ULL +
                                       options_.uart_bytes_per_second - 1) /
                                      options_.uart_bytes_per_second;
            next_uart_ns_ = now_ns + interval;
            return work;
        }
        work.wake_ns = std::min(work.wake_ns, now_ns < next_uart_ns_
                                                  ? next_uart_ns_
                                                  : now_ns + kQueueRetryNs);
        break;
    }
    if (can_pending) {
        work.wake_ns = std::min(work.wake_ns, now_ns + kQueueRetryNs);
    }
    return work;
}

void UsbTransmitter::Run() {
    try {
        std::string error;
        if (!dust_control::concurrency::ConfigureThread(
                {options_.realtime, options_.cpu, options_.priority}, error)) {
            Fail("USB transmit thread setup: " + error);
        }
        pthread_setname_np(pthread_self(), options_.thread_name.c_str());
        while (!stop_ && !failed_) {
            if (!link_.healthy()) {
                Fail(link_.error().empty() ? "Bridge sensor stream timeout"
                                           : link_.error());
                break;
            }
            Work work;
            {
                std::unique_lock lock(mutex_);
                if (stop_ || failed_) {
                    break;
                }
                work =
                    NextWork(rt::MonotonicTimeNs(), link_.FreeTransmitSlots());
                if (work.kind == WorkKind::kWait) {
                    if (work.wake_ns == UINT64_MAX) {
                        changed_.wait_for(lock, std::chrono::milliseconds(1));
                    } else {
                        changed_.wait_until(
                            lock, std::chrono::steady_clock::time_point(
                                      std::chrono::nanoseconds(work.wake_ns)));
                    }
                    continue;
                }
            }
            if (work.kind == WorkKind::kExpired) {
                Fail(std::string("CAN producer expired: ") +
                     ProducerName(work.producer));
                break;
            }
            // 发送线程自身被调度停顿时，也不能恢复发送过期批次。
            if (rt::MonotonicTimeNs() >= work.control_deadline_ns) {
                Fail("CAN producer deadline exceeded during USB dispatch");
                break;
            }
            const bool sent =
                work.kind == WorkKind::kCan
                    ? link_.Send({work.frames.data(), work.count})
                    : link_.SendUart(work.channel, work.bytes.data(),
                                     work.count);
            if (!sent) {
                Fail(link_.error().empty() ? "USB transmit rejected"
                                           : link_.error());
                break;
            }
        }
    } catch (const std::exception& exception) {
        Fail(exception.what());
    }
    // MCU 只有会话级租约；任一控制源失效均停止整个会话，UI 不得继续续租。
    link_.Stop();
    running_ = false;
}
}  // namespace dust_control::host
