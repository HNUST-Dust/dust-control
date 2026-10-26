/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <atomic>
#include <condition_variable>
#include <span>
#include <string>
#include <thread>

#include "concurrency/priority_mutex.hpp"
#include "hardware/can_frame.hpp"

namespace dust_control::test {
class UsbTransmitFixture;
}

namespace dust_control::host {
// 主机选择的命令租约及接收数据时效，单位为微秒；不是线协议固定值。
inline constexpr uint32_t kCommandTimeoutUs = 20000;

class UsbLink;
enum class CanProducer : uint8_t { kChassis, kGimbal };
struct CanCommandGroup {
    uint8_t bus = 0;
    uint16_t id = 0;
    bool operator==(const CanCommandGroup&) const = default;
};
struct UsbTransmitOptions {
    bool realtime = true;
    int cpu = 3;
    int priority = 79;
    uint32_t uart_bytes_per_second = 8000;
    std::string thread_name = "dust-usb-tx";
};

// 多业务线程提交编码结果；唯一发送线程拥有所有业务 USB 提交和 Stop。
// CAN 保留每个来源的最新批次，UART 使用有界 FIFO。对象不能重新启动。
class UsbTransmitter {
   public:
    explicit UsbTransmitter(UsbLink& link, UsbTransmitOptions options = {});
    ~UsbTransmitter();
    // 启动前注册完整 CAN 命令组；跨来源的同总线同 ID 被拒绝，不能共享槽位。
    bool ConfigureCan(CanProducer producer,
                      std::span<const CanCommandGroup> groups,
                      uint64_t timeout_ns);
    // 启动装配时查询，不建立假控制源或续租。
    bool HasCanProducers() const;
    bool Start();
    // 由生命周期所有者调用：唤醒并等待发送线程提交 Stop，再允许关闭 UsbLink。
    // 先请求所有板停止，再逐一等待，避免等待一板时其他板耗尽租约。
    void RequestStop();
    void Stop();
    // 同步复制，不等待 USB。序号必须递增；时间为目标生成时的主机单调纳秒。
    // 空批次代表本周期没有命令，仍更新该来源的活性；旧序号不会刷新期限。
    bool SubmitCan(CanProducer producer,
                   std::span<const dust_control::devices::CanFrame> frames,
                   uint64_t sequence, uint64_t generated_ns);
    // 一次提交一个完整 UART 消息（最多 512 字节），满时返回 false。
    // 过期且尚未开始的消息丢弃；开始发送后保持消息顺序，允许 CAN 在块间抢占。
    bool SubmitUart(uint8_t channel, std::span<const uint8_t> bytes,
                    uint64_t generated_ns, uint64_t lifetime_ns = 200000000);
    bool healthy() const {
        return running_.load() && !failed_.load() && !stop_.load();
    }
    std::string error() const;

   private:
    friend class dust_control::test::UsbTransmitFixture;
    static constexpr size_t kSourceCount = 2;
    static constexpr size_t kMaxGroups = 16;
    static constexpr size_t kUartQueueCapacity = 16;
    struct CanSource {
        bool configured = false;
        std::array<CanCommandGroup, kMaxGroups> groups{};
        size_t group_count = 0;
        uint64_t timeout_ns = 0;
        std::array<dust_control::devices::CanFrame, kMaxGroups> frames{};
        size_t frame_count = 0;
        size_t cursor = 0;
        uint64_t sequence = 0;
        uint64_t generated_ns = 0;
        bool has_value = false;
        bool pending = false;
    };
    struct UartMessage {
        std::array<uint8_t, 512> bytes{};
        size_t size = 0;
        size_t cursor = 0;
        uint8_t channel = 0;
        uint64_t generated_ns = 0;
        uint64_t lifetime_ns = 0;
    };
    enum class WorkKind { kWait, kCan, kUart, kExpired };
    struct Work {
        WorkKind kind = WorkKind::kWait;
        CanProducer producer = CanProducer::kChassis;
        std::array<dust_control::devices::CanFrame, 6> frames{};
        size_t count = 0;
        uint8_t channel = 0;
        std::array<uint8_t, 96> bytes{};
        uint64_t wake_ns = 0;
        uint64_t control_deadline_ns = UINT64_MAX;
    };
    // 下列状态操作要求持有 mutex_，不调用 USB、不等待、不执行动态分配。
    bool StoreCan(CanProducer producer,
                  std::span<const dust_control::devices::CanFrame> frames,
                  uint64_t sequence, uint64_t generated_ns, uint64_t now_ns);
    bool StoreUart(uint8_t channel, std::span<const uint8_t> bytes,
                   uint64_t generated_ns, uint64_t lifetime_ns,
                   uint64_t now_ns);
    Work NextWork(uint64_t now_ns, unsigned free_transfers);
    void Run();
    void Fail(const std::string& reason);

    UsbLink& link_;
    UsbTransmitOptions options_;
    mutable dust_control::concurrency::PriorityMutex mutex_;
    std::condition_variable_any changed_;
    std::array<CanSource, kSourceCount> sources_{};
    std::array<UartMessage, kUartQueueCapacity> uart_queue_{};
    size_t uart_head_ = 0;
    size_t uart_size_ = 0;
    size_t next_source_ = 0;
    uint64_t started_ns_ = 0;
    uint64_t next_uart_ns_ = 0;
    bool started_ = false;
    std::string error_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
    std::thread thread_;
};
// 非拥有端口：业务只能提交数据，不能停止发送器或关闭共享会话。
class CanCommandPort {
   public:
    CanCommandPort(UsbTransmitter& transmitter, CanProducer producer)
        : transmitter_(transmitter), producer_(producer) {}
    bool Configure(std::span<const CanCommandGroup> groups,
                   uint64_t timeout_ns) const {
        return transmitter_.ConfigureCan(producer_, groups, timeout_ns);
    }
    bool Submit(std::span<const dust_control::devices::CanFrame> frames,
                uint64_t sequence, uint64_t generated_ns) const {
        return transmitter_.SubmitCan(producer_, frames, sequence,
                                      generated_ns);
    }

   private:
    UsbTransmitter& transmitter_;
    CanProducer producer_;
};
class UartFramePort {
   public:
    explicit UartFramePort(UsbTransmitter& transmitter)
        : transmitter_(transmitter) {}
    bool Submit(uint8_t channel, std::span<const uint8_t> bytes,
                uint64_t generated_ns, uint64_t lifetime_ns = 200000000) const {
        return transmitter_.SubmitUart(channel, bytes, generated_ns,
                                       lifetime_ns);
    }

   private:
    UsbTransmitter& transmitter_;
};
}  // namespace dust_control::host
