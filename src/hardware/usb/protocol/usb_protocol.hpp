/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file usb_protocol.hpp
 * @brief 可移植的 USB v4 消息结构与显式编解码接口。
 */
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace dust_control::wire {
constexpr uint32_t kMagic = 0x31425257;
constexpr uint8_t kVersion = 4;
constexpr size_t kInBytes = 160;
constexpr size_t kOutBytes = 128;
constexpr size_t kStandardInBytes = 64;
constexpr size_t kMaxCanFramesPerPacket = 6;
constexpr size_t kCanReceiveRecordBytes = 24;
constexpr size_t kRecordBytes = 16;
constexpr size_t kMaxRecords = kMaxCanFramesPerPacket;
constexpr uint8_t kUartPortCount = 6;
constexpr size_t kMaxUartWrite = kOutBytes - 32;
constexpr uint8_t kExtended = 1;
constexpr uint8_t kRemote = 2;
constexpr uint8_t kUartAction = 4;
constexpr uint8_t kFirmwareCanProtection = 1;
using UsbInPacket = std::array<uint8_t, kInBytes>;
using UsbOutRequest = std::array<uint8_t, kOutBytes>;

/**
 * @brief USB v4 消息类型标识。
 */
enum class Kind : uint8_t {
    Hello = 1,
    Ready = 2,
    CanTransmitRequest = 3,
    CanReceiveReport = 4,
    UartTransmitRequest = 5,
    UartReceiveReport = 6,
    ImuSampleReport = 7,
    ConfigureSafety = 8,
    Arm = 9,
    Stop = 10,
    Heartbeat = 11,
    Result = 12,
    Status = 13,
};

/**
 * @brief 硬件配置固定的逻辑 UART 端口。
 */
enum class Port : uint8_t {
    Channel1 = 1,
    Channel2 = 2,
    Channel3 = 3,
    Channel4 = 4,
    Channel5 = 5,
    Channel6 = 6,
};

/**
 * @brief 会话失效原因。
 */
enum class StopReason : uint8_t {
    None,
    Explicit,
    LeaseExpired,
    UsbLost,
    TransportError,
    Replaced
};

/**
 * @brief 经典 CAN 帧的逻辑字段；线上占用 16 字节，不依赖结构体内存布局。
 */
struct CanFrame {
    uint8_t channel = 0;
    uint8_t length = 0;
    uint8_t flags = 0;
    uint32_t id = 0;
    uint8_t data[8]{};
};

/** @brief 会话停止时向指定 UART 发送的动作，不包含 CAN ID 或 CAN 标志。 */
struct UartStopAction {
    uint8_t channel = 0;
    uint8_t length = 0;
    uint8_t data[8]{};
};

// 以下为逻辑消息结构，不依赖内存填充、对齐或主机字节序，禁止直接发送
// sizeof(struct)。
struct UsbRequestHeader {
    uint32_t request_sequence = 0;
    uint32_t session_id = 0;
};
struct UsbReportHeader {
    uint32_t transmit_timestamp_us = 0;
    uint32_t session_id = 0;
};

/** @brief 主机要求 MCU 向 CAN 总线发送的帧批次。 */
struct CanTransmitRequest {
    UsbRequestHeader header;
    uint8_t frame_count = 0;
    std::array<CanFrame, kMaxCanFramesPerPacket> frames{};
};
/** @brief MCU 从 CAN 总线收到的一帧及采集信息。 */
struct CanReceivedFrame {
    CanFrame frame;
    uint32_t receive_timestamp_us = 0;
    uint32_t bus_sequence = 0;
};
/** @brief CAN 接收上报，线上长度为 16 + frame_count × 24。 */
struct CanReceiveReport {
    UsbReportHeader header;
    uint8_t frame_count = 0;
    std::array<CanReceivedFrame, kMaxCanFramesPerPacket> frames{};
};
/** @brief 主机要求 MCU 向 UART 发送的字节流，线上固定 128 字节。 */
struct UartTransmitRequest {
    UsbRequestHeader header;
    uint8_t channel = 0;
    uint8_t byte_count = 0;
    std::array<uint8_t, kMaxUartWrite> bytes{};
};
/** @brief UART 接收块，线上固定 64 字节；块边界不是外设协议帧边界。 */
struct UartReceiveReport {
    UsbReportHeader header;
    uint8_t channel = 0;
    uint8_t byte_count = 0;
    uint32_t block_sequence = 0;
    uint32_t receive_timestamp_us = 0;
    std::array<uint8_t, 40> bytes{};
};

/** @brief 握手回复：描述通道能力、包长上限和固件保护配置。 */
struct ReadyReport {
    UsbReportHeader header;
    uint32_t capabilities = 0;
    uint32_t request_bytes = kOutBytes;
    uint32_t max_report_bytes = kInBytes;
    uint32_t can_channel_mask = 0;
    uint8_t max_can_frames = kMaxCanFramesPerPacket;
    uint8_t protection_frame_count = 0;
    uint8_t protection_flags = kFirmwareCanProtection;
    uint32_t protection_signature = 0;
    uint32_t max_lease_us = 0;
};
/** @brief IMU 原始采样及温度，固定 64 字节上报。 */
struct ImuSampleReport {
    UsbReportHeader header;
    uint32_t sample_timestamp_us = 0;
    std::array<uint8_t, 25> spi_bytes{};
    float temperature_c = 0;
    uint32_t sample_sequence = 0;
};
/** @brief 软件请求处理结果，不代表外设已执行。 */
struct RequestResultReport {
    UsbReportHeader header;
    uint32_t request_sequence = 0;
    Kind request_kind = Kind::Hello;
    bool armed = false;
    int32_t result_code = 0;
};
enum class StatusTarget : uint8_t { Session = 0, Can = 1, Uart = 2 };
struct StatusRequest {
    UsbRequestHeader header;
    StatusTarget target = StatusTarget::Session;
    uint8_t channel = 0;
};
struct SessionStatusReport {
    UsbReportHeader header;
    uint8_t channel = 0;
    bool armed = false;
    bool configured = false;
    StopReason stop_reason = StopReason::None;
    uint32_t rejected_requests = 0;
    uint32_t can_report_drops = 0;
    uint32_t sensor_report_drops = 0;
    uint32_t stop_errors = 0;
    uint32_t reply_drops = 0;
    uint32_t usb_send_errors = 0;
    uint32_t usb_receive_drops = 0;
    uint32_t usb_endpoint_errors = 0;
    uint32_t protection_signature = 0;
    uint32_t max_lease_us = 0;
};
struct CanStatusReport {
    UsbReportHeader header;
    uint8_t channel = 0;
    uint32_t received = 0;
    uint32_t submitted = 0;
    uint32_t completed = 0;
    uint32_t errors = 0;
    uint32_t dropped = 0;
    uint32_t queue_depth = 0;
    uint32_t controller_state = 0;
    uint32_t tx_errors = 0;
    uint32_t rx_errors = 0;
};
struct UartStatusReport {
    UsbReportHeader header;
    uint8_t channel = 0;
    uint32_t received_blocks = 0;
    uint32_t dropped_blocks = 0;
    uint32_t completed_writes = 0;
    uint32_t tx_errors = 0;
};
/** @brief 设置租约，确认 CAN 默认保护并补充 UART 停止动作。 */
struct ConfigureSafetyRequest {
    UsbRequestHeader header;
    uint32_t lease_us = 0;
    uint8_t uart_stop_count = 0;
    std::array<UartStopAction, kMaxRecords> uart_stop_actions{};
};
struct HelloRequest {
    UsbRequestHeader header;
};
struct ArmRequest {
    UsbRequestHeader header;
};
struct StopRequest {
    UsbRequestHeader header;
};
struct HeartbeatRequest {
    UsbRequestHeader header;
};

bool Encode(const ReadyReport& message, UsbInPacket& packet);
bool Encode(const ImuSampleReport& message, UsbInPacket& packet);
bool Encode(const RequestResultReport& message, UsbInPacket& packet);
bool Encode(const SessionStatusReport& message, UsbInPacket& packet);
bool Encode(const CanStatusReport& message, UsbInPacket& packet);
bool Encode(const UartStatusReport& message, UsbInPacket& packet);
bool Encode(const HelloRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, HelloRequest& message);
bool Encode(const ArmRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, ArmRequest& message);
bool Encode(const StopRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, StopRequest& message);
bool Encode(const HeartbeatRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, HeartbeatRequest& message);
bool Encode(const StatusRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, StatusRequest& message);
bool Encode(const ConfigureSafetyRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, ConfigureSafetyRequest& message);
// 显式编解码：失败不提交业务操作；Decode 接收完整下行包。
bool Encode(const CanTransmitRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, CanTransmitRequest& message);
bool Encode(const CanReceiveReport& message, UsbInPacket& packet);
bool Encode(const UartTransmitRequest& message, UsbOutRequest& packet);
bool Decode(const UsbOutRequest& packet, UartTransmitRequest& message);
bool Encode(const UartReceiveReport& message, UsbInPacket& packet);

// 原始字节辅助接口；调用方负责缓冲区长度。包容量不等于实际传输长度。
size_t UsbReportLength(const UsbInPacket& packet);
uint32_t U32(const uint8_t* p);
void Put32(uint8_t* p, uint32_t value);
void Put16(uint8_t* p, uint16_t value);
void Header(uint8_t* p, Kind kind, uint32_t sequence, uint32_t session);
bool HasHeader(const uint8_t* p, size_t size);
bool NewSequence(uint32_t next, uint32_t previous);
CanFrame Decode(const uint8_t* p);
bool ValidRecords(const UsbOutRequest& packet, bool safety = false);
uint32_t ProtectionSignature(uint32_t timeout_us, const CanFrame* records,
                             size_t count);

class UsbOutRequestView {
   public:
    explicit UsbOutRequestView(const UsbOutRequest& request)
        : request_(request) {}

    Kind MessageKind() const {
        return Kind(request_[9]);
    }
    uint32_t Sequence() const {
        return U32(request_.data() + 4);
    }
    uint32_t Session() const {
        return U32(request_.data() + 12);
    }
    uint32_t LeaseUs() const {
        return U32(request_.data() + 16);
    }
    uint8_t RecordCount() const {
        return request_[10];
    }
    uint8_t UartChannel() const {
        return request_[10];
    }
    uint8_t UartLength() const {
        return request_[11];
    }
    uint8_t StatusTarget() const {
        return request_[10];
    }
    uint8_t StatusChannel() const {
        return request_[11];
    }
    const uint8_t* Payload() const {
        return request_.data() + 32;
    }

   private:
    const UsbOutRequest& request_;
};

// 上行与下行解码均在同一协议实现中；失败不提交部分结果。
bool Decode(const UsbInPacket& packet, CanReceiveReport& message);
bool Decode(const UsbInPacket& packet, UartReceiveReport& message);
bool Decode(const UsbInPacket& packet, ImuSampleReport& message);
bool Decode(const UsbInPacket& packet, ReadyReport& message);
bool Decode(const UsbInPacket& packet, RequestResultReport& message);

/**
 * @brief 独立于硬件的租约与序号状态；所有访问必须由同一所有者执行。
 */
class SessionPolicy {
   public:
    /**
     * @brief 重置策略并设置会话随机标识；调用者必须拒绝零会话标识。
     */
    void Open(uint32_t nonce);

    /**
     * @brief 记录已接纳的序号，不续租。
     */
    void Observe(uint32_t seq);

    /**
     * @brief 在授权转发前设置 1000..1000000 微秒租约；状态或范围无效时返回
     * false。
     */
    bool Configure(uint32_t timeout_us);

    /**
     * @brief 判断 seq 是否晚于上次观察到的序号。
     */
    bool Fresh(uint32_t seq) const;

    /**
     * @brief 使用新序号授权已配置的策略转发，并从 now（微秒）开始计租。
     */
    bool Arm(uint32_t seq, uint32_t now);

    /**
     * @brief 接纳新流量并续租；未授权、序号陈旧或租约过期时返回 false。
     */
    bool Accept(uint32_t seq, uint32_t now);

    /**
     * @brief 使用可处理回绕的无符号微秒减法检查已授权会话的租约。
     */
    bool Expired(uint32_t now) const;

    /**
     * @brief 重置会话状态并撤销转发许可。
     */
    void Close();

    /**
     * @brief 返回当前会话标识；零表示未激活。
     */
    uint32_t Session() const;

    /**
     * @brief 返回是否已授予转发许可。
     */
    bool Armed() const;

    /**
     * @brief 返回是否已配置租约。
     */
    bool Configured() const;

   private:
    uint32_t session_ = 0;
    uint32_t timeout_ = 0;
    uint32_t last_sequence_ = 0;
    uint32_t last_receive_ = 0;
    bool configured_ = false, armed_ = false, seen_ = false;
};

}  // namespace dust_control::wire
