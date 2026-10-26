# USB v4 线上协议基准

记录日期：2026-10-26。本文件依据当前主机仓库的消息定义、编解码、会话策略及接收实现整理，作为后续重构的核对基准；本次没有修改协议代码，也没有重新读取 MCU 仓库。

文件拆分、目录、命名空间、类名和函数名可以改变。只要本文件记录的线上格式、字段语义和交互行为保持兼容，就不需要同步修改 MCU。修改这些通信约定则属于协议变更，须单独评审并协调两端。

## 1. 传输与基础类型

- 使用 USB Bulk；当前设备接口为 vendor interface 0 / alternate 0，OUT 端点 `0x01`，IN 端点 `0x81`，High Speed、最大端点包长 512 字节。端点包长不是应用消息长度。
- 所有 OUT 请求固定 **128 字节**；CAN IN 报告为 **16 + 24 × N 字节**，`N=1..6`；其他 IN 报告固定 **64 字节**。
- IN 数组容量为 160 字节，不代表每次必须发送 160 字节；接收方必须检查实际传输长度。
- 多字节整数均为小端序。`u8/u32/i32` 分别为 8 位无符号、32 位无符号和 32 位有符号整数；负结果码按 32 位补码传输。
- 唯一浮点字段是温度：IEEE 754 binary32，其 32 位位模式按小端序传输。
- 偏移均以消息首字节为 0；记录内偏移另行注明。不得发送 C++ 结构体内存或依赖 `sizeof(struct)`。
- 编码器先清零包缓冲，再写字段；未使用区域为零。当前解码器并非对所有消息的所有保留字节都执行拒绝检查，见第 10 节，重构不能擅自收紧接纳条件。
- 本层没有额外应用 CRC 字段；第 8 节的 CRC32 仅标识保护配置。

## 2. 公共头：偏移 0..15

| 偏移 | 字节数 | 字段 | 格式及含义 |
|---|---:|---|---|
| 0 | 4 | magic | `0x31425257`，线上字节 `57 52 42 31` |
| 4 | 4 | request_sequence / transmit_timestamp_us | OUT 为主机请求序号；IN 为 MCU 发报时间，单位微秒、u32 回绕 |
| 8 | 1 | version | 固定 `4` |
| 9 | 1 | kind | 消息编号，见下表 |
| 10 | 1 | 按消息定义 | 数量、通道或状态目标；无用途时编码为零 |
| 11 | 1 | 按消息定义 | 长度或状态通道；无用途时编码为零 |
| 12 | 4 | session_id | u32 会话标识；活动会话非零，上下行对应同一会话 |

| kind | 名称 | 方向 | 实际长度 |
|---:|---|---|---:|
| 1 | Hello | 主机 → MCU | 128 |
| 2 | Ready | MCU → 主机 | 64 |
| 3 | CanTransmitRequest | 主机 → MCU | 128 |
| 4 | CanReceiveReport | MCU → 主机 | 16 + 24 × N |
| 5 | UartTransmitRequest | 主机 → MCU | 128 |
| 6 | UartReceiveReport | MCU → 主机 | 64 |
| 7 | ImuSampleReport | MCU → 主机 | 64 |
| 8 | ConfigureSafety | 主机 → MCU | 128 |
| 9 | Arm | 主机 → MCU | 128 |
| 10 | Stop | 主机 → MCU | 128 |
| 11 | Heartbeat | 主机 → MCU | 128 |
| 12 | Result | MCU → 主机 | 64 |
| 13 | Status | 双向 | 请求 128，报告 64 |

## 3. Hello、Arm、Stop、Heartbeat

四种请求仅消息编号不同：偏移 10、11 和 16..127 全部为零。请求序号与会话标识仍位于公共头。

Hello 建立会话并取得 Ready；Arm 在安全配置后授权转发；Stop 显式停止。Heartbeat 有既定编号和空载荷格式，但当前主机不依靠假心跳延续失效的 CAN 控制。格式存在不等于允许任意流量续租，接纳及续租须由会话处理方判断。

## 4. CAN 消息

### 4.1 CAN 记录：16 字节

| 记录内偏移 | 字节数 | 字段 | 约束 |
|---|---:|---|---|
| 0 | 1 | channel | CAN 通道 0..3 |
| 1 | 1 | length | 0..8，经典 CAN |
| 2 | 1 | flags | bit 0 扩展帧；bit 1 远程帧；其他位为零 |
| 3 | 1 | reserved | 必须为零 |
| 4 | 4 | id | 标准帧 ≤ `0x7ff`；扩展帧 ≤ `0x1fffffff` |
| 8 | 8 | data | 始终占 8 字节，length 表示有效长度；编解码会复制全部 8 字节 |

常量 `Extended=1`、`Remote=2`；`UartAction=4` 仅用于安全配置动作，不是普通 CAN 标志。电机协议载荷为不透明字节，本层不区分 DJI、DM 等电机协议。

### 4.2 CanTransmitRequest（kind 3）

| 偏移 | 字节数 | 内容 |
|---|---:|---|
| 10 | 1 | frame_count，当前编解码接纳 0..6 |
| 11 | 1 | 零 |
| 16 | 16 | 零 |
| 32 + 16 × i | 16 | 第 i 条 CAN 记录，i=0..N-1 |
| 32 + 16 × N | 剩余至 128 | 零 |

同一批次允许重复 CAN ID；记录校验与 MCU 的命令白名单校验是不同阶段。零帧批次能通过当前编解码，不应据此推断它能被 MCU 接纳或续租。

### 4.3 CanReceiveReport（kind 4）

偏移 10 为 N（1..6），偏移 11 编码为零；记录从 **16** 开始，每条 **24** 字节：

| 记录内偏移 | 字节数 | 内容 |
|---|---:|---|
| 0 | 16 | CAN 记录 |
| 16 | 4 | receive_timestamp_us，MCU 收帧时间 |
| 20 | 4 | bus_sequence，CAN 总线序号 |

实际长度依次为 40、64、88、112、136、160 字节。MCU 可以把当前已有帧立即上报，不必等满 6 帧。

## 5. UART 消息

逻辑通道固定编号 1..6；通道到遥控、HI91、裁判系统等设备的绑定属于主机配置，不属于线上协议。

### 5.1 UartTransmitRequest（kind 5）

| 偏移 | 字节数 | 内容 |
|---|---:|---|
| 10 | 1 | channel，1..6 |
| 11 | 1 | byte_count，1..96 |
| 16 | 16 | 零 |
| 32 | byte_count | 原始 UART 字节 |
| 32 + byte_count | 剩余至 128 | 零 |

### 5.2 UartReceiveReport（kind 6）

| 偏移 | 字节数 | 内容 |
|---|---:|---|
| 10 | 1 | channel，1..6 |
| 11 | 1 | byte_count，1..40 |
| 16 | 4 | block_sequence，接收块序号 |
| 20 | 4 | receive_timestamp_us，MCU 接收时间 |
| 24 | byte_count | 原始 UART 字节 |
| 24 + byte_count | 剩余至 64 | 零填充 |

块边界不是外设帧边界；外设协议帧可能跨多个报告，也可能在一个报告中出现多帧。512 字节业务消息、发送队列和带宽预算属于主机调度，不是单包 UART 容量。

## 6. 板载 IMU（kind 7）

| 偏移 | 字节数 | 内容 |
|---|---:|---|
| 10 | 1 | 原始数据长度，固定 25 |
| 11 | 1 | 零 |
| 16 | 4 | sample_timestamp_us，MCU 采样时间 |
| 20 | 25 | spi_bytes，原始 SPI 数据 |
| 45 | 3 | 零 |
| 48 | 4 | temperature_c，IEEE 754 binary32，摄氏度 |
| 52 | 4 | sample_sequence |
| 56 | 8 | 零 |

这是板载 IMU 报告，与 UART 上的 HI91 外置 IMU 数据流不同。本层不定义 25 字节内部寄存器的物理量换算。

## 7. Ready（kind 2）和 Result（kind 12）

### 7.1 Ready

偏移 10、11 编码为零。

| 偏移 | 字节数 | 字段 |
|---|---:|---|
| 16 | 4 | capabilities，能力位图 |
| 20 | 4 | request_bytes，当前为 128 |
| 24 | 4 | max_report_bytes，当前为 160 |
| 28 | 4 | can_channel_mask，bit n 对应 CAN n |
| 32 | 1 | max_can_frames，当前为 6 |
| 33 | 1 | protection_frame_count，固件保护帧数量 |
| 34 | 1 | protection_flags，bit 0（值 1）表示固件 CAN 保护能力 |
| 35 | 1 | 零 |
| 36 | 4 | protection_signature，见第 8 节 |
| 40 | 4 | max_lease_us，固件租约上限 |
| 44 | 20 | 零 |

当前主机要求 capabilities 低三位均为 1；诊断将其称为 CAN/remote/HI91 能力。UART 发送按通道检查能力位：UART1→bit1、UART2→bit2、UART3→bit4、UART4→bit5、UART5→bit6、UART6→bit7。当前主机没有单独解释 bit3，本文不为它新增含义。

主机还校验所用 CAN 通道存在、max_can_frames ≥ 6、固件保护 bit0 已置位。主机读取但不比较 protection_signature；不能把签名相等作为当前已有的握手条件。

### 7.2 Result

偏移 10、11 编码为零。

| 偏移 | 字节数 | 字段 |
|---|---:|---|
| 16 | 4 | request_sequence，对应原始 OUT 请求序号 |
| 20 | 1 | request_kind，对应原始消息编号 |
| 21 | 1 | armed，0 或 1 |
| 22 | 2 | 零 |
| 24 | 4 | result_code，i32；0 成功，非零失败 |
| 28 | 36 | 零 |

公共头偏移 4 仍是发报时间，不是被确认请求的序号。Result 表示软件处理结果，不表示电机或 UART 外设已经执行。正常业务成功不逐包确认；主机等待 ConfigureSafety、Arm 的确认，并将业务拒绝视为会话失败。

## 8. ConfigureSafety（kind 8）与保护签名

| 偏移 | 字节数 | 内容 |
|---|---:|---|
| 10 | 1 | uart_stop_count，0..6 |
| 11 | 1 | 零 |
| 16 | 4 | lease_us |
| 20 | 12 | 零 |
| 32 + 16 × i | 16 | 第 i 条 UART 停止动作 |
| 32 + 16 × N | 剩余至 128 | 零 |

UART 停止动作沿用 16 字节记录布局，但 channel 为 1..6，length 为 1..8，flags 固定 **4**，reserved 和 id 为零，data 占 8 字节。ConfigureSafety 的具名解码器只接纳 UART 动作；不能通过该消息替换 MCU 内建 CAN 保护表。count 为零仍选择 MCU 内建 CAN 保护，不表示清空保护。

编码器及 SessionPolicy 要求租约为 1000..1000000 微秒；固件还可施加 Ready 所报的更小上限。当前 ConfigureSafety 解码器本身没有检查租约范围，由后续会话策略检查，重构时应保留这一阶段划分。

保护签名计算输入为：默认租约的 4 个小端字节，随后按配置顺序拼接各条编码后的 16 字节 CAN 保护记录。CRC 初值 `0xffffffff`，逐字节异或后按低位迭代，多项式 `0xedb88320`，结束异或 `0xffffffff`。无记录数量前缀。签名不覆盖反馈路由，不提供认证，也不是每条消息的校验和。

## 9. Status（kind 13）

请求偏移 10 为 target：Session=0、Can=1、Uart=2；偏移 11 为 channel；16..127 为零。当前请求编解码仅限制 target≤2，不检查 channel 范围；具体通道可用性由处理方判断。

三种报告均为 64 字节，公共头偏移 10 为 target，偏移 11 为 channel。下表未使用字节编码为零。

### 9.1 SessionStatusReport（target 0）

| 偏移 | 字节数 | 字段 |
|---|---:|---|
| 16 | 1 | armed |
| 17 | 1 | configured |
| 18 | 1 | stop_reason |
| 19 | 1 | 零 |
| 20 | 4 | rejected_requests |
| 24 | 4 | can_report_drops |
| 28 | 4 | sensor_report_drops |
| 32 | 4 | stop_errors |
| 36 | 4 | reply_drops |
| 40 | 4 | usb_send_errors |
| 44 | 4 | usb_receive_drops |
| 48 | 4 | usb_endpoint_errors |
| 52 | 4 | protection_signature |
| 56 | 4 | max_lease_us |
| 60 | 4 | 零 |

stop_reason 编号：None=0、Explicit=1、LeaseExpired=2、UsbLost=3、TransportError=4、Replaced=5。

### 9.2 CanStatusReport（target 1）

从偏移 16 起，每项 u32，依次为：

| 偏移 | 字段 |
|---|---|
| 16 | received |
| 20 | submitted |
| 24 | completed |
| 28 | errors |
| 32 | dropped |
| 36 | queue_depth |
| 40 | controller_state |
| 44 | tx_errors |
| 48 | rx_errors |

52..63 为零。controller_state 在当前共享定义中仅是 u32，本文不额外规定控制器状态枚举。

### 9.3 UartStatusReport（target 2）

从偏移 16 起，每项 u32：received_blocks（16）、dropped_blocks（20）、completed_writes（24）、tx_errors（28）。32..63 为零。

当前主机接收 Status 后不送往传感器解析，也没有这三种状态报告的具名解码接口；报告格式以现有编码器为依据。

## 10. 校验与交互行为基准

### 校验边界

- 公共头检查最小 16 字节、magic 和 version；它不负责检查会话、消息载荷或实际 USB 长度。
- OUT 具名解码器接收固定 128 字节数组。CAN/UART/安全配置检查数量、有效记录、规定保留区及末尾填充；空控制请求和 Status 检查其规定零区。
- CAN 记录 offset 3 非零、长度超过 8、通道或 ID 越界、未知 CAN 标志均拒绝。
- IN 实际长度由 USB 回调先验证，不能仅因数组容量 160 就判为有效。CAN N=0 或 N>6 无有效报告长度；未知上行 kind 无有效报告长度。
- 主机 IN 解码器检查对应 kind；CAN 检查记录，UART 检查通道及长度，IMU 检查长度字节等于 25，Result 检查 armed≤1。Ready 的能力约束由会话层另行检查。
- 当前 IN 具名解码器没有普遍拒绝非零保留区；Result 解码器也没有穷举验证 request_kind，等待确认的会话逻辑才匹配其类型。
- 接收解码失败不向调用方提交部分消息。不是本会话的上行不进入当前会话处理。

### 会话、序号和时间

1. 主机选择非零 u32 session_id，预投递接收，然后发送 Hello；当前 Hello 请求序号为 0。MCU 返回同一会话的 Ready。
2. 主机发送 ConfigureSafety，等待成功 Result；再发送 Arm 并等待成功 Result，且报告 armed 必须为真。Adapter Arm 不等于电机 Enter 或整机输出许可。
3. SessionPolicy 在未 Arm 时配置租约；Arm 要求已配置、尚未授权和新序号，并从当前 MCU 时间开始计租。Observe 只记录序号，不续租。
4. 序号新旧判断为 `int32_t(next - previous) > 0`，支持 u32 回绕，只在有符号半区间内定义“更新”。未见过序号时 Fresh 为真。
5. Accept 要求已授权、序号更新且租约未过期；成功后更新序号与收包时间。Expired 判定为 `uint32_t(now - last_receive) >= timeout`，等于期限即过期。
6. Close 清除会话、配置、序号和授权。超时后不靠迟到业务自动恢复；主机失败后不自动重新 Arm。
7. MCU 独立维护 CAN 白名单、反馈过滤及保护表。停止、租约超时、断连等触发保护的部署行为见 [USB 会话说明](USB.md)。这些设备配置不由本层电机载荷自动推导。
8. IN 的发报时间和采样时间均为 MCU 的 u32 微秒时钟。主机按回绕减法计算采样年龄，再从本机到达时间减去年龄；不假定两端时钟同步。主机时间是估计值，不是新增线上字段。

当前主机选择最多 20000 微秒租约（与 Ready 上限取较小值），并使用 20000 微秒作为报告健康及采样年龄策略。四个 IN 预投递、握手等待时限、线程优先级、队列长度、UART 带宽预算均为主机实现策略，不应误写成 USB v4 格式限制。

## 11. 重构时如何核对

- 以本文件核对编号、偏移、类型、容量、实际传输长度、清零规则、校验边界及会话语义。修改文件结构本身不构成协议变更。
- `tests/protocols/adapter_protocol.cpp` 覆盖字段位置、边界、上行解码及租约回绕；另有 `usb_wire_vectors.cpp` 对照重构前冻结的 `usb_v4_vectors.txt`，核对所有消息类型的实际长度、完整缓冲区及保护签名。固定样例不能穷举所有输入，不能只凭自编码再自解码就证明兼容。
- `tools/check_usb_protocol.py` 编译主机协议并核对冻结字节；提供 MCU checkout 时，还用同一组用例编译 MCU 来源作对照。它不再要求源码相同，也不会修改 MCU 文件。此检查只覆盖列出的报文，不证明完整固件行为。
- 后续重构应保持独立的固定字节用例与当前协议兼容；若发现文档与实现有出入，先确认差异，不能借整理代码静默修改线上行为。

本次记录的来源：`src/hardware/usb/protocol/usb_protocol.hpp`、`usb_protocol_codec.hpp`、`usb_report_decoder.hpp`、`usb_session_policy.hpp`，以及 `usb_link.cpp`、`usb_receive.cpp`、`usb_transmitter.hpp` 的主机策略。

实现位置更新：USB 精修后，以上来源的协议定义、收发编解码及 SessionPolicy 集中于 `usb_protocol.hpp/.cpp`。最初记录的字节格式保持不变；初始来源文件名保留用于追溯。

命名空间更新：主机协议现为 `dust_control::wire`；原 `dust::wire` 来源仍可通过协议检查工具按相同字节用例核对。命名空间不是线上字段，此调整不改变本文件记录的通信约定。
