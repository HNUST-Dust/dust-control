# USB v4 协议与会话

主机通过 USB Bulk 连接 dust-adapter，主机与固件必须使用配套的 USB v4 协议。
逐字节格式和交互基准见 [USB v4 线上协议基准](USB_V4_WIRE.md)。
定义与实现当前位于 `src/hardware/usb/protocol/`；保持线上格式和语义兼容的主机重构
不需要同步修改 MCU。当前协议实现集中在 `usb_protocol.hpp/.cpp`，不再要求
与 MCU 保持相同的源码文件组织。可执行固定字节基准检查：

```bash
python3 tools/check_usb_protocol.py
# 可选：用同一组用例编译 MCU 来源，对比报文字节；不修改或刷写 MCU。
python3 tools/check_usb_protocol.py /path/to/dust-adapter
```

基准记录在 `tests/protocols/usb_v4_vectors.txt`，由重构前实现生成，覆盖全部
消息类型、CAN 批次数量、六路 UART、安全动作数量及保护签名。
这验证所列报文的字节兼容性，不替代完整固件行为或实机验证。
具名消息使用 Encode/Decode，包缓冲区使用 UsbOutRequest、UsbInPacket，
不直接发送 C++ 结构体内存。旧版本不兼容，不提供自动回退。

## 板卡发现

默认枚举要求如下：

| 项目 | 要求 |
|---|---|
| VID | `0x34b7` |
| manufacturer | `Dust` |
| product | 默认 `dust-hpm6750`，可按 Adapter 配置 |
| 速度 | High Speed |
| 接口 | vendor interface 0 / alternate 0 |
| Bulk 端点 | OUT `0x01`、IN `0x81`，MPS 512 |

PID 由完整 MCU UID 的 CRC16-CCITT-FALSE 得到，可能为 0 或 0xffff，也可能冲突，
不能作为唯一板卡标识。HPM6750 的序列号为完整 32 位大写十六进制 UID。
`robot.yaml` 中 serial 精确匹配，区分大小写，不接受前缀；多板要求不同的完整序列号。
可选 pid 只是附加筛选条件，默认不限制 PID。

多个候选会报错，不任意取第一块。无法读取可能匹配的设备时，也不会静默跳过权限
或字符串错误。日志包含总线、地址、VID/PID、manufacturer、product 和完整序列号。
多板配置见 [运行配置](../ROBOT_CONFIGURATION.md)。

## 会话与保护

启动时预投递四个 IN 请求，检查 Ready 的固件保护能力和所需 CAN 通道。
主机用 ConfigureSafety count 0 选择 MCU 内建保护表，租约不超过 20 ms；等待确认，
再显式 Arm 并等待确认。配置、Arm 或业务数据被拒绝均使会话失败，不自动重新 Arm。
Adapter Arm 不是电机 Enter，也不是整机输出许可。

MCU 独立维护 CAN 反馈过滤、命令白名单和默认保护帧，在断连、租约过期、显式 Stop，
以及默认启动路径执行保护。它不解析电机载荷，不运行底盘算法。
Ready 提供保护表数量、默认租约和 CRC32；主机要求保护能力，但不比较保护表签名，
也不保存 MCU 设备表副本。主机可缩短租约，不能延长固件上限。

主机按本板电机 YAML 解码和编码，MCU 独立执行白名单。新增设备或地址需要同步修改
主机映射与 MCU 配置，不能只改一侧。主机保护与发送调度见
[架构](../ARCHITECTURE.md#编码与发送)。

只绑定空云台/UI、没有真实 CAN 生产者的会话保持 idle，不打开或 Arm。
纯接收板、UART 单独运行的会话方式还需要随真实模块和固件语义接入，不能用假 CAN
心跳绕过租约。

## CAN 报告

| 方向或类型 | 长度与容量 |
|---|---|
| 所有 OUT 请求 | 固定 128 字节，包含 Hello、Stop |
| CAN OUT | 每记录 16 字节，每请求最多 6 帧 |
| CAN IN | `16 + 24 × N` 字节，N 为 1～6 |
| 其他 IN 报告 | 固定 64 字节 |
| 主机 IN 缓冲与投递长度 | 160 字节 |

CAN 上行实际长度为 40/64/88/112/136/160 字节，每条记录包含时间戳和序号。
MCU 取当前已有帧，不等满六帧。回调按类型和帧数严格检查实际长度；
Ready 偏移 24 的字段声明最大上行长度 160。没有周期性业务成功 ACK。
UART 的通道、原始块和发送规则见 [UART 接口](UART.md)。

## 权限与验证范围

当前部署通过 sudo 获取 USB 与实时调度权限；开机启动可使用 root systemd 服务，
ExecStart 直接执行节点，不在服务中嵌套 sudo。运行参数仍来自 YAML。

自动回归使用 mock，覆盖身份筛选、握手、拒绝、重连、序号和租约边界；
不等同于实际板卡验收。历史 stress/echo 工具有自己的固件协议和身份，不能用于
证明当前 USB v4 已通过硬件验收。测试方法见 [测试说明](../TESTING.md)。
实机仍需验证描述符、重启后的身份稳定性、多板选择、断连保护、UART 回环与并发负载。
