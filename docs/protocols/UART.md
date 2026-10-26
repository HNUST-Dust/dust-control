# UART 通道与设备接口

当前结构和线程规则见 [架构说明](../ARCHITECTURE.md)。

## 接收

UsbLink 校验报告、会话和块序号，换算主机时间后按每块 Adapter 的 `uart_sensors` 绑定分发。以下为默认接线：

| 逻辑通道 | 消费者 |
|---|---|
| UART 1 | WflySbusDecoder |
| UART 2 | Hi91Decoder |
| UART 3 | RefereeDecoder |
| 未绑定通道 | 原始块消费者自行解析 |

各 Decoder 独占自己的拼帧与恢复状态，并通过线程安全 Read() 返回快照副本。
控制线程每周期读取所需设备，再组合 ControllerInputs；不同设备不是原子采样。
裁判解析当前只检查帧边界，不校验 CRC8/CRC16，不参与控制许可或功率限制。
板载 IMU 由 OnboardImu 单独保存，不覆盖控制用 HI91。

六路原始块由 UartReceiveQueue 管理：TakeUartChunk(channel, chunk) 取走最早记录，
多消费者分摊记录，不是广播。每通道最多 32 块，每块最多 40 字节；队列满丢弃
最旧块、累加丢失计数，并标记 discontinuity。原始块边界不是传感器帧边界。
Ready 重置设备状态、拼帧缓存、原始队列和计数；静默遥控通道仍会触发超时处理。

## 发送

业务线程先编码完整消息，再调用 UsbTransmitter::SubmitUart(channel, bytes,
generated_ns, lifetime_ns)。方法线程安全并复制数据；消息最多 512 字节，队列
最多 16 条，满时返回 false。生成时间使用主机单调纳秒，默认有效期 200 ms。

每块活动 Adapter 的 USB 发送线程按 FIFO 分块为最多 96 字节的请求；未开始的过期消息丢弃，
已开始消息保持顺序发完，CAN 可在块间优先发送。默认总预算 8000 字节/秒。
提交成功不表示 MCU 确认或 UART 已完成发送。六路写能力须由 Ready 确认。
任一已注册 CAN 来源超时，整个 USB 会话停止，UART 流量不能延续失效控制。

## 配置与协议维护

主机电机映射由 dust_control.yaml 根节点 motors 指定，各电机用 adapter 指定归属，默认使用
config/motors_params.yaml；传感器期限来自 config/sensors_params.yaml。MCU 自行维护 CAN 白名单和保护帧。新增设备需同步
相关主机参数及 MCU 配置。协议实现位于 src/hardware/usb/protocol/；
文件组织独立于 MCU，以下命令核对固定报文字节：

```bash
python3 tools/check_usb_protocol.py /path/to/dust-adapter
```

具名消息使用 Encode/Decode，当前只接受 USB v4。上行 CAN 可变长度说明见
[USB v4 CAN 报告](USB.md#can-报告)。测试及证据边界见
[验证记录](../history/VALIDATION.md)。自动回归均为离线/mock。
