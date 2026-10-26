# 架构与数据流

main → DustControlNode → DustControl。ROS 发布观测；DustControl 加载 YAML、装配并协调启停。
底盘、云台、UI 保留各自线程，通信统一交给一个 EthercatMaster 周期线程。

```text
DustControl
├─ ChassisRuntime / GimbalRuntime / UiRuntime
├─ 按名字拥有 EthercatSession
│  ├─ 原设备 Decoder 与快照
│  ├─ 原始 UART 接收队列
│  └─ EthercatTransmitter（有界来源/消息缓冲，无独立线程）
└─ EthercatMaster（唯一 IgH 线程）
   └─ IghTransport → 动态 PDO / domain → 1～3 块从站
```

接收：IgH Receive/domain_process → 校验 OP/WKC → 每板业务头/会话校验 →
CAN/UART/原始 IMU → 现有设备 Consume/Read。控制读取快照并生成 MotorTargets，
MotorEncoder 编成 CAN 后提交本板来源缓冲；主站每周期编码最新有效来源并 Send。

业务不拥有主站，不调用 ecrt，不停止共享会话，不在循环解析 YAML。
控制器与设备编解码保持独立于 ROS/IgH；本分支直接使用具体 EtherCAT 端口，
没有额外 USB/EtherCAT 后端切换层。

先完整加载和校验配置，再创建会话、注册命令组、启动底盘安全计算，最后启动主站。
全板握手完成前 started/startup_ready 门控阻止业务输出；全板 ready 后仍需原遥控与状态机条件。
空业务模块不伪造 CAN 来源；配置的从站统一交换诊断/传感器，不通过虚假控制心跳续租。

退出先取消许可、回收业务线程，再统一发送全板 Stop 并释放主站。
丢线、从站复位或来源失效终止整机会话；不在运行中自动重新 Arm。

配置与协议详见 [装配](DUST_CONTROL_CONFIGURATION.md)、[运行时](ETHERCAT_RUNTIME.md)、
[PDO 配置](ETHERCAT_PDO_CONFIGURATION.md)。
