# RMCS、librmcs 与 dust-control 通信架构对比

核对日期：2026-10-26。范围是设备接入、协议与传输边界、业务依赖和迁移 EtherCAT 的影响；不是性能测试或完整代码审计。本次只新增分析文档，没有修改产品实现。

## 版本范围

- RMCS：用户指定的 main，本次读取提交 `a27e3d14c36ef8d780287e154b32ed4a70bbbdc9`。
- librmcs：用户指定的 `v3.3.0b1`，提交 `731d266e4f0ffff2608a2b77b434df545cf8dfca`。
- RMCS 的 rmcs_core/CMakeLists.txt 本身引用 v3.3.0b1 的 SDK 源码发布包，并固定 SHA256，两个指定版本确有关联。[构建定义](https://github.com/Alliance-Algorithm/RMCS/blob/a27e3d14c36ef8d780287e154b32ed4a70bbbdc9/rmcs_ws/src/rmcs_core/CMakeLists.txt)

## 核心结论

RMCS 将控制组件与具体板卡隔离，librmcs 将板卡消息与底层 USB 实现分开。但它们有真实的分层、接口分派和对象组合，不是没有抽象的设计。值得借鉴的是职责边界和依赖方向，不是完整复制其类层次。

librmcs v3.3.0b1 有通用 Transport 接口，但公开 Handler 构造仍接收 USB VID/PID，并固定调用 usb::create_transport；所核对源码树中没有 EtherCAT 后端。不能据此认为其已经支持通过配置无改动替换 EtherCAT。[Handler 实现](https://github.com/Alliance-Algorithm/librmcs/blob/v3.3.0b1/host/src/protocol/handler.cpp#L422-L426)

## 实际调用与所有权

### librmcs

发送路径可概括为：

```text
硬件组件
  → RmcsBoardLite / Pro 的 PacketBuilder
  → Handler 的 PacketBuilder 与序列化实现
  → Transport 虚接口
  → Usb / libusb
```

接收由 Transport 提供字节回调，Handler 的反序列化器拆出数据，交给板卡 Callback；板卡通过描述符识别 CAN/UART 通道，再回调使用方。接收 span 只在回调期间有效；传输缓冲由创建它的 Transport 回收，所有权规则在接口中明确规定。

依据：[板卡接口](https://github.com/Alliance-Algorithm/librmcs/blob/v3.3.0b1/host/include/librmcs/board/rmcs_board_lite.hpp)、[Transport](https://github.com/Alliance-Algorithm/librmcs/blob/v3.3.0b1/host/src/transport/transport.hpp)、[协议 Handler](https://github.com/Alliance-Algorithm/librmcs/blob/v3.3.0b1/host/src/protocol/handler.cpp)。

这些层各有职责，但全部照搬会比本项目当前结构引入更多对象和接口。其 Transport 面向缓冲区发送与字节回调，这个接口本身也不等于已解决 EtherCAT 周期过程数据映射。

### RMCS

以 OmniInfantry 为例：硬件组件直接拥有电机、IMU、遥控等设备对象，继承板卡 Callback，按 CAN 通道及 ID 将数据送入相应设备。命令阶段由设备生成载荷，再调用板卡 builder 发送；设备对象不持有 USB 发送器。[硬件接入](https://github.com/Alliance-Algorithm/RMCS/blob/a27e3d14c36ef8d780287e154b32ed4a70bbbdc9/rmcs_ws/src/rmcs_core/src/hardware/omni_infantry.cpp)

DjiMotor 使用原子原始 CAN 数据暂存，接收时 store_status，周期更新时 update_status 转换物理量。它不依赖 libusb，却依赖 Component 注册输入/输出，因此不能称为完全独立于框架的设备协议类。[DjiMotor](https://github.com/Alliance-Algorithm/RMCS/blob/a27e3d14c36ef8d780287e154b32ed4a70bbbdc9/rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp)

控制组件通过具名、带类型的 InputInterface / OutputInterface 交换数据。普通输入接口在配对后通过指针访问绑定数据，不是每周期通过 ROS 话题序列化传输。Executor 根据组件依赖建立执行顺序。[组件接口](https://github.com/Alliance-Algorithm/RMCS/blob/a27e3d14c36ef8d780287e154b32ed4a70bbbdc9/rmcs_ws/src/rmcs_executor/include/rmcs_executor/component.hpp)、[Executor](https://github.com/Alliance-Algorithm/RMCS/blob/a27e3d14c36ef8d780287e154b32ed4a70bbbdc9/rmcs_ws/src/rmcs_executor/src/executor.hpp)

多板示例 DeformableInfantryOmni 显式拥有 BottomBoard 和 TopBoard，分别使用 serial_filter。回调属于具体板卡对象，再判断通道与 CAN ID；相同通道和 ID 不会脱离板卡上下文单独作为全局身份。[多板接入](https://github.com/Alliance-Algorithm/RMCS/blob/a27e3d14c36ef8d780287e154b32ed4a70bbbdc9/rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-omni.cpp#L65-L68)

## 与当前项目逐项比较

| 关注点 | RMCS / librmcs 的实际做法 | dust-control 当前情况 | 判断 |
|---|---|---|---|
| 控制算法 | 控制组件读写具名数据接口 | 纯控制器使用 ControllerInputs / MotorTargets | 已有良好边界，无需复制整个 Executor |
| 业务运行时 | 硬件组件承担板卡调用和设备路由 | ChassisRuntime 直接配置、查询和调用 UsbTransmitter | 这是当前最优先改善的耦合 |
| 设备协议 | 设备不持有 USB；部分设备依赖 Component | 电机、IMU、遥控、裁判解析不依赖 USB 或 ROS | 当前独立性值得保留 |
| 设备所有权 | 硬件组件拥有设备和板卡对象 | UsbLink 直接拥有多个设备解码器 | 应将设备归装配对象，USB 借用并交付数据 |
| 传输抽象 | Transport 虚接口，Usb 实现 | UsbLink 直接管理 libusb | 可借鉴边界，但无需立即增加一个字节传输抽象层 |
| 业务提交端口 | 上层通过数据接口隔离板卡 | CanCommandPort / UartFramePort 转发给具体 UsbTransmitter | 可以改造已有端口，替换转发，而不是叠加包装 |
| 多板身份 | 板卡对象及 serial_filter 提供上下文 | 每台电机 adapter + bus + ID，按板生成子配置 | 两者思路一致，应保留当前显式 YAML 归属 |
| 路由配置 | 所读硬件示例中大量通道、ID 和设备组合在 C++ 中明确编排 | 电机与传感器绑定可由 YAML 配置 | 不必照搬示例中的硬编码接线 |
| 线程交接 | 示例设备原子暂存原始数据，周期更新物理量 | 接收线程解析，PublishedSnapshot 发布完整观测 | 都需要线程所有权约束，不能只搬函数名字 |
| 超时 | Handler 有独立会话 keepalive 线程 | 每控制来源期限、目标生成时间及会话保护 | 语义不同，不能用会话存活替代控制目标新鲜度 |

当前耦合证据：

- `src/chassis/chassis_runtime.hpp`：ChassisMotorAdapter 保存 UsbTransmitter 指针。
- `src/chassis/chassis_runtime.cpp`：注册 CAN 组、提交帧、检查发送器状态，引用 USB 模块中的期限常量。
- `src/hardware/usb/usb_transmitter.hpp`：两个业务端口直接保存具体 UsbTransmitter 引用。
- `src/hardware/usb/usb_link.hpp`：USB 会话直接拥有电机及多个传感器解码器。
- `src/dust_control.hpp`、`dust_control_config.hpp`：装配及 Adapter 配置使用具体 USB 类型。

## 不增加包装层的整改建议

### 第一阶段：改造已有端口

将现有 CanCommandPort、UartFramePort 改成与 USB 无关的能力接口，由 UsbTransmitter 直接实现；业务依赖接口，DustControl 仍决定注入哪个实际对象。

```text
当前云台路径：业务 → 端口转发对象 → UsbTransmitter
调整后路径：  业务 → 接口调用 → 同一个 UsbTransmitter 对象
```

抽象接口只规定提交、启动前注册和必要状态查询，不另建一层 TransmitterAdapter。没有新增线程、队列或代理对象；虚函数调用与增加转发对象是不同的事情。

底盘使用相同接口后，不再包含 USB 发送器头。端口模型与公共 CAN 数据归硬件公共接口，业务运行时的链接依赖也从 dust_usb 改成实际使用的设备库与公共接口。具体发送线程、缓冲容量和 USB 批次限制继续归 USB 后端。

### 第二阶段：调整已有对象的职责

- 将设备解码器的所有权移至现有 Adapter 装配对象；USB 接收方借用设备，不新增 DevicesManager 或路由代理。
- 保留 Consume / Read 及现有快照结构；协议字节、分派身份、采样时间和断流通知保持明确。
- 控制期限独立配置；USB 会话租约由 USB 实现消费。避免业务为了获取常量而依赖 USB 模块。
- DustControl 作为装配处保留具体后端知识。若未来支持混合后端，再在此处选择实现，不把后端选择散落到控制循环里。

这借鉴了 RMCS 的硬件/控制分工，但不引入其插件、具名接口注册和依赖图 Executor，因为当前项目已有明确的模块和静态数据结构。

### EtherCAT 的边界

上述接口适用于 EtherCAT 从站继续桥接当前 CAN/UART 的情形。若直接接入原生 EtherCAT 电机，CAN 端口就不是恰当的统一边界，应使用现有 MotorTargets / MotorFeedback 表达设备意图和物理反馈，而不是强制把 PDO 伪装成 CAN。

即便上层接口相同，EtherCAT 主站周期、PDO 布局、工作计数器、时钟同步及从站保护仍需独立设计；抽象接口不会自动完成这些工作。

## 后续维护决策

用户已明确 USB 和 EtherCAT 分别维护，不要求同一运行时切换后端。因此不执行下文
基于可替换目标提出的完整端口抽象和设备所有权迁移。源码对比结论仍保留；整改聚焦
现有职责问题，控制算法与设备协议保持独立，具体运行时和装配允许依赖对应传输实现。

## 此前以可替换后端为目标的建议

优先落实已有发送端口的去 USB 绑定、期限归属和构建依赖，再调整设备所有权。继续保留按电机配置的 Adapter 身份、独立设备协议和现有快照模型。RMCS 是职责组织的参考，不是需要整体迁入的框架；librmcs 的 Transport 是可扩展边界的实例，不是现成 EtherCAT 支持的证明。
