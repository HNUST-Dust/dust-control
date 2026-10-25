# 架构与数据流

应用通过 `src/main.cpp` 启动一个 ROS2 节点。Robot 负责读取启动配置、拥有资源、
装配模块和协调启停；底盘、云台和 UI 各自拥有业务线程，每块活动 Adapter 拥有
独立 USB 会话及收发线程。控制算法和设备协议保持独立于 ROS2。

## 模块与所有权

```text
main → DustControlNode → Robot
                         ├─ ChassisRuntime → 底盘控制线程
                         ├─ GimbalRuntime  → 云台线程
                         ├─ UiRuntime      → UI 线程
                         └─ 按名字拥有 UsbSession
                            ├─ UsbLink → USB 事件与接收线程
                            ├─ 各设备 Decoder → 最新状态快照
                            ├─ 原始 UART 队列
                            └─ UsbTransmitter → USB 发送线程
```

| 位置 | 职责 |
|---|---|
| `src/dust_control_node.*` | 配置目录参数、ROS 观测转换与发布、模块状态检查 |
| `src/robot.*`、`src/robot_config.*` | YAML 装配关系、会话所有权和整机生命周期 |
| `src/chassis/chassis_runtime.*` | 读取输入、调用算法、编码并提交底盘命令 |
| `src/chassis` / `dust_chassis` | 状态机、控制算法、运动学与只读数值配置 |
| `src/gimbal`、`src/ui` | 各自的业务线程及借用接口，目前为空实现 |
| `src/hardware/usb` | 设备发现、会话、报告分发与发送调度 |
| `src/hardware/motors` | DJI、DM、CubeMars 协议、电机映射、反馈和命令编码 |
| `src/hardware/remote` | 遥控协议、SBUS 拼帧与输入映射；保留 DR16、VT03、WFLY 支持 |
| `src/hardware/imu`、`src/hardware/referee` | IMU 与裁判协议、拼帧、恢复状态和快照 |
| `src/concurrency` | 有界快照、优先级继承锁与线程调度配置 |
| `src/yaml_validation.hpp` | 配置加载器共用的字段与数值校验辅助函数 |

模块借用 Decoder 和发送端口，不创建或关闭共享 USB 会话。每块板的设备状态、
序号和发送调度独立，同板模块共用收发线程。默认底盘和 UI 绑定 base，云台离线。
没有真实 CAN 生产者的会话保持空闲，不打开板卡或 Arm。配置方法与启动限制见
[运行配置](ROBOT_CONFIGURATION.md)。

## 接收与控制

```text
USB IN → UsbLink 校验报告、会话和序号，换算时间戳
         ├─ CAN       → MotorFeedbackDecoder
         ├─ 绑定 remote 的 UART → SbusDecoder
         ├─ 绑定 hi91 的 UART → Hi91Decoder
         ├─ 绑定 referee 的 UART → RefereeDecoder
         └─ 板载 IMU  → OnboardImu

Decoder 最新快照 → ChassisRuntime → ControllerInputs → controller.Tick()
```

设备解析器各自维护拼帧、恢复状态和最新快照；解析状态由接收线程独占，Read()
通过优先级继承互斥锁返回状态副本。快照锁只保护数据复制，不用于协议解析。
不同设备不是原子采样，底盘输入层分别检查时间戳和有效期限。

六路原始 UART 块另由 UartReceiveQueue 保存，未绑定通道的消费者自行解析。
新会话 Ready 重置设备状态、拼帧缓存和原始队列；静默遥控仍会触发超时处理。
裁判解析目前只检查帧边界，不校验 CRC8/CRC16，不参与控制许可或功率限制。
板载 IMU 不覆盖控制用 HI91。通道细节见 [UART 接口](protocols/UART.md)。

当前底盘运动输入来自物理遥控，暂不接入外部规划目标。控制线程不调用 ROS、
不解析 YAML、不枚举 USB。算法只接收数值输入和执行器目标，不处理协议字节。
设备的被动数据类型归各设备模块，ControllerInputs 归底盘；没有集中 model 目录。
DR16、VT03、CubeMars 协议虽然尚未接入当前设备链路，仍保留并参与编译。

## 编码与发送

```text
controller.Tick() → MotorTargets → MotorEncoder.Encode()
    → UsbTransmitter.SubmitCan() → 对应板的发送线程 → UsbLink → USB OUT

UI 编码完整消息 → UartFramePort → 同一发送线程 → USB OUT
```

MotorEncoder 返回拥有载荷的完整 CAN 批次，不持有传输资源。编码失败停止底盘，
不提交部分结果或复用上一周期命令。每个 CAN 来源启动前注册完整
(bus, command_id) 命令组，同一会话内每组只有一个所有者，包括 DJI 帧内全部槽位。
发送层不合并电机槽位，也不理解电机协议。

CAN 提交线程安全，每来源最多 16 帧，最新批次覆盖尚未发送的旧批次。单个 USB
请求最多 6 帧；跨请求不保证 MCU 原子执行，已经进入 libusb 的请求无法撤回。
生成时间与递增序号用于拒绝过期或倒退目标。空批次只提交一次，不周期重放续租。

UART 提交完整消息并复制数据，最多 512 字节、FIFO 最多 16 条，满时返回 false。
未开始的过期消息丢弃，开始后按顺序发完，分块最多 96 字节，CAN 在块间优先。
默认 UART 总预算 8000 字节/秒，为 CAN 预留两个 USB OUT 槽位。
提交成功只表示主机接收，不表示 MCU 确认或 UART 已发送完成。

每个注册 CAN 来源独立检查期限，最大 20 ms；任一来源超时即停止整个 USB 会话。
其他控制来源或 UI 流量不能掩盖停顿。没有独立续租心跳，不自动重新 Arm。
主机与 MCU 的会话保护边界见 [USB 协议](protocols/USB.md)。

## 观测与 ROS2

底盘每控制周期尝试向预分配的 ChassisObservationQueue 压入观测；队列满时丢样
并计数，不等待 ROS。ROS 每 1 ms 有界消费，只发布最近观测，不补发旧样本。
高频状态使用 KeepLast(1) / BestEffort / Volatile；IMU、关节和诊断以较低频率发布。
具体字段、频率和时间语义见 [ROS2 接口](ROS2.md#ros-接口)。

Foxglove 使用独立 Bridge 消费 ROS2 话题，与其他节点共用一个可视化入口。
没有额外遥测线程、UDP 调试出口或原生 Foxglove SDK。

## 配置与生命周期

Robot 在创建线程前加载 robot.yaml 及所需设备、底盘文件，完整校验后再装配。
配置加载器归使用模块；算法接收只读数值副本，不依赖 YAML。稳定算法参数和协议
规格保留在所属代码中；不生成参数头，不提供全局配置或热更新。

底盘从每台电机的 adapter 归属推导需要借用的板，汇集反馈并分发命令；
启动时从唯一电机文件生成每板设备子集。遥控与 HI91 取自 chassis.sensor_adapter。
底盘线程先运行，板就绪后立即提交安全命令，避免等待下一块板握手时耗尽本板租约。
仅关节板在安全状态下每 10 周期重新生成失能命令，以满足 20 ms 租约。
全部所需会话处理成功前，底盘的整机启动许可保持关闭；以后云台实际输出也必须
沿用该许可。Arm 成功不等于电机使能，实际输出还受 YAML、遥控和状态机门控。

退出先撤销整机输出许可，停止并回收全部业务线程，向所有发送器请求停止，再等待并回收各板的发送与接收资源，避免顺序等待耗尽其他板租约。
部分启动失败采用相同清理顺序。任一已启动会话或业务线程故障，整体停止。
Robot 本身没有工作线程，业务模块不能停止借用的会话。

## 开发边界

MPC 作为独立在研算法模块保留，默认关闭，尚未接入正式控制路径，见 [MPC](MPC.md)。
EtherCAT 仅在开发分支维护，见 [分支约定](BRANCHES.md)。控制单位沿用现有约定：
轮速度 rpm、IMU 姿态 deg、关节 rad/rad/s/N·m，避免结构调整同时改变算法。

`architecture.dependencies` 检查模块引用和链接边界。OrbStack 的功能回归不替代
实机多板、时序或吞吐验收；测试范围见 [测试说明](TESTING.md)。

设备接收路由由每块 Adapter 的 `uart_sensors` 配置决定，默认 1/2/3，详见装配说明。
电机接线由 `motors_params.yaml` 每台具名设备的 adapter、bus 和 ID 配置；protocol 指定通信协议，
设备层按配置执行反馈解码、命令编码及 ID 路由。设备排列顺序不影响底盘角色映射。
控制层使用固定的六路反馈/目标数组，顺序为左轮、右轮、左 B/D、右 B/D，不另设角色访问包装。

## 电机协议文件组织

每种电机协议集中在对应的 `dji_protocol.h/.cpp`、`dm_protocol.h/.cpp`、
`cubemars_protocol.h/.cpp` 中。DJI/DM 文件包含原始报文、固定型号规格、
执行器目标编码和物理反馈换算；CubeMars 保留现有独立报文编解码，尚未接入底盘。
`motor_protocol.hpp/.cpp` 仅负责按配置分派协议、组装 CAN 批次、匹配本板反馈及发布快照；
不持有 USB 资源。`motor_config.*` 负责 YAML 加载、设备归属和校验。
MotorFeedback、MotorTarget、MotorTargets、MotorMode 和 ActuatorId 集中在 motor_protocol.hpp，
底盘输入与控制输出通过公共头使用这些数据类型。架构检查禁止控制算法调用电机编解码和快照服务。
离线默认轮力矩预算保留
为控制层数值常量，正式运行由设备层从协议规格计算并注入。

MotorAddress 属于 motor_config.hpp 中的接线配置数据。生产代码没有默认电机地址表，
默认配置表示没有设备；Robot 必须读取电机 YAML。离线回归的固定接线只在
tests/support/motor_fixture.hpp 中提供，测试调用方显式传入。
