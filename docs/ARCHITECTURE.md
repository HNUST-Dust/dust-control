# 模块边界与运行规则

`dust_chassis` 依赖公共物理量模型、Eigen、生成参数和只读运行配置；不依赖设备
协议、USB、EtherCAT、Foxglove 或 Linux 线程。设备目录拥有电机反馈、执行器目标、
遥控状态和 IMU 样本的被动数据定义；底盘仅引用这些头文件，不链接设备实现。
`src/chassis/controller_inputs.h` 汇总底盘输入，不携带 CAN 字节。
`src/hardware/can_frame.hpp` 定义设备编码器与通信实现共用的 CAN 帧。
板载 IMU 和裁判系统快照由各自 Decoder 头文件拥有；不设集中 model 目录。
`Tick` 的可选同步观测快照不进行 I/O。
`architecture.dependencies` 回归检查控制层、公共模型和设备层的源码引用与
控制库链接依赖，防止重新引入设备协议或通讯实现。

硬件代码按设备归属组织，协议函数与有状态 Decoder/Encoder 放在同一设备目录，
不再设置平行的 devices/protocols 两层目录或库：

| 目录 / 构建目标 | 职责 |
|---|---|
| `hardware/motors` / `dust_motors` | DJI、DM、CubeMars 协议；电机路由、反馈、方向和限幅、命令编码 |
| `hardware/remote` / `dust_remote` | DR16、VT03、WFLY 协议；SBUS 拼帧、遥控映射、丢帧处理 |
| `hardware/imu` / `dust_imu` | HI91 协议、拼帧、时间恢复和安装映射；板载 IMU 数据 |
| `hardware/referee` / `dust_referee` | 裁判协议、拼帧和状态快照 |
| `hardware/usb` / `dust_usb` | USB 会话、报告分发和统一发送调度 |

协议编解码函数保留独立文件，与设备类属于同一构建模块；不新增转接类。
`hardware/byte_stream.hpp` 提供多个串口设备共用的有界拼帧缓冲。
`MotorEncoder` 是无状态编码器，
接收完整的 MotorTargets，返回拥有 CAN 载荷的批次；不使用 Sink 或发送回调。
DR16、VT03、CubeMars 协议作为后续设备支持保留并参与编译；当前未接入设备链路，
不能仅因暂时没有调用者而删除。
`MotorFeedbackDecoder`、`SbusDecoder`、
`Hi91Decoder`、`RefereeDecoder` 和 `OnboardImu` 各自持有最新样本，解析操作由接收线程独占。
SBUS、HI91、裁判解析器分别拥有拼帧缓冲及恢复状态，不持有传输资源。
各设备内部通过 PublishedSnapshot 发布固定大小状态，Read() 加锁返回副本；
解析缓冲和时钟恢复状态不加快照锁，只由 USB 接收线程访问。
现有轮速度仍保留 rpm、IMU 姿态保留 deg，避免将单位迁移混入算法重构；关节使用
rad、rad/s、N·m。轮力矩使用机体坐标，关节力矩保留现有电机坐标。

`dust_usb` 拥有 libusb 事件线程和独立 USB 发送线程；接收与发送调度均与 1 kHz 控制循环解耦。
UsbLink 校验 USB 报告、换算主机时间，直接向各设备解析器分发 CAN/UART，
各设备自行发布快照。接收实现位于 `usb_receive.cpp`，
仍是同一个 UsbLink 的成员实现。
每周期目标集合对每个电机只保存一项操作，结构上避免重复命令；MotorEncoder 按
左 W/B/D、右 W/B/D 顺序生成帧，提交给 UsbTransmitter；发送线程调用私有的
UsbLink::Send(span) 编码 USB 请求并异步发送。
当前不新增板卡封装，也不拆分仓库。

接收路由为 CAN → MotorFeedbackDecoder、UART 1 → SbusDecoder、UART 2 →
Hi91Decoder、UART 3 → RefereeDecoder、板载 IMU → OnboardImu。
UART 4～6 继续只发布原始块队列。USB 块序号负责识别断流，各设备自行清理未完成帧。
新会话 Ready 时重置所有设备样本、拼帧缓存和原始队列；SBUS 连续丢帧超时仍由
USB 事件循环定期检查，静默通道不会阻止禁用。

`referee_decoder().Read()` 同时提供最近原始块和最近完整帧的
独立时间戳；沿用现有帧边界解析，尚未校验 CRC8/CRC16，不参与控制许可或功率限制。
`onboard_imu().Read()` 提供板载原始字节、温度和采集序号，
不覆盖控制用 HI91。控制线程在 ChassisRuntime 中直接调用所需设备的 Read()，
组合 ControllerInputs 后冻结本周期输入。没有跨设备的原子采样承诺，各设备时间戳
仍由控制输入层检查；底盘算法不依赖 Decoder 类型。
六路原始 UART 块由独立 UartReceiveQueue 保存。
`dust_realtime` 提供 Linux 单调时钟、绝对周期等待及调度配置，供不同通讯应用复用。
`src/main.cpp` 启动 ROS 2 节点，`src/dust_control_node.*` 处理参数、指令和状态发布。
`src/chassis/chassis_runtime.*` 拥有底盘控制线程，只编排自身的输入快照、指令仲裁、
算法调用、编码和提交；命令与队列位于 chassis/motion_command.hpp，仲裁位于
chassis/command_arbiter.hpp。底盘借用各 Decoder、发送器和遥测发布器，不创建或
关闭这些共享资源。算法库 dust_chassis 与有硬件依赖的 dust_chassis_runtime 分开构建。

`src/hardware/usb/usb_session.*` 拥有 USB 链路、原始 UART 队列和发送器，提供会话
启动与停止。底盘构造时向借用的发送器注册自己的完整 CAN 命令组，随后装配处
启动 USB 会话，再启动底盘线程。TelemetryPublisher 继续自行拥有遥测工作线程。

`src/robot.*` 是 ROS 与离线入口共用的装配处，只加载配置、连接对象和协调生命
周期，不拥有工作线程，不读取设备或计算控制目标。退出先停止业务线程，再停止
USB 发送并关闭链路，最后回收遥测。异常启动也按此顺序清理已创建资源。
各入口检查模块完成和故障后请求整体退出；发送器独立检查每个控制源期限，
防止入口调度延迟或其他业务流量掩盖底盘停顿。没有集中式 ControlRuntime 或 src/runtime。

发送路径为 controller.Tick() → motor_encoder.Encode(output.motor_targets)
→ UsbTransmitter::SubmitCan(kChassis) → USB 发送线程 → UsbLink。控制器内部
ChassisOutputPolicy 保留使能、门控和关节补偿。编码失败停止底盘线程并向入口
报告故障，不提交部分批次或复用上一周期命令。
`src/standalone_main.cpp` 为同一模块装配提供离线命令行入口。
ROS 回调与控制线程通过有界指令/观测队列连接，控制线程不调用 ROS。
ROS 接口和指令仲裁详见 [ROS 2 说明](ROS2.md)。

`dust_telemetry` 只读取观测快照，控制侧每 10 ms 尝试压入预分配 SPSC 队列；
队列满立即丢遥测，不等待。普通调度的消费线程负责 JSON 和 SDK，SDK 网络线程
继承避开控制/USB CPU 的亲和性。USB 库不依赖遥测库。

`dust_mpc` 是单独的 OSQP 控制算法库，保留离线 demo 和性能测试。
旧 MPC Link 通信协议及 `usb_mpc` 开发应用已移除。
将 MPC 作为新的 `ChassisController` 算法接入仍是独立开发工作。
EtherCAT 实现、专用协议/安全策略、应用和测试仅在 `dev/wbr_infantry_ethercat` 分支；
当前 USB 正式分支没有相应源码或构建入口。

公共 `wbr::`、`modules::` 等命名空间、线协议和配置字段暂保留，避免将大规模
符号改名和行为变更混入本次结构重构。CMake 对外提供 `dust::chassis`、`dust::usb`、
`dust::telemetry`、`dust::mpc` 等目标。

默认生产构建只产生当前底盘应用与依赖。MPC 开发模块、开发应用和测试
由构建选项显式开启，不由设备探测结果自动替换正式控制路径。

## 启动配置

配置由使用模块拥有：`hardware/motors/motor_config.*` 校验电机地址、槽位和期限，
`hardware/imu/imu_config.*` 与 `hardware/remote/remote_config.*` 分别校验 HI91 和
遥控期限。`hardware/config_yaml.hpp` 仅提供共用的 YAML 字段和数值校验函数，
不保存配置。YAML 解析仅在三个设备模块的启动加载函数中执行。

Robot 构造时协调加载两个 YAML；全部校验成功后保存本实例的配置，
再允许创建控制器、Encoder、UsbLink 和线程。配置通过构造参数复制给使用者：
MotorEncoder 与 MotorFeedbackDecoder 使用相同的电机映射，UsbLink 用同一配置
检查网关通道；HI91 将自身期限用于时间恢复。底盘只接收 InputDeadlines 数值，
不依赖 YAML 或设备配置加载实现。

不存在进程级全局配置、Freshness()/Addresses() 访问或单次加载标志。
每个设备实例持有只读配置副本；Reset 不修改配置，不提供运行中重载接口。
不同配置的实例可同时存在，后续加载失败不会影响已存在对象。
`src/chassis/defaults` 保存其余编译期参数；日常调试入口仅为 `config` 中两个 YAML。

## 数据与资源接口

设备快照使用 `src/concurrency/published_snapshot.hpp`，锁使用 PTHREAD_PRIO_INHERIT，
临界区只复制状态；Read() 允许短暂阻塞，不在锁内解析或分配内存。
原始 UART 队列的模型和接口位于 uart_receive_queue.hpp，会话清理和队列实现
位于 uart_receive_queue.cpp。调用方获得副本，不持有内部引用或锁。
USB 设备发现与独占打开、会话握手、接收/发送完成及资源回收按职责连续组织。
ROS 节点内部按启动配置、指令接收、状态/IMU/关节发布排列，保持现有 topic 和门控。
MotionCommandQueue 单生产者复制入队、单消费者 TakeLatest 取走当前最新完整记录。

## CAN 组归属与发送调度

采用完整 CAN 命令组独占方案：启动前通过 ConfigureCan 注册每个控制器拥有的
(bus, command_id)，相同总线相同 ID 只能有一个所有者，包括 DJI 帧内的全部槽位。
不同总线的相同 ID 可以独立使用。控制器先用自己的 Encoder 生成完整帧，再提交
SubmitCan；发送层不合并电机槽位、不理解电机协议。当前只注册实际底盘映射；
云台接入时须注册独立命令组并同步检查 MCU 安全配置，不应随意复用底盘组。

每来源最多 16 帧，提交时复制并覆盖未发送的旧批次，单个 USB 请求最多 6 帧，
两来源轮流调度。跨 USB 请求不保证 MCU 原子执行；新批次可替换旧批次尚未提交
的剩余帧，已进入 libusb 的请求无法撤回。递增序号和主机单调生成时间用于拒绝
重放、倒退、未来和过期目标。空批次表明活跃控制周期没有命令，仅提交一次，
不会由发送线程周期重放来续租。

每个注册 CAN 来源独立检查期限（最大 20 ms，底盘默认 20 ms），首次提交前
也计时；所有来源首次提交前不下发业务流量。任一来源超时即停止整个 USB 会话，
过期后不能靠新目标恢复，须重新启动会话。MCU 当前只有会话级租约，其他 CAN
来源和 UI 流量都不能掩盖底盘停顿。不新增独立续租心跳。

UI 使用线程安全 SubmitUart(channel, bytes, generated_ns, lifetime_ns)。
完整消息最多 512 字节，FIFO 最多 16 条，满时返回 false，由调用方决定重试或丢弃。
尚未开始的过期消息丢弃；开始后按顺序发完，切成最多 96 字节的 USB 请求，
CAN 可在块间优先发送。默认 UART 总预算 8000 字节/秒，并为 CAN 预留两个
USB OUT 槽位；实际预算应结合 UART 波特率和裁判系统限制配置。
提交成功只表示主机接收数据，不表示 MCU 确认或 UART 已发送。

UsbLink 的 Send、SendUart、Stop 为私有接口，仅 UsbTransmitter 使用；握手完成后
命令序号与业务 USB 提交由发送线程独占。退出先停止并等待发送线程，再 Close
取消并排空 USB 传输。提交临界区使用优先级继承锁，只执行校验和有界复制。
这版提供云台和 UI 的提交入口，尚未增加相应算法或空业务线程。现有硬件运行
增加一个 dust-usb-tx 线程；离线仿真不启动 USB 线程。
