# 模块边界与运行规则

`dust_chassis` 只依赖协议类型、Eigen 和生成参数；不依赖 USB、EtherCAT、Foxglove
或 Linux 线程。`FrameSink` 是电机帧输出边界，`ControllerInputs` 是冻结的输入，
`Tick` 的可选同步观测快照不进行 I/O。

`dust_usb` 拥有单一 libusb 事件线程和 SensorStore；USB 接收与 1 kHz 控制循环解耦。
`dust_realtime` 提供 Linux 单调时钟、绝对周期等待及调度配置，供不同通讯应用复用。
`apps/chassis` 编排启动、输入快照、控制调用、输出门控和退出停止。

`dust_telemetry` 只读取观测快照，控制侧每 10 ms 尝试压入预分配 SPSC 队列；
队列满立即丢遥测，不等待。普通调度的消费线程负责 JSON 和 SDK，SDK 网络线程
继承避开控制/USB CPU 的亲和性。USB 库不依赖遥测库。

`dust_mpc` 是单独的 OSQP 控制算法库，`usb_mpc` 开发应用保留原控制与安全策略。
将 MPC 作为新的 `ChassisController` 算法接入仍是独立开发工作。
EtherCAT 实现、专用协议/安全策略、应用和测试仅在 `dev/wbr_infantry_ethercat` 分支；
当前 USB 正式分支没有相应源码或构建入口。

公共 `wbr::`、`modules::` 等命名空间、线协议和配置字段暂保留，避免将大规模
符号改名和行为变更混入本次结构重构。CMake 对外提供 `dust::chassis`、`dust::usb`、
`dust::telemetry`、`dust::mpc` 等目标。

默认生产构建只产生当前底盘应用与依赖。MPC 开发模块、开发应用和测试
由构建选项显式开启，不由设备探测结果自动替换正式控制路径。
