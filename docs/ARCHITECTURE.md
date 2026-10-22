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

`dust_mpc` 是单独的 OSQP 控制算法库，`dust_ethercat` 是单独的下位机通讯库，
两者不是历史代码。现有 `ethercat_mpc` 和 `usb_mpc` 应用保留各自原控制/安全策略。
当前缺少将 MPC 作为新的 `ChassisController` 算法自动切换、或把 EtherCAT 替换
USB 网关的统一适配层；本次不虚构已完成集成，也不修改控制行为来追求统一接口。
后续在这些库的输入/输出边界上增量集成。

公共 `wbr::`、`modules::` 等命名空间、线协议和配置字段暂保留，避免将大规模
符号改名和行为变更混入本次结构重构。CMake 对外提供 `dust::chassis`、`dust::usb`、
`dust::telemetry`、`dust::mpc`、`dust::ethercat` 等目标。

默认生产构建只产生当前底盘应用与依赖。MPC、EtherCAT 开发模块、开发应用和测试
由构建选项显式开启，不由设备探测结果自动替换正式控制路径。
