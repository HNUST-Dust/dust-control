# 架构与数据流

`main → DustControlNode → DustControl`。节点发布观测，DustControl 加载配置、
拥有设备会话与业务模块，并协调启停。

```text
DustControl
├─ ChassisRuntime → ChassisController::Tick
├─ GimbalRuntime / UiRuntime（生命周期占位，无设备端口或业务输出）
├─ EthercatSession（每板一份）
│  ├─ MotorFeedbackDecoder / MotorCommandEncoder
│  ├─ WflySbusDecoder / Hi91Imu / OnboardImu
│  └─ EthercatTransmitter（完整六电机批次与20 ms期限）
└─ EthercatMaster（唯一通信线程）→ IghTransport
```

通用线程、时钟、等待、快照和统计位于 `src/concurrency/`，不依赖业务模块。
控制算法只引用设备被动数据头，不引用编解码服务、ROS、YAML 或主站。
底盘、云台、UI 各自拥有线程；业务只借用设备接口，不拥有或停止共享主站。

底盘周期按等待、采集输入、控制计算、编码提交、发布观测顺序执行。
输出策略生成周期命令和本周期空槽目标；通信编码器据此补齐 DTG1 六电机批次，
不读取底盘状态、不重放上一周期的使能或清错命令。
所有板先编码成功，再逐板提交；编码失败不提交本周期的部分目标。

`config/pdo_schema.yaml` 由 CMake 自动生成 `build/<preset>/generated/pdo_schema.hpp`。
布局、固定路由与启动 SDO 值只在 schema 维护；源目录不保存生成头或第二种 PDO 实现。
应用 schema 4 从 `dust_control.yaml` 加载从站身份、主站和模块配置。

接收路径为 OP/WKC 检查、DTG1 页解码、设备校验与物理量转换、快照发布。
HI91 网关序号只用于会话去重/回绕/回退检查，不作为传感器毫秒时间。
紧凑页直接转为测量值，不生成串口帧或 CRC；板载 IMU 直接读取三轴原始值，
保留原量程和坐标映射，不替代控制用 HI91。

启动先完成配置与装配，再启动底盘安全计算和主站握手，全部就绪后才发布启动许可。
退出先取消许可、回收业务线程，再由主站统一发 Stop 并关闭资源。
丢线、从站复位、损坏页或命令超时终止会话，不自动重新使能。

底盘总控的阅读入口为 `ChassisController::Tick`：输入整理、支撑力估计与离地检测、
状态转换及进入处理、按状态分派动作、统一输出策略。支撑力估计只传给离地检测。
`UpdateControlState` 负责进入状态时的复位和参考同步，`ComputeControlOutput`
显式列出每个状态对应的动作，直接接收各动作的统一控制结果。
动作完成或失败仅记录反馈，下一周期才由状态机转换；完成周期仍使用原状态的输出策略。
凳式控制器无调用方，已移出构建；仍被平衡控制使用的腿长参数保持原样。

回正、爬楼、腾空和跳跃读取同一份 `ChassisCycleInput`，动作结果通过
`control` 返回 `ChassisControlOutput`，各自保留阶段及完成/失败信息。
回正的 `RecoveryGimbalStatus` 是独立可选参数，云台失能请求保留在动作结果中。
左右腿和B/D关节均使用 `SidePair` / `JointPair`，总控不再维护数组顺序转换；
每个动作仍独立拥有参考、PID和阶段状态，计算顺序及复位规则保持原样。
