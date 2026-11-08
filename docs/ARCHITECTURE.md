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

跳跃不直接依赖平衡控制器。预压缩期间，`JumpControllerOutput` 的
`precompress_leg_length` 表示本周期需要的平衡控制目标，总控设置腿长、
屏蔽遥控腿长增量后调用已有平衡实例；其余阶段直接采用 `control`。
请求与返回阶段分别表达本周期输出和周期末阶段，进入推力的边界周期仍执行预压缩。
平衡参考、滤波及积分状态继续由平衡控制器持有，进入和退出跳跃时的同步规则不变。

`BalanceController::Update` 按固定顺序串联五个私有步骤：姿态参考渐变与横滚补偿、
机体运动估计、遥控运动参考与 yaw 滤波、LQR 力矩计算和分配、左右腿 VMC 输出。
`MotionTracking` 只传递本周期的运动误差和腿长参考速度，不引入跨周期状态。
参考、滤波和每侧腿长积分仍由同一个平衡实例持有；复位和参考同步规则保持原样。
外部支撑力继续覆盖横滚补偿并停用本地腿长 PID，单腿输出负责完成轴向力转换及关节限幅。

平衡控制的状态按职责分组，三个入口有不同的参考保留规则：

| 状态 | `Reset()` | `ResetForEnable()` | `SynchronizeReferences(input)` |
| --- | --- | --- | --- |
| 腿长参考 | 保留 | 配置腿长 | 当前双腿平均长度 |
| 腿角、pitch、roll 参考 | 保留 | 配置腿角偏置、零 pitch/roll | 当前 common_theta、pitch、roll |
| 运动估计与平移参考 | 清空 | 清空 | 清空 |
| yaw 目标与反馈滤波 | 清空并取消初始化 | 同左 | 同左 |
| 左右腿长积分 | 清空 | 清空 | 清空 |

构造时只设置配置腿长，其余参考和历史状态为零；使能初始化由总控显式调用。
使能边沿调用 `ResetForEnable()`；进入跳跃、从回正/爬楼/腾空/跳跃返回平衡时，
调用 `SynchronizeReferences()` 接续当前姿态。停止、使能握手、等待反馈或进入回正时
调用 `Reset()`；输出策略还会在停止、握手、等待反馈期间每周期触发它。
三个入口均可重复调用。yaw 目标角与反馈滤波在下一次 `Update()` 中分别使用
该周期的 yaw 和 yaw-rate 初始化，交接时不会提前锁定传感器采样。
