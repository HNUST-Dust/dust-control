# ROS 2 节点第一版

已在 Ubuntu 22.04 / ROS 2 Humble 验证。对外为单个 `dust_control` 节点；
控制算法、USB v4 协议和保护帧不变。原命令行程序保留作离线回归入口，
两者复用 `src/robot.*` 的模块装配，不能同时占用同一块 MCU。

## 构建和离线启动

```bash
source /opt/ros/humble/setup.bash
colcon --log-base build/colcon-log build --base-paths . \
  --build-base build/colcon --install-base build/ros-install \
  --cmake-args -DDUST_BUILD_ROS2=ON -DDUST_ENABLE_FOXGLOVE=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source build/ros-install/setup.bash
ros2 launch dust_control dust_control.launch.py simulate:=true realtime:=false
```

该命令不打开 USB。Ctrl+C 停止并回收控制线程。真实运行时默认禁止输出；
启动参数 `enable_output=true` 仍须满足物理遥控失能姿态确认、遥控使能、
传感器新鲜度及原状态机门控。无运行期使能服务，不自动重新 Arm。

已 source ROS 时 CMake 默认构建节点；纯 C++ 构建可显式设置
`-DDUST_BUILD_ROS2=OFF`，无需 ROS 依赖。ROS 模式需系统 Python 可导入 ament/rosidl。

## 执行和所有权

- `src/dust_control_node.*`：ROS 参数、订阅、状态发布；普通调度 executor。
- `src/options.*`、`src/standalone_main.cpp`：启动选项和保留的命令行入口。
- `src/chassis/chassis_runtime.*`：独立 1 kHz 底盘线程；指令队列和仲裁归属底盘。
- `src/hardware/usb/usb_session.*`：USB 会话和收发资源生命周期。
- `src/robot.*`：配置加载、模块装配、启动和退出顺序，无工作线程或业务循环。
- `src/telemetry/`：遥测工作线程；主入口 `src/main.cpp` 只启动 ROS 节点。

节点每 5 ms 检查模块完成或故障，观测发布仍为每 20 ms。退出先回收底盘线程，
再停止 USB 发送、关闭链路和回收遥测。底盘不会主动关闭借用的共享资源。

ROS executor → 单生产者指令队列 → 控制线程 → Encoder → UsbTransmitter → USB OUT；
USB 事件线程 → 各 Decoder 快照 → 控制线程 → 观测队列 → ROS executor。
控制线程不调用 ROS 发布器、参数或服务。控制线程提交底盘最新 CAN 批次，后续云台和 UI 使用各自提交入口；
独立 USB 发送线程统一下发，任一注册 CAN 控制源超时即停止整个会话，没有独立续租心跳。控制时钟、指令过期、设备 freshness 和 MCU 租约均不依赖 ROS 时间。
观察队列和命令队列容量固定；满时丢弃新记录，不阻塞生产者。
这些边界不代表已完成实际 RT 延迟验收；设备快照和发送提交仍使用优先级继承互斥锁。

## ROS 接口

默认节点名 `dust_control`，私有 topic 支持 namespace 和 remapping。

| Topic | 类型 | 含义 |
|---|---|---|
| `~/cmd_vel` | `geometry_msgs/msg/Twist` | base_link 下前进 m/s、逆时针 yaw rad/s；其他轴必须为零 |
| `~/motion_command` | `dust_control/msg/MotionCommand` | 前进速度、yaw 角速度、腿长变化率 m/s；frame_id 为空或 base_link |
| `~/imu` | `sensor_msgs/msg/Imu` | HI91 原始测量，hi91_link 坐标系；不发布未验证的姿态四元数 |
| `~/joint_states` | `sensor_msgs/msg/JointState` | 四个关节电机的解码位置、速度和反馈力矩；反馈有效时发布 |
| `~/status` | `diagnostic_msgs/msg/DiagnosticArray` | 状态机、使能请求、新鲜度、错过周期、指令拒绝数 |

指令 QoS 为 reliable / volatile / depth 1，传感器为 SensorDataQoS。
观测来自 100 Hz 采样，ROS 发布周期 20 ms，消费最近完整快照。
IMU stamp 由当前 ROS 时间扣除主机单调时钟测得的样本年龄估算，非独立时钟同步。
没有发布虚构的 odom 或 TF；轮式里程计及 HI91 到 base_link 的外参需后续明确。
`joint_states` 当前不包含尚未定义连续位置的两个轮电机。

## 指令仲裁

`command_source` 默认为 `remote`，ROS 指令不会参与控制。
设置为 `ros` 后，在物理遥控使能、遥控运动轴处于死区内且无手动动作时，
可使用最新 ROS 指令。两种 ROS topic 共享一个指令流，按本机接收顺序采用最新记录。

- 物理遥控运动、跳跃、爬楼等操作优先；干预时丢弃缓存 ROS 指令。
- 手动释放后必须收到新 ROS 指令，不能自动恢复旧目标。
- 指令年龄按本机单调接收时刻计算，默认 200 ms；超时运动目标归零，平衡循环继续。
- ROS 不改写遥控 sequence、robot_enable 或失能姿态，不会掩盖遥控失联。
- SI 单位目标经限幅、死区反变换映射到现有控制器，保留原限速与平滑。
- 非有限数、非支持轴和 frame_id 被拒绝，不刷新指令超时。
- ROS 指令不提供跳跃、爬楼等离散动作触发；本版仍由物理遥控处理。

所有硬件/控制参数为启动时只读：`simulate`、`enable_output`、`realtime`、
`control_cpu`、`usb_cpu`、`priority`、`strict_hi91_crc`、`config_directory`、
`usb_serial`、`usb_product`、`usb_pid`（-1 不筛选）、`command_source`、
`command_timeout_ms`、`run_seconds`（0 持续运行）、`foxglove_port`。
`config_directory` 在 launch 中指向安装的 motors/sensors YAML；直接启动节点不指定
时使用内建基线。暂不支持运行时修改电机映射或增益。

## 离线验证

`runtime.command_arbitration` 验证物理遥控优先、指令超时、失能隔离、NaN/Inf、
队列容量及手动释放后的旧指令清理。`ros.node_integration` 使用真实 ROS 图验证
状态接收、自定义消息、非法指令拒绝、只读参数、SIGINT 和非法启动参数。
原有 USB mock、协议、新鲜度、控制和遥测测试仍使用相同控制循环。
