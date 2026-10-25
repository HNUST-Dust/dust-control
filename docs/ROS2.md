# ROS2 节点使用与接口

已在 Ubuntu 22.04 / ROS 2 Humble 验证。对外为单个 `dust_control` 节点；
控制算法、USB v4 协议和保护帧不变。`src/main.cpp` 是唯一应用入口，
通过 `src/robot.*` 装配各模块，模拟模式也使用此入口。

## 构建和离线启动

```bash
source /opt/ros/humble/setup.bash
colcon --log-base build/colcon-log build --base-paths . \
  --build-base build/colcon --install-base build/ros-install \
  --cmake-args -DDUST_BUILD_ROS2=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source build/ros-install/setup.bash
# 先在配置目录的 robot.yaml 中设置 simulate: true、realtime: false
ros2 launch dust_control dust_control.launch.py
```

该命令不打开 USB。Ctrl+C 停止并回收控制线程。真实运行时默认禁止输出；
YAML `execution.enable_output: true` 仍须满足物理遥控失能姿态确认、遥控使能、
传感器新鲜度及原状态机门控。无运行期使能服务，不自动重新 Arm。

已 source ROS 时 CMake 默认构建节点；纯 C++ 构建可显式设置
`-DDUST_BUILD_ROS2=OFF -DBUILD_TESTING=ON`，只构建库和模块测试，无生产应用入口。ROS 模式需系统 Python 可导入 ament/rosidl。

## 执行方式

节点使用普通调度的 ROS executor，发布回调每 1 ms 有界读取最新底盘观测，
辅助发布每 10 ms 执行，模块状态每 5 ms 检查。控制线程独立运行，不调用 ROS。
线程所有权、设备快照与退出顺序统一见 [架构说明](ARCHITECTURE.md)。

## ROS 接口

默认节点名 `dust_control`，私有 topic 支持 namespace 和 remapping。

| Topic | 类型 | 含义 |
|---|---|---|
| `~/imu` | `sensor_msgs/msg/Imu` | HI91 原始测量，hi91_link 坐标系；不发布未验证的姿态四元数 |
| `~/joint_states` | `sensor_msgs/msg/JointState` | 四个关节电机的解码位置、速度和反馈力矩；反馈有效时发布 |
| `~/chassis/telemetry` | `dust_control/msg/ChassisTelemetry` | 姿态、双腿运动学、有效性、样本时间和周期统计；无效数值为 NaN |
| `~/status` | `diagnostic_msgs/msg/DiagnosticArray` | 状态机、使能请求、新鲜度、错过周期 |

暂不接入外部规划节点，没有运动目标订阅或通往控制线程的指令队列。
底盘状态每个控制周期采样（目标 1 kHz），高频发布回调周期 1 ms，只发布最近观测。
`~/chassis/telemetry` 使用 KeepLast(1) / BestEffort / Volatile；订阅者也应采用
相容 QoS 并缓存最新状态。回调延迟时跳过旧样本，不补发历史，不承诺每个控制周期都送达。
IMU 与 joint_states 最多 100 Hz，使用 SensorDataQoS；status 约 50 Hz，reliable / depth 1。
非实时调度和虚拟机的唤醒延迟可能使实际频率低于目标，需以目标主机实测为准。
IMU stamp 由当前 ROS 时间扣除主机单调时钟测得的样本年龄估算，非独立时钟同步。
没有发布虚构的 odom 或 TF；轮式里程计及 HI91 到 base_link 的外参需后续明确。
`joint_states` 当前不包含尚未定义连续位置的两个轮电机。

## 启动配置

ROS 只声明只读的 config_directory 参数，默认 launch 选择安装目录中的 config。
可用下面的命令选择外部配置，无需重编译：

```bash
ros2 launch dust_control dust_control.launch.py config_directory:=/path/to/config
```

simulate、realtime、enable_output 统一由 robot.yaml 的 execution 段加载。
旧 ROS 参数与命令行运行开关不再支持，不保留覆盖 YAML 的第二配置入口。
不提供 run_seconds，使用 Ctrl+C 正常停止；模块禁用时也可正常退出。
线程、Adapter、序列号及 CPU 配置见 [装配文档](ROBOT_CONFIGURATION.md)。

ROS2 话题和 Foxglove 性能验收留到实机；OrbStack 用于构建与功能调试。

仓库内启动使用 `./scripts/run.sh`，默认运行 build/production/dust_control_node，
配置目录为仓库 config。选择其他目录时传原生 ROS 参数：

```bash
./scripts/run.sh --ros-args -p config_directory:=/path/to/config
```
