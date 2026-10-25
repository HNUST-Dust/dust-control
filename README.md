> 当前主机使用 USB v4，需配套同版本 dust-adapter 固件。
> v0.1 固件不兼容。当前分支保留原控制算法/遥控门控，尚未完成机器人上板验收。

> 协议与会话见 [USB v4](docs/protocols/USB.md)。

# dust-control — wbr_infantry USB

完整阅读入口见 [文档索引](docs/README.md)；构建与启动见 [ROS2 使用说明](docs/ROS2.md)。
Humble 环境下可用 colcon 构建；应用统一使用 ROS2 节点入口，默认关闭电机输出。


当前正式兵种分支：`wbr_infantry`；EtherCAT 开发分支：`dev/wbr_infantry_ethercat`。
历史版本：`v0.1`。分支及 CI 约定见 [分支说明](docs/BRANCHES.md)。

HNUST-Dust 轮腿机器人 Linux 控制工程。默认应用沿用已迁移的 1 kHz 底盘控制器，
通过 USB Bulk 与 MCU 网关通讯；Foxglove 通过独立的 ROS 2 Bridge 统一接入各节点话题。
MPC 控制算法作为在研模块保留；EtherCAT 实现、应用及构建入口仅在
`dev/wbr_infantry_ethercat` 分支继续开发。

## 依赖与默认构建

在已安装 ROS2 Humble 的 Orb Ubuntu 或机器人 Linux 主机中：

```bash
sudo apt-get install build-essential cmake ninja-build pkg-config libeigen3-dev libusb-1.0-0-dev libyaml-cpp-dev python3-yaml
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

CMake >=3.21，C++20。默认构建不引入 OSQP/MPC、开发应用或测试程序，
不下载或链接 Foxglove SDK。ROS 2 可视化需另装 `ros-$ROS_DISTRO-foxglove-bridge`。

```bash
# 离线调试：先将 robot.yaml 的 simulate 改为 true、realtime 改为 false
./scripts/run.sh  # Ctrl+C 停止
# 真实 USB：恢复 simulate: false、realtime: true，保持 enable_output: false
sudo ./scripts/run.sh
```

允许真实输出仍需现场操作人员在 robot.yaml 中显式设置 `execution.enable_output: true`，启动后先观察到遥控
失能姿态，再满足原控制器的使能、传感器新鲜度和恢复条件。Foxglove 不提供使能接口。
原控制参数、符号、零点、20 ms MCU lease、SBUS 丢帧策略和 HI91 恢复规则保持原样。

编码风格及格式检查入口见 [C++ 编码约定](docs/CPP_STYLE.md)。

## 开发构建与测试

```bash
./scripts/test.sh
```

`development` 预设启用 MPC、开发应用和自动回归，MPC 使用固定版本 OSQP v1.0.0。
开发构建不会自动切换默认底盘控制算法。当前分支没有 EtherCAT 构建选项；
开发该方案请切换到 `dev/wbr_infantry_ethercat`。

```bash
cmake --preset development
cmake --build --preset development --parallel 2
ctest --preset development
# 纯 C++、无 ROS 2 的开发配置：
./scripts/test.sh headless
```

ROS 2 消息和多节点 Bridge 验证见 [测试说明](docs/TESTING.md)。
生产运行入口默认使用 `build/production/dust_control_node`；
可通过 `DUST_BUILD_DIR` 选择其他构建目录。

## 工程结构

项目实现与内部头文件统一放在 `src/`，按职责相邻存放；当前不提供对外 C++ SDK，
因此不保留独立的根目录 `include/`。测试和开发工具保留各自目录。

| 目录 | 职责 |
|---|---|
| `src/main.cpp`、`src/dust_control_node.*` | ROS 2 唯一应用入口、配置目录和状态发布 |
| `src/robot.*`、`src/robot_config.*` | YAML 装配、具名 Adapter 所有权、模块启动退出协调 |
| `src/hardware/` | 按 motors、remote、imu、referee 归并协议与设备处理；usb 独立管理通信 |
| `src/chassis/` | 底盘控制线程、控制管线、状态机、运动学和 MPC 算法 |
| `src/gimbal/` | 独立云台线程及借用设备接口，目前为空实现 |
| `src/ui/` | 独立 UI 渲染线程，借用底盘板的裁判数据与 UART 提交端口 |
| `src/concurrency/` | 有界线程间快照与优先级继承锁 |
| `src/chassis/chassis_observation.hpp` | 底盘观测结构和有界 SPSC 队列 |
| `config/robot.yaml` | 模块与 Adapter 绑定、线程启用及 CPU/优先级 |
| `config/` | 各模块的电机映射、传感器期限与底盘参数 |
| `config/chassis_params.yaml` | 底盘启动配置，修改后重启生效 |
| `src/chassis/chassis_config.h` | 底盘共享物理常量、坐标约定和周期 |
| `tests/` | 各模块自动回归，不链接进生产应用 |
| `tools/foxglove/` | ROS 2 曲线布局与统一 Bridge 使用说明 |
| `tests/benchmarks/` | 独立性能研究工具，默认不编译 |
| `cmake/`、`scripts/` | 依赖和统一构建运行入口 |

阅读 [模块边界](docs/ARCHITECTURE.md)、[迁移记录](docs/history/MIGRATION.md)、
[MPC 开发](docs/MPC.md)、[Foxglove 使用](tools/foxglove/README.md)。
重构后已做的离线验证不代替硬件验证，重构版本尚未在机器人上使能运行。

运行时配置范围、加载方式和校验规则见 [配置说明](config/README.md)。

线程与多 Adapter 的配置和资源归属见 [装配说明](docs/ROBOT_CONFIGURATION.md)。
