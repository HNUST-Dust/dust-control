> 当前主机使用 USB v4，需配套同版本 dust-adapter 固件。
> v0.1 固件不兼容。当前分支保留原控制算法/遥控门控，尚未完成机器人上板验收。

> 见 [通用 USB 迁移](docs/GENERIC_USB_ADAPTER.md)。

# dust-control — wbr_infantry USB

ROS 2 单节点入口与独立控制线程见 [ROS 2 使用说明](docs/ROS2.md)。
Humble 环境下可用 colcon 构建；默认关闭电机输出，保留原离线命令行入口。


当前正式兵种分支：`wbr_infantry`；EtherCAT 开发分支：`dev/wbr_infantry_ethercat`。
历史版本：`v0.1`。分支及 CI 约定见 [分支说明](docs/BRANCHES.md)。

HNUST-Dust 轮腿机器人 Linux 控制工程。默认应用沿用已迁移的 1 kHz 底盘控制器，
通过 USB Bulk 与 MCU 网关通讯；Foxglove 提供可选的原生只读遥测。
MPC 控制算法作为在研模块保留；EtherCAT 实现、应用及构建入口仅在
`dev/wbr_infantry_ethercat` 分支继续开发。

## 依赖与默认构建

在 Orb Ubuntu 或机器人 Linux 主机中：

```bash
sudo apt-get install build-essential cmake ninja-build pkg-config libeigen3-dev libusb-1.0-0-dev libyaml-cpp-dev python3-yaml
./scripts/build.sh
```

CMake >=3.21，C++20。首次构建下载固定版本并校验 Foxglove C++ SDK；
默认构建不引入 OSQP/MPC、开发应用或测试程序。
离线环境可通过 `DUST_FOXGLOVE_SDK_DIR` 指向已解压 SDK，
或 `-DDUST_ENABLE_FOXGLOVE=OFF` 关闭 SDK。其他架构需指定匹配的官方 SDK。

```bash
# 无硬件离线运行真实控制核心；没有传感器时测量字段为 null。
./scripts/run.sh --simulate --no-rt --seconds 1
# 真实 USB，默认禁止电机输出：
sudo ./scripts/run.sh --foxglove
```

允许真实输出仍需现场操作人员显式传 `--enable-output`，启动后先观察到遥控
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
# 不需要 Foxglove SDK 的开发配置：
./scripts/test.sh headless
```

原生 WebSocket 测试需要 Python `websockets==15.0.1`；仅测试客户端需要 Python 包，
运行服务不需要 Python 进程。详见 [测试说明](docs/TESTING.md)。
生产运行入口默认使用 `build/production/chassis_controller`；
可通过 `DUST_BUILD_DIR` 选择其他构建目录。

## 工程结构

项目实现与内部头文件统一放在 `src/`，按职责相邻存放；当前不提供对外 C++ SDK，
因此不保留独立的根目录 `include/`。测试和开发工具保留各自目录。

| 目录 | 职责 |
|---|---|
| `src/main.cpp`、`src/dust_control_node.*` | ROS 2 节点入口、参数、指令接收和状态发布 |
| `src/robot.*` | 配置加载、模块装配和启动退出协调，不创建工作线程 |
| `src/options.*`、`src/standalone_main.cpp` | 运行选项和保留的离线命令行入口 |
| `src/hardware/` | 按 motors、remote、imu、referee 归并协议与设备处理；usb 独立管理通信 |
| `src/chassis/` | 底盘控制线程、指令仲裁、控制管线、状态机、运动学和 MPC 算法 |
| `src/concurrency/` | 有界线程间快照与优先级继承锁 |
| `src/telemetry/` | 有界队列、非实时序列化、Foxglove 服务 |
| `config/` | 电机映射与传感器 freshness 的启动时配置，各硬件模块负责加载和校验 |
| `src/chassis/defaults/` | 内部编译期参数和 schema |
| `tests/` | 各模块自动回归，不链接进生产应用 |
| `tools/foxglove/` | 遥测消息定义、曲线布局与使用说明 |
| `tests/benchmarks/` | 独立性能研究工具，默认不编译 |
| `cmake/`、`scripts/` | 依赖、参数生成和统一构建运行入口 |

阅读 [模块边界](docs/ARCHITECTURE.md)、[迁移记录](docs/MIGRATION.md)、
[MPC 开发](docs/MPC.md)、[Foxglove 使用](tools/foxglove/README.md)。
重构后已做的离线验证不代替硬件验证，重构版本尚未在机器人上使能运行。

运行时配置范围、加载方式和校验规则见 [配置说明](config/README.md)。
