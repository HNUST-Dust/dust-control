# dust-control — wbr_infantry USB

当前正式兵种分支：`wbr_infantry`；EtherCAT 开发分支：`dev/wbr_infantry_ethercat`。
发布版本：`v0.1`。分支及 CI/CD 约定见 [分支说明](docs/BRANCHES.md)。

HNUST-Dust 轮腿机器人 Linux 控制工程。默认应用沿用已迁移的 1 kHz 底盘控制器，
通过 USB Bulk 与 MCU 网关通讯；Foxglove 提供可选的原生只读遥测。
MPC 控制算法作为在研模块保留；EtherCAT 实现、应用及构建入口仅在
`dev/wbr_infantry_ethercat` 分支继续开发。

## 依赖与默认构建

在 Orb Ubuntu 或机器人 Linux 主机中：

```bash
sudo apt-get install build-essential cmake ninja-build pkg-config libeigen3-dev libusb-1.0-0-dev python3-yaml
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
生产运行入口默认使用 `build/production/apps/chassis_controller`；
可通过 `DUST_BUILD_DIR` 选择其他构建目录。

## 工程结构

| 目录 | 职责 |
|---|---|
| `apps/` | 底盘、USB MPC 等可执行入口 |
| `modules/chassis/` | 原控制管线、状态机、运动学和执行器门控 |
| `modules/mpc/` | 正在开发的腿高/横滚 MPC 算法 |
| `modules/usb/` | MCU 网关、USB 生命周期、采集快照与解码 |
| `modules/mpc_link/` | 现有 MPC offload 链路协议，区别于网关协议 |
| `modules/runtime/` | 共享 Linux 时钟、调度和统计工具 |
| `modules/telemetry/` | 有界队列、非实时序列化、Foxglove 服务 |
| `modules/protocols/`、`include/` | 传感器/电机编解码与公共协议头文件 |
| `config/chassis/` | 原 YAML 参数和校验 schema |
| `tests/` | 各模块自动回归，不链接进生产应用 |
| `tools/foxglove/` | 遥测消息定义、曲线布局与使用说明 |
| `benchmarks/` | 独立性能研究工具，默认不编译 |
| `cmake/`、`scripts/` | 依赖、参数生成和统一构建运行入口 |

阅读 [模块边界](docs/ARCHITECTURE.md)、[迁移记录](docs/MIGRATION.md)、
[MPC 开发](docs/MPC.md)、[Foxglove 使用](tools/foxglove/README.md)。
重构后已做的离线验证不代替硬件验证，重构版本尚未在机器人上使能运行。
