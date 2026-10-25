# 构建与测试

## 运行方式

```bash
# 生产应用：需要已安装并 source ROS2 Humble
source /opt/ros/humble/setup.bash
./scripts/build.sh

# 开发构建与回归，启用 MPC 和开发工具
./scripts/test.sh

# 无 ROS 的模块回归，不生成生产应用
./scripts/test.sh headless
```

应用集成测试统一运行 dust_control_node。无 ROS 构建保留控制、设备、线程生命周期
和配置校验测试；纯 C++ 测试程序不是另一个生产入口。
Python3 / PyYAML 用于回归，生产代码不生成 YAML 参数头。

MPC 首次构建需要获取固定版本 OSQP；可用 FETCHCONTENT_SOURCE_DIR_OSQP 指定本地源码。
可选 Foxglove 测试需要系统安装 foxglove_bridge，Python 依赖见
[tests/requirements.txt](../tests/requirements.txt)。不下载原生 Foxglove SDK。

## 覆盖范围

| 测试 | 主要检查 |
|---|---|
| `chassis.contracts` | 控制行为、状态机和运行时参数的实际作用 |
| `chassis.original_balance_parity` | 可选原 MCU 同输入数值对照，需 DUST_REFERENCE_CHASSIS_DIR |
| `chassis.observation_queue` | 有界队列饱和、回绕与并发读写 |
| `chassis.runtime_lifecycle` | 底盘独立运行、停止与借用资源边界 |
| `runtime.business_lifecycle` | 云台/UI 线程、绑定与资源回收，无虚假 CAN 来源 |
| `configuration.startup_loading` | 参数实际生效、非法配置提前失败、实例配置互不污染 |
| `configuration.robot_topology` | 两/三板绑定、跨板电机路由与 ID 隔离、延迟握手、故障回收、空板不 Arm |
| `chassis.disabled_smoke` | ROS 节点按 YAML 模拟运行，禁止模拟开启真实输出 |
| `usb.lifecycle`、`usb.reconnect` | USB mock 身份筛选、握手、拒绝、断连与重连 |
| `usb.*`、`devices.*` | wire 边界、UART、CAN 调度、设备编解码和快照并发 |
| `architecture.dependencies` | 控制、设备、传输与装配之间的引用和链接边界 |
| `ros.telemetry_contracts` | 字段、时间戳、NaN、左右腿有效性与源停更 |
| `ros.node_integration` | 真实 ROS 图、观测发布、参数约束、退出和 USB 故障传播 |
| `ros.foxglove_bridge` | 可选真实 Bridge、多节点共享、CDR 解码与 Bridge 独立重启 |
| `mpc.*` | 按构建选项启用的 MPC 数学和 QP 回归 |

头文件和实现保持可独立编译。C++ 格式检查见 [编码风格](CPP_STYLE.md)。
chassis_balance_fixture 始终在测试构建中可用，可与迁移前主机同输入输出比较。
Release 测试中的必要断言仍然启用，自动测试不打开真实电机输出。

配置回归包括电机路由、方向、电流换算与限幅、传感器超时、机构几何、控制增益、
气弹簧、关节限幅和倾角保护；检查缺失、未知、重复、非有限值、非法范围与版本。
多会话测试覆盖两/三块同时活动的 mock Adapter，验证跨板电机子集、相同 CAN ID 隔离、
延迟握手、后续板 Arm 失败、运行断连及正常停止；不代表已验证实体多板的时序。
纯 C++ 分支直接检查装配与启动前校验，ROS 分支额外检查节点启动和信号退出。

## 环境与结果

OrbStack 用于日常开发和功能调试。USB mock 和模拟节点仍保留原 20 ms 保护阈值，
虚拟机调度停顿可能触发租约、控制看门狗或传感器流超时；失败必须记录，不能通过
放宽保护、静默重试或删除断言把结果改成通过。

Bridge 集成测试将 Bridge/DDS 负载移出 CPU 3，用于功能验证，不作为实时性能验收。
1 kHz 状态、Foxglove、实际多板 USB 和完整控制时序留到目标机器人主机测量。
性能工具与历史数据见 [性能记录](benchmarks/README.md)，不作为带固定频率阈值的 CTest。

逐次结果及失败记录见 [验证历史](history/VALIDATION.md)。旧记录中的构建目录、
命令、参数和测试数量仅对应当时版本，不能作为当前接口说明。
EtherCAT 构建和回归在开发分支执行，见 [分支约定](BRANCHES.md)。
