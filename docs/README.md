# 文档索引

本目录的现行说明以当前源码为准。历史迁移过程与验证结果单独保存，旧命令和旧参数
不代表今天仍然可用。当前应用只有 ROS2 节点入口，配置在启动时从 YAML 加载。

## 建议阅读顺序

| 文档 | 解决的问题 |
|---|---|
| [架构](ARCHITECTURE.md) | 模块如何分工，线程和 Adapter 由谁拥有，数据如何流动 |
| [运行配置](ROBOT_CONFIGURATION.md) | execution、模块启用、多板绑定、CPU 和优先级如何配置 |
| [ROS2 使用](ROS2.md) | 如何构建、启动节点，订阅底盘观测，选择配置目录 |
| [配置字段](../config/README.md) | 电机、传感器和底盘 YAML 的具体范围及校验规则 |
| [测试](TESTING.md) | 生产与模块测试如何构建，哪些结论仍需实机验证 |

## 开发与协议

| 文档 | 内容 |
|---|---|
| [C++ 风格](CPP_STYLE.md) | 排版、命名、所有权与维护规则 |
| [USB 协议与会话](protocols/USB.md) | 板卡发现、USB v4 报告、握手和保护边界 |
| [UART 接口](protocols/UART.md) | 六路通道分配、原始队列与发送规则 |
| [MPC](MPC.md) | 在研算法、开发程序与验证入口 |
| [分支约定](BRANCHES.md) | USB 正式分支、EtherCAT 开发分支及 CI |
| [Foxglove 使用](../tools/foxglove/README.md) | 独立 Bridge、统一端口与布局 |

## 历史与证据

- [迁移与导入来源](history/MIGRATION.md)：迁移背景、原始文件位置和源码清单。
- [重构与验证记录](history/VALIDATION.md)：逐次修改的检查结果，包括失败与局限。
- [YAML 参数筛选](history/YAML_PARAMETER_REVIEW.md)：参数削减时的决策依据。
- [性能报告](benchmarks/README.md)：历史测量、环境和原始数据。
- [v0.1 发布记录](releases/v0.1.md)：发布时的源码与验证范围。

OrbStack 用于开发和功能调试。实际多板 USB、控制周期、ROS2 话题和 Foxglove
性能留到目标机器人主机验证，历史虚拟机测量不构成实机性能承诺。
