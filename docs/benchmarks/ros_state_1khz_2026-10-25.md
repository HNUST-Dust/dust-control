# 底盘状态 1 kHz ROS 2 / Foxglove 通讯测试

代码目标已调整为每控制周期生成观测、1 ms ROS 发布回调，发布最近状态。
QoS 为 KeepLast(1) / BestEffort / Volatile；Bridge 订阅同一话题并使用相同可靠性和深度。
诊断约 50 Hz，标准 IMU/关节话题最多 100 Hz。

通讯链路可以承载约 1 kHz。但本机 OrbStack 非实时生产节点的端到端输出约 320 Hz，
不能据此宣称当前虚拟机中的整个控制与发布链路已稳定达到 1 kHz。

环境：Ubuntu 22.04、Linux 7.0.14-orbstack、ROS 2 Humble、rmw_fastrtps_cpp、
官方 foxglove_bridge 3.6.0。进程可用 CPU 0–8，普通调度，无实际 Adapter。
每种场景预热 2 秒，测量约 10 秒。状态 CDR 大小 204 B。
使用根 build，CMAKE_BUILD_TYPE 未指定；本次 CPU 数字不代表优化后的 Release 闭环负载。

## 隔离传输能力的 1 kHz 基准

使用独立 C++ 测试发布者忙等待维持频率，消息类型、字段布局、QoS 与生产一致。
忙等待只存在于 EXCLUDE_FROM_ALL 的测试程序中，消耗约一个 CPU，不能作为生产 CPU 开销。

| 场景 | ROS 接收 Hz | ROS 样本年龄 P99 | Bridge 客户端 Hz | Bridge 样本年龄 P99 |
|---|---:|---:|---:|---:|
| 仅快速 ROS 订阅者 | 999.70 | 0.113 ms | — | — |
| ROS + Bridge WebSocket | 997.35 | 0.588 ms | 999.63 | 1.243 ms |
| 慢 ROS 订阅者 + Bridge | 19.04 | 0.993 ms | 999.24 | 0.197 ms |

慢订阅者在每次回调后休眠 50 ms，接收频率约 19 Hz，收到状态的年龄 P99 仍小于 1 ms，
说明它跳过中间状态，没有积累几十或几百毫秒的旧消息。Bridge 继续接近 1 kHz。
第二场景 CDR 载荷约 203 KB/s，Bridge 客户端收到含 Foxglove 消息头的数据约 217 KB/s；
不包含以太网、IP、TCP、WebSocket 帧头和 DDS 发现等开销。
Bridge 进程树占单核约 8.74%，该场景 Python 测试接收端约 32.21%。
这不是 Foxglove 桌面应用渲染负载；不同场景尾延迟不同，单次测量不能视为确定性上界。

## 当前生产节点的非实时模拟对照

| 场景 | ROS 接收 Hz | ROS 样本年龄 P99 | Bridge 客户端 Hz | Bridge 样本年龄 P99 |
|---|---:|---:|---:|---:|
| 仅快速 ROS 订阅者 | 319.25 | 3.622 ms | — | — |
| ROS + Bridge WebSocket | 324.51 | 3.569 ms | 328.68 | 4.909 ms |
| 慢 ROS 订阅者 + Bridge | 18.99 | 5.758 ms | 322.82 | 3.971 ms |

第二场景生产节点占单核约 10.04%，Bridge 进程树约 13.12%。
源端错过控制周期的计数明显增长；不能将全部序号跳变解释成网络丢包。
单独探测 `sleep(1 ms)`，100 次的中位数约 2.993 ms；将 timerslack 从 50000 ns
设为 1 ns 后仍约 2.993 ms，表明本环境的周期唤醒精度是当前限制之一。
额外以 sudo、无限 memlock 和程序已有实时配置进行模拟对照：0.5 秒短运行完成但
错过 303 个控制周期；10 秒测量尚未完成第一场景就触发 `Control deadline exceeded bridge watchdog`。
因此本次不能给出实时完整节点达到 1 kHz 的通过结论。没有修改生产等待方式、
20 ms 看门狗或租约来追求测试数字。

这次验证了普通 ROS 订阅者与真实 Bridge WebSocket 路径，没有测试跨机网络、
真实闭环负载、USB 反馈或 Foxglove GUI 渲染；目标机器人主机仍需进行实时调度实测。

完整原始统计见 [JSON](ros_state_1khz_2026-10-25.json)，复现命令见
[基准说明](../../tests/benchmarks/README.md#current-ros-2-state--foxglove-benchmark)。
