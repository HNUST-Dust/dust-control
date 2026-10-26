# 统一 Foxglove 接入

每套 ROS 2 系统启动一个独立的 `foxglove_bridge`，所有业务节点通过 ROS 2
话题提供数据。dust-control 不再内嵌 Foxglove SDK，不监听 WebSocket 端口。
底盘、云台、UI 线程不创建各自的 ROS 节点或 Foxglove 服务。

安装系统对应版本的 Bridge，并构建、source 本工程的 ROS 安装目录：

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
# 先在 dust_control.yaml 中设置 execution.simulate: true、execution.realtime: false
ros2 launch dust_control dust_control.launch.py
# 另一个已 source ROS 和工程安装目录的终端；全系统只启动一次：
ros2 launch dust_control foxglove.launch.py
```

Foxglove 选择 Foxglove WebSocket，连接 `ws://127.0.0.1:8765`，导入
`debug-layout.json`。实际硬件运行时去掉 simulate/realtime 覆盖项。
已有系统级 Bridge 时直接复用，不再启动本工程的 Bridge launch。
Bridge 默认仅监听本机；远程查看可传 `address:=0.0.0.0`，端口可用 `port:=8765` 指定。
Bridge 只启用 connectionGraph 能力，保留话题订阅，不提供客户端发布、参数修改或服务调用。
业务节点与 Bridge 的 ROS_DOMAIN_ID、DDS 发现和 QoS 必须兼容。

底盘曲线话题默认 `/dust_control/chassis/telemetry`，消息定义在
`msg/ChassisTelemetry.msg`。命名空间或节点名改变后，布局中的话题路径也需同步。
观测由控制线程每周期生成（目标 1 kHz），ROS 回调每 1 ms 发布当前最近观测，
不补发历史样本；控制线程不做 ROS 序列化或网络发送。话题使用 BestEffort / Volatile /
KeepLast(1)，Bridge 对该话题也使用 BestEffort，并将订阅深度限制为 1。
同一高频状态话题同时供算法节点和 Foxglove 使用；诊断约 50 Hz，标准 IMU/关节话题最多 100 Hz。
云台和 UI 当前为空线程，尚无观测话题。
其他 ROS 2 节点直接发布自己的话题，由同一个 Bridge 接入，不经过 dust-control。

原生遥测字段全部保留；JSON null 改为 ROS 浮点 NaN。左右腿分别判定新鲜度，
不能只用整体 feedback_valid 判断单腿是否有效。height_m = -hz（轮轴低于髋部时为正）。
header.stamp 由 ROS 当前时间减去样本单调时钟年龄估算；monotonic_ns 供同主机内部
计算，不能直接当 ROS 时间。control_duration_us 包含控制计算、编码和发送队列提交。
queue_drops 统计底盘观测队列满时的累计丢样，随下一次成功入队的样本上报；
sequence 跳变可能来自发布回调主动选择最新样本、队列丢样或下游传输丢包，missed_releases 为错过控制周期。
源停更超过 200 ms 后 source_connected=false、测量字段为 NaN；进程退出后无新消息，
查看时也应关注消息接收时间。Bridge 停止、重启或端口占用不影响控制程序的生命周期。

原 `--foxglove`、`--foxglove-port`、节点 `foxglove_port` 参数及 SDK 构建选项已删除。
原 UDP 调试输出和 `--telemetry-port` 已删除；观测与队列归底盘模块所有。
