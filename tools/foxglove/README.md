# Foxglove 原生 C++ 遥测

使用控制器内置的非实时遥测线程和官方 C++ SDK，不需要 Python 转发服务。

本目录保留：

- `schema.json`：`/wbr/chassis` 消息定义，CMake 嵌入 C++，构建所需。
- `debug-layout.json`：Foxglove 原生 Plot 布局，可导入客户端。
- `requirements-test.txt`：仅端到端测试使用的 Python WebSocket 客户端依赖。
- `README.md`：使用说明。

## 构建与运行

Orb Ubuntu 仓库：`~/dust-control`。

```bash
cd ~/dust-control
./scripts/build.sh
# 有真实 USB 输入，默认禁止电机输出：
sudo ./scripts/run.sh --foxglove
```

默认监听 `127.0.0.1:8765`；`--foxglove-port N` 指定端口。
不传 Foxglove 选项时不创建服务。启动前用 Ctrl-C 停止先前可能仍在运行的
Python 演示进程，避免端口冲突；旧转发脚本现已删除。

仅现场操作人员在准备完成后使用原有 `--enable-output` 允许遥控门控输出：

```bash
sudo ./scripts/run.sh --foxglove --enable-output
```

遥控启动失能确认、输入超时、故障停止和 MCU command lease 保持原策略。
Foxglove 服务只读，不能使能机器人。

离线模式运行真实控制核心，但没有传感器，测量字段为 null：

```bash
./scripts/run.sh --simulate --no-rt --foxglove --seconds 10
```

首次构建下载并校验固定版本 Foxglove SDK 0.27.0 的 Linux x86_64 包。
也可配置 `-DDUST_FOXGLOVE_SDK_DIR=/path/to/foxglove` 使用已解压 SDK；
`-DDUST_ENABLE_FOXGLOVE=OFF` 可完全关闭原生 SDK 依赖。

## Mac 连接

Mac 已有 8766 → Orb 8765 隧道时可直接使用；重建命令：

```bash
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8766:127.0.0.1:8765 ubuntu@orb
```

在 Foxglove 中选择 **Open connection → Foxglove WebSocket**，
填写 `ws://127.0.0.1:8766`。导入本目录 `debug-layout.json`，Mac 路径为：

```text
$HOME/OrbStack/ubuntu/home/<linux-user>/dust-control/tools/foxglove/debug-layout.json
```

需要支持官方 SDK `foxglove.sdk.v1` 子协议的客户端。
客户端布局导入与视觉效果尚待确认。

## 字段与有效性

统一 topic：`/wbr/chassis`。

| 字段 | 含义 |
|---|---|
| `pitch_deg` / `yaw_deg` / `roll_deg` | HI91 姿态，度，原控制坐标约定 |
| `left_length_m` / `right_length_m` | 左右等效腿长，米 |
| `left_height_m` / `right_height_m` | `-leg.hz`，髋部到轮轴的有符号竖直距离，米 |
| `left_angle_rad` / `right_angle_rad` | 校准后腿角，弧度 |
| `left_length_rate_m_s` / `right_length_rate_m_s` | 腿长变化率，米/秒 |
| `imu_fresh` / `left_leg_fresh` / `right_leg_fresh` | 对应测量的新鲜度与有效性 |
| `feedback_valid` / `motor_stale_mask` | 控制输入整体有效性、六电机过期位图 |
| `source_connected` / `transport_age_ms` | 遥测源状态、队列传递年龄 |
| `state` | 原控制状态编号，见 `src/chassis/chassis_types.h` |
| `control_duration_us` | Tick、快照复制与 USB Send 耗时，微秒，仅采样周期 |
| `missed_releases` / `queue_drops` | 控制漏周期、生产队列满时丢样累计计数 |
| `sequence` / `monotonic_ns` / `simulated` | 样本序号、单调采样时间、是否模拟数据 |

竖直距离不是世界坐标离地高度：未补偿机身姿态、轮半径或接触状态。
腿部数据直接来自当前控制周期，复用原运动学、零点和方向。
Yaw 保留 HI91 原始范围，跨包角边界可能跳变，未作连续解包。
过期测量发 null，不将缓存值当作新鲜数据；左右腿独立判定。

## 线程与局限

1 kHz 控制线程每 10 ms 向预分配 SPSC 队列复制一个观测样本；
队列满立即丢遥测，无等待、动态分配、JSON 或网络 I/O。
消费线程使用 SCHED_OTHER，CPU affinity 排除控制/USB CPU，合并积压样本，
在其中初始化 SDK、序列化和发布。SDK 还会创建网络线程，它们继承该调度和亲和性。
慢客户端使用有界发送积压。`queue_drops` 不含消费端合并或客户端丢样，可通过
`sequence` 观察间断。

超过 200 ms 未更新时，仍在运行的消费线程发布失联状态并清空测量；
整个进程退出则 WebSocket 断开。时间有效性使用单调时钟，启动时的固定偏移
用于映射 Unix log time，不以系统校时影响 watchdog。
100 Hz 降采样不覆盖完整 1 kHz 瞬态，不能据此推断最坏控制执行时间。
原生接入尚无 MCAP CLI 录制选项；硬件 RT 时序尚待验证。

## 测试

常规构建执行控制回归，以及队列/本地 UDP 序列化契约测试。
UDP 通道保留用于这些测试，不需要转发服务。原生 WebSocket 端到端测试：

```bash
python3 -m venv .cache/foxglove-test-venv
.cache/foxglove-test-venv/bin/pip install -r tools/foxglove/requirements-test.txt
.cache/foxglove-test-venv/bin/python tests/chassis/foxglove_native_integration.py
```

覆盖消息定义/订阅、数值与单位、单侧过期、源停更、生产入口、SDK 线程调度与
CPU 隔离，以及端口冲突时在控制开始前失败。所有测试均不连接真实硬件。
