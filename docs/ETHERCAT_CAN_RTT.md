# NUC—EtherCAT—CAN DM 往返耗时台架

本工具使用真实 IgH，不启动机器人控制器。仅向 CAN3 的 DM ID=0x003 发送
`FF FF FF FF FF FF FF FD` 失能帧，接收 ID=0x013，不发送使能、清错、保存零点或运动指令。
主机配置为 `config/ethercat_dm_rtt.yaml`，仅保留 chassis 1 个发送槽和 CAN3 4 个接收槽，UART/IMU 关闭。
板上必须烧录 `config/can_bench_dm_rtt.yaml` 专用 MCU 配置，不能使用原 RX-only 镜像。
MCU 地址白名单只允许 CAN3 ID3，启动和保护帧同样是失能帧；主机工具固定具体发送载荷。

## 测量口径

- t0：NUC 已写好本次命令 PDO，即将调用 `ecrt_master_send()` 的单调时钟纳秒值。
- t1：某轮 `ecrt_master_receive()` 返回的单调时钟值，该轮包含对应 CAN 反馈。
- RTT=t1−t0，包括主站发送、EtherCAT、从站转发、CAN 仲裁与串行传输、电机处理、反馈返回和主站接收轮询。
  不等于纯 EtherCAT RTT，也不等于纯 CAN 线缆往返时间；不包含 t0 前的业务计算。
- 两端计时在同一 NUC、同一 steady_clock，无需用 MCU 与主机时钟相减。
- 先保持 Discover，约1秒检查空闲反馈（前100ms允许启动残留），随后 Arm。
  只有同一 session、同一 boot、完整 WKC、正确布局和本地保护能力的 Active 应答被处理。
- 只允许一个在途命令；命令间隔至少10ms，8ms内无应答立即停止，不重发、不把迟到应答配给下一条。
  重复的 PDO 保持相同 CAN batch_sequence，由固件去重，不重复发送 CAN。
- 只接纳新 CAN 批次中 ID0x13、8字节、控制器ID低4位为3、状态高4位为0（失能）的反馈，
  并核对从站接纳的命令批次序号。未发命令时收到目标反馈、额外重复反馈、OP/WKC丢失或固件保护均失败。
- DM 普通反馈不回显主机命令序号。以上为在空闲检查和单在途条件下的顺序配对推断，
  不能提供与带序号回显节点相同的严格关联保证，也不适合同时在途多条命令的1kHz吞吐测量。
- 主循环不打印每条样本，记录在内存，结束后写CSV；绑定当前允许的最高编号CPU，尝试FIFO40和内存锁定，
  明确打印实际是否成功。不自动更改系统权限、CPU隔离或IRQ亲和性。

## 运行

使用与 Discover 工具相同的 CMake 配置，启用 `DUST_BUILD_TOOLS` 和 `DUST_BUILD_IGH`：

```sh
cmake --build build/bench --target dust_ethercat_dm_rtt
# 先做10条探测；每轮退出后从站锁定配置，下一轮前需重启控制板。
./build/bench/src/hardware/ethercat/dust_ethercat_dm_rtt config/ethercat_dm_rtt.yaml probe.csv 10
# 重启后，1000条正式样本
./build/bench/src/hardware/ethercat/dust_ethercat_dm_rtt config/ethercat_dm_rtt.yaml rtt.csv 1000
```

固件构建与刷写：

```sh
west build -b dust-hpm5e31 -d build/dm_rtt -- \
  -DDUST_ADAPTER_CAN_CONFIG="$PWD/config/can_bench_dm_rtt.yaml"
west flash -d build/dm_rtt
```

CSV包括 sample、tx_ns、rx_ns、rtt_us、wake_lateness_us。分位数按nearest-rank计算，
标准差使用总体标准差；peak_to_peak=max−min，max_adjacent_change=max(|RTT[i]−RTT[i−1]|)。
部分样本后失败时仍保存已有样本，但不算通过。超时和协议错误保留为失败，不从统计中静默删除。

## 2026-10-27 实机结果

NUC内核5.15.0-1114-realtime；IgH CLI1.7.0；专用网口54:b2:03:9e:f2:fa。
CPU7，实际SCHED_OTHER（当前账户无实时调度权限），mlock成功。HPM5E31固件源517d774，
FreeRun，PDO名义周期1000µs，Rx96B/Tx400B，布局签名2699856283。
板上现在运行此DM RTT专用镜像，不能当作全电机正常控制镜像使用。

10条失能帧探测通过，确认该台架能返回失能状态。正式1000条测量：

| 指标 | 结果 |
|---|---:|
| 样本数 | 1000 |
| 最小RTT | 2791.134 µs |
| 平均RTT | 2973.878 µs |
| P50 | 2884.060 µs |
| P95 | 3135.207 µs |
| P99 | 3152.087 µs |
| 最大RTT | 3174.751 µs |
| 总体标准差 | 124.910 µs |
| 峰峰值 | 383.617 µs |
| 最大相邻样本变化 | 326.914 µs |
| 超时 / OP或WKC丢失 | 0 / 0 |

主机发送周期的唤醒迟到平均106.356µs，最大222.664µs；实际平均命令间隔10627.657µs。
本轮为低频单在途延迟基线，不是1kHz命令吞吐、实时调度上限或长期压力验收。
后续可在获得实时调度权限后保持相同口径进行对比；若要1kHz多命令在途，需要可回显序号的应答端。

- [原始CSV](bench-results/2026-10-27-dm-rtt-1000.csv)
- [工具输出](bench-results/2026-10-27-dm-rtt-1000.txt)
- [时序与分布图](bench-results/2026-10-27-dm-rtt-1000.png)

首轮因过期的启动失能反馈进入PDO而被保护拦住，尚未产生测量样本。
固件新增入队数据新鲜度过滤及回归：过期CAN反馈丢弃并计入drops，禁止冒充新应答；
同时统一UART重启路径idle为200µs，避免921600下计数溢出。完整固件回归和新用例通过，
专用固件JTAG写入及镜像校验通过。默认生产配置与RX-only配置均保留。

## 加速交换周期初轮对比（2026-10-27，快照修复前）

固件9f2e8fc支持125/250/500/1000µs，Active主循环改为10µs短忙等。
主机周期通过YAML配置，控制频率保持1kHz。专用配置：
`config/ethercat_dm_rtt_500us.yaml`、`_250us.yaml`、`_125us.yaml`；
后两者只是配置入口，不能据此宣称相应频率已经稳定。

台架工具显式设置IgH send interval，并根据权限选择等待方式：
FIFO40时提前40µs唤醒再忙等，避免全周期忙等触发RT带宽节流；
普通调度时500µs以上提前150µs唤醒，250µs以下整周期忙等。
这些是台架工具的等待策略；生产主站继续使用其配置的实时线程与绝对定时。

| 条件 | 有效样本 | 平均RTT | P99 | 最大RTT | 标准差 | 结果 |
|---|---:|---:|---:|---:|---:|---|
| 旧固件，1000µs，普通sleep | 1000 | 2973.878µs | 3152.087µs | 3174.751µs | 124.910µs | 通过 |
| 新固件，500µs，提前150µs唤醒 | 1000 | 1497.792µs | 1498.645µs | 1512.690µs | 0.670µs | 通过 |
| 新固件，250µs，普通调度 | 0或1 | — | — | — | — | WKC=0，失败 |
| 新固件，250µs，用户sudo运行FIFO40 | 0 | — | — | — | — | AL=OP/WKC=0，失败 |

500µs通过记录：[输出](bench-results/2026-10-27-dm-rtt-500us.txt)、
[CSV](bench-results/2026-10-27-dm-rtt-500us.csv)。峰峰值24.045µs，最大相邻变化14.945µs，
1000条均为失能状态反馈，无超时、无OP/WKC丢失。
普通sleep的500µs初轮在107条后失败，仍保留失败事实；改进等待策略后通过。
旧/新固件和等待策略均有差异，不能把全部改善归因于某一个参数。

250µs不是“已可用于控制”的配置：实时调度也失败，不应简单归咎于CFS。
诊断只在首次WKC不完整后继续轮询最多1ms，记录迟到帧再按原失败返回，不隐藏周期违约。
[诊断记录](bench-results/2026-10-27-dm-rtt-250us-deadline.txt)显示首次poll在发送后247.693µs，
到433.730µs才观察到完整WKC；此为主机观察到的返回时刻，不是网线硬件时间戳。
PHY日志确认100Mbps全双工。I225-LM由ec_igc驱动，PCIe5GT/s×1，运行电源on/active，
但l1_aspm、l1_1_aspm、l1_1_pcipm均为1。

`tools/ethercat_rtt_aspm_probe.sh`为单次root对比入口：只操作专用0000:58:00.0的ec_igc网口，
保存三项ASPM原值，临时关闭后运行250µs/FIFO40测试，正常退出或失败均恢复原值。
不是永久调优脚本，也不改无线网络或电机参数。需要先重启控制板，以解除上一轮配置冻结。
ASPM目前仅为待验证假设；尚不能用它解释全部延迟，也不能宣称已达到亚毫秒稳定RTT。

## 快照修复后的最终实测

固件47fcc87保留最新CAN快照，直到下一批或过期；没有新数据时保持批次序号和采样时间，
主机去重，不把重复快照当新反馈，不增加CAN重传或ACK。新回归覆盖空周期保留和过期清除，
完整固件协议回归通过，已JTAG烧录并校验。

主机工具使用优化编译（`g++ -O2 -DNDEBUG`，日志`compiler_optimized=1`）。以下两轮均为
CPU7、FIFO40、mlock成功，临时关闭专用网口L1/L1.1，退出后三项PCIe设置恢复为1。
每轮1000条失能帧、单个在途请求、发送间隔至少10ms，无超时、无OP/WKC丢失。

| PDO周期 | 交换频率 | 平均RTT | P99 | 最大RTT | 标准差 |
|---|---:|---:|---:|---:|---:|
| 250µs | 4kHz | 1129.737µs | 1249.695µs | 1498.634µs | 131.693µs |
| 125µs | 8kHz | 933.962µs | 999.882µs | 1124.419µs | 73.074µs |

8kHz最小623.537µs，P50=997.619µs，P95=999.447µs，峰峰值500.882µs。
**最大RTT仍超过1ms，不能宣称每个1kHz控制周期都能取得相应反馈。**
当前是低频单在途延迟台架，不是1kHz命令流、全部电机/UART、多板或长期压力验收。
控制频率继续为1kHz；CAN批次去重保证通信加速不会把同一命令重复发送为8kHz。
通用PDO模板仍默认1000µs，避免把大PDO/多板模板直接改成未经验证的125µs配置；
8kHz台架入口为`config/ethercat_dm_rtt_125us.yaml`，Rx96/Tx400B。

优化同时涉及周期、MCU轮询、快照修复、优化编译、调度与ASPM，不能归因于单一参数。
未在最终构建上完成ASPM开/关单变量对照；没有永久修改NUC的PCIe设置。
后续需要分析尾延迟，并进行1kHz命令负载验收。

- [4kHz日志](bench-results/2026-10-27-dm-rtt-250us.txt) / [CSV](bench-results/2026-10-27-dm-rtt-250us.csv)
- [8kHz日志](bench-results/2026-10-27-dm-rtt-125us.txt) / [CSV](bench-results/2026-10-27-dm-rtt-125us.csv)
- [8kHz时序与分布](bench-results/2026-10-27-dm-rtt-125us.png)
- [修复前ASPM关闭但丢反馈的失败记录](bench-results/2026-10-27-dm-rtt-250us-before-snapshot.txt)

脚本支持125/250/500µs，默认250。复位控制板后，8kHz复现命令：

```sh
sudo bash /tmp/dust-ecat-bench/ethercat_rtt_aspm_probe.sh 125
```

当前耗时口径、周期分桶、理论线时和分段测量设计见 [耗时构成分析](ETHERCAT_RTT_BREAKDOWN.md)。
