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
- 主循环不打印每条样本，记录在内存，结束后写CSV；绑定当前允许的最高编号CPU，尝试FIFO60和内存锁定，
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
