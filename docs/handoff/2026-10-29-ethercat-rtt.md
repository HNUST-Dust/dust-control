# EtherCAT 固件与 RTT 优化交接（2026-10-29）

## 用户目标与已确定的方向

用户要求以 EtherCAT IgH 替换 USB 后端；从站继续作为 CAN/UART 网关，控制、电机编解码和传感器解析留在 NUC。接口和 PDO 容量由 YAML 配置，预计 1～3 块 HPM5E31 板。源码统一在 dust-adapter/src，不恢复 apps 目录。

用户明确决定保留多线程。当前采用多线程＋CAN 接收事件唤醒＋CPU 32 位 ESC 读写；DMA 关闭。下一步针对各阶段等待优化，优先解释 CAN 队列→工作线程约 28 µs、工作线程完成→SSC 主线程取得快照约 45 µs。不要重新争论单线程方案，也不要未经新证据重新启用 DMA。

## 仓库、环境和连接

当前 Linux 工作目录 `/home/panpoming/dust-control`：

| 仓库 | 分支 | 本轮完成时提交 |
|---|---|---|
| dust-control | dev/wbr_infantry_ethercat | 776c6ef182141efb598a3a6b1ac255cb0df61944 |
| dust-adapter | ethercat | 1d0866cec085845d2b17c4f2f8351737772ac39a |
| hpm_support | codex/dust-hpm5e31 | d1e9fbfbdca40c3ecc965795917735cb92bd6fd2 |

以上代码和测试结果已推送 GitHub HNUST-Dust 对应分支。本交接文档会产生后续 host 提交。固件 west.yml 固定了上述支持库版本。

固件共享路径：
`/Users/panpoming/Documents/zephyr_projects/dust-adapter-ws/dust-adapter`

支持库是其同级 `hpm_support`，SDK 是 `../sdk_env/hpm_sdk`。可从 Linux 读取共享路径；构建、JTAG 和 SSH 通过 Mac：

```bash
mac zsh -lc 'source /Users/panpoming/Documents/zephyr_projects/.venv/bin/activate && cd /Users/panpoming/Documents/zephyr_projects/dust-adapter-ws/dust-adapter && west build -d build/dm_rtt_event_perf'
mac zsh -lc 'ssh -o ConnectTimeout=5 wbr-nuc "ethercat slaves; ethercat master"'
```

NUC：`wbr-infantry@192.168.1.94`，Mac SSH alias `wbr-nuc`。基准工具持久目录 `~/dust-ecat-bench`；不要用 /tmp 保存 NUC 工具，重启会丢失。SSH 没有免密 sudo。需 root 的测试让用户启动一次协调脚本，随后通过普通用户 release 门闩协调各轮。

Linux 工具默认 sandbox 曾出现 SeccompInstall 错误；使用 require_escalated 走正常审批。不要绕过自动审批拒绝。未授权不要开新 agent。

## 台架操作边界

已授权的测试仅 CAN3 命令 ID `0x003`、反馈 ID `0x013`，只发失能帧：
`FF FF FF FF FF FF FF FD`。不发使能、运动、零位或错误清除。用户确认 DM 已供电、ID 正确。不要扩大为运动控制。

- CAN3 左腿；CAN2 疑似焊接问题，跳过。
- HI91：UART2，921600，8N1；遥控：SBUS/UART5，100000，8E2。
- SPI IMU 暂时禁用。
- 专用主站接口 ec_igc / I225-LM，PCI `0000:58:00.0`，MAC `54:b2:03:9e:f2:fa`。
- 用户已授权安全失能帧 bench 的 OP/Arm、固件烧录、RAM 导出及代码推送。
- JTAG halt/烧录前必须确认测试进程退出且 `ethercat master` 为 `Active: no`。不要在主站运行时暂停 MCU。
- ASPM 对比脚本临时关闭 L1/L1.1，退出恢复；不能改永久调度权限或系统网络设置。
- 实测命令间隔 10 ms，即 100 Hz；PDO 周期 125 µs，即 8 kHz。不是完整 1 kHz 业务负载测试，也没有硬实时最坏值保证。
- DM 配对是顺序应答，没有 nonce 回显，不能声称严格因果测量。

## 当前板卡状态

最终已烧录并校验 `build/dm_rtt_event_perf/zephyr/zephyr.hex`：多线程、事件唤醒、CPU 32 位 ESC、DTR 和 MCAN 时间戳均关闭。调度统计开启（用于公平对比，非 CPU 利用率计量）。

最后核查：唯一从站 PREOP，主站 Idle / Active:no；三个 ASPM 属性均恢复 1。

自己的从站身份：
- 名称 `DUST HPM5E31 Gateway LAB`
- VID `0x00000000`（实验身份）
- PID `0x44544331`，Revision `0x00010000`，Alias 0。

用户曾中途改固件/EEPROM，枚举为 digital_io / VID0x0048504d / PID1 / Alias13330，后明确要求恢复我们自己的。已恢复成功。我们的 EEPROM 是固件内置 2048 字节 RAM SII 仿真，每次启动从 aEepromData 初始化；无需另写外部 EEPROM。`ethercat sii_read` 导出的有效分类区为 266 字节，不表示 RAM 镜像只有266字节。不要随意擦除 Flash NVS/OTP。正式主机拒绝 VID0 的保护保持不变。

## 已解决的问题与测量结论

### 约 6 ms 异常的根因（已修复）

不是多线程天然开销。MCAN FIFO 曾持续占用8，硬件帧到读取约6.5 ms。支持库 d1e9fbf 修正：ISR 清标志后有界排空入口 FIFO；正确写入正向 CAN mask 和软件 AND 匹配；拒绝未注册帧；读取失败不回调旧缓冲。修正后 FIFO 基本为1、硬件 SOF 到读取约122 µs。不要回退这些修复。

### 同步 DMA 已实测更慢（保持关闭）

纠正 FIFO 后 CPU vs 同步 DMA 各64条：RTT均值872.004 vs961.917 µs；ESC读16.334 vs41.005、写54.468 vs93.961 µs。DMA 需要 bounce buffer、每次配置和忙等完成，不能与异步流水 DMA 混为一谈。旧 DMA 错误/超时计数为0，慢不等于出错。

### 单/多线程 ABBA（用户已经选择多线程）

trace/DMA/MCAN诊断均关、各2000样本：单线程均值720.616 µs，多线程788.532 µs，P99分别874.376/999.149 µs。此结果包含优先级/布局等变化，不全是上下文切换。见 `docs/bench-results/2026-10-28-thread-ab`。

### 最新有效优化：取消接收线程 100 µs 超时空唤醒

接收线程优先级0、SSC主线程优先级1。旧接收线程每100µs超时唤醒并扫描；现 `CONFIG_DUST_ECAT_CAN_RX_POLL_US=0`，用 K_FOREVER 等待信号量，IRQ到达立即唤醒。SSC在读取快照前按缓存最旧时间检查20ms过期，保留故障关闭语义，覆盖时间戳回绕、Stop和并发接收。

四轮 poll/event/event/poll、各1000条，关闭 trace，全部 PASS，无超时/WKC miss：

| 合并2000样本 | Poll100µs | Event |
|---|---:|---:|
| 平均 RTT µs | 848.673 | 749.799 |
| P50 | 873.736 | 748.967 |
| P95 | 999.146 | 873.997 |
| P99 | 1124.008 | 874.515 |
| 最大 | 1249.409 | 999.638 |
| >1ms | 41 | 0 |

均值改善98.874µs/11.65%。每轮空唤醒约50000→1；事件版每轮changed_batches=1000。已将默认改为0，并推送1d0866c。

原始CSV、日志、RAM统计、配置和构建hash：
`docs/bench-results/2026-10-29-can-wake/`

### 最新可信分段（修正观测竞态后，各64条，全64有效）

| MCU阶段 | Poll均值µs | Event均值µs | Event P95µs |
|---|---:|---:|---:|
| ESC读96字节 |16.098|13.811|13.792|
| 读完→Rx映射 |4.223|2.897|2.958|
| Rx映射→协议处理开始 |11.386|10.180|13.708|
| 协议处理 |24.573|19.875|22.042|
| 处理完→接受命令 |2.093|2.763|4.875|
| 接受→CAN入队 |2.878|2.010|2.708|
| 入队→提交CAN驱动 |6.513|4.562|6.042|
| CAN提交→反馈回调 |297.074|275.602|334.583|
| 回调→接收队列 |3.757|2.972|6.667|
| 接收队列→工作线程出队 |26.997|27.956|53.917|
| 出队→缓存完成 |6.157|2.427|5.875|
| 缓存完成→SSC取得快照 |65.204|45.370|71.667|
| 快照→发布 |17.438|18.746|24.333|
| 发布→输入映射 |8.857|5.744|9.000|
| 输入映射→ESC写开始 |1.684|1.010|2.083|
| ESC写400字节 |64.375|50.013|53.958|
| Rx映射→ESC写完（聚合） |538.985|469.230|585.458|

CAN提交→驱动返回23.171µs、→TX完成153.288µs是提交→反馈275.602µs的嵌套区间，不要重复相加。逐段P95不能相加为端到端P95。不能直接相减MCU与NUC绝对时间。

trace开启的事件版host均值781.865µs；不能和关闭trace的749.799µs混合统计。Host总RTT与MCU区间差值含PDO相位、网络、主站处理等，尚不能完全归因。

详表、原始文件和汇总：`docs/bench-results/2026-10-29-can-wake/corrected-trace/`。

## 观测竞态：必须保留的修复与旧数据限制

DTR1 v3新增 sequence-matched CAN_SNAPSHOT，原v1/v2解码兼容。最初仅要求工作线程CAPTURE后即可记PUBLISH；工作线程可能抢占正在发布旧快照的SSC，导致把旧发布归到新反馈。最初v3两轮有23/64及2/64条快照晚于发布。即便单条顺序单调也可能选错发布，故两轮都不能用于精确发布阶段分析；旧capture→publish结论同样有此局限。

1d0866c要求匹配的主线程CAN_SNAPSHOT后才允许PUBLISH。回归模拟“worker capture完成但SSC尚无新快照”的情况。补测两轮64/64均完整、零flag。原始失败观测数据保留，未伪装为有效。

DTR ABI：头40B，128条记录，v3每条192B，Python格式`<IIQIIIIii19Q>`。可选MCAN MCT1 sidecar保持不变。

`dust_ecat_sched_stats` 是28字节 `<7I>`：version, poll_us, active_wakes, active_timeouts, active_empty_wakes, active_changes, ssc_expirations。计数不是CPU占用率。

## 源码入口和线程所有权

固件：
- `src/main.c`：主循环 `dust_gateway_poll(); MainLoop(); dust_gateway_poll();`，活跃时busy_wait10µs。
- `src/ethercat/slave/io_backend.cpp`：CAN IRQ→msgq+sem、独立worker、generation、缓存/expiry、SSC快照复制。
- `src/ethercat/slave/gateway.cpp`：ServiceCanReceive、CaptureCanFeedback、Core.Publish、主线程额外有界CAN发布及PDO_InputMapping。
- `src/ethercat/slave/rtt_trace.cpp/.h`：有界记录及序号配对。
- `src/ethercat/core.cpp`：协议处理、Publish、CRC/Digest；后续可测，但尚未优化。
- `Kconfig`：CAN_RX_THREAD、CAN_RX_POLL_US默认0、SCHED_STATS。

主线程独占 SSC/Core、TX队列、UART窗口、ESC/PDO发布。CAN worker只更新接收缓存；mutex保护缓存，主线程复制到published_feedback稳定副本后再发布。不要让worker直接调用SSC/写ESC，也不要复用不稳定的buffer指针。

Host：
- `tools/ethercat_rtt_decode.py`：v1/v2/v3解码，独立时钟，不跨时钟相减。
- `tools/ethercat_can_wake.sh`：六轮root协调器，固定失能帧、编号门闩、可恢复。
- `tools/ethercat_thread_ab.sh` / summary.py：之前严格ABBA。

## 已运行验证

- 完整 `scripts/test_protocol.sh`（DUST_HOST_ROOT指向host）：SSC、504跨端案例、IO、UART、MCAN匹配/FIFO/时间戳、ESC字宽和DMA故障关闭均通过。
- 原生接收线程真实stdthread模型，K_FOREVER唤醒、Stop、expiry、32位时间回绕测试通过。
- `scripts/test_ethercat_rtt_trace.sh` 包含修正后的发布资格测试，通过。
- host 10个decode测试通过；旧v2实测重放与原结果字节一致。
- 四种实际固件配置构建通过；每对只改变poll_us。原始实际配置已归档。
- 所有实机轮次日志PASS，无超时/WKC miss；最终固件校验成功、从站稳定PREOP。

## 下一步建议（尚未实施）

1. 围绕接收队列→线程出队补充IRQ退出/调度入口、线程取sem返回时间，检查中断屏蔽、锁、调度和实际worker运行边界。不要先入为主归因为Zephyr切换成本，不恢复周期空轮询。
2. 围绕缓存完成→SSC快照，细分SSC循环相位、Core处理和映射前路径；探索主线程有界提前刷新，保留单一SSC所有者。
3. ESC400B写约50µs，可研究CPU字传输/实际需要字段，但目前无证据支持同步DMA。PDO容量减少必须继续符合YAML及网关能力需求。
4. CAN275.6µs需要硬件TX/RX时间戳或外部总线观测分离仲裁、线速、设备应答和ISR；不要拿整段当CPU忙等。
5. 每次只改一个因素，记录基线与候选实际配置和ELF hash，trace用于定位，关闭trace的ABBA用于量化。后续还需完整1kHz控制负载测试，用户未授权运动帧。

## 实机工具、临时协调器与恢复操作

OpenOCD：
`/Users/panpoming/Documents/zephyr_projects/openocd-hpm-rtt-v0.1.0/local-rtt/bin/openocd`

NM：
`/Users/panpoming/Documents/zephyr_projects/zephyr-sdk-0.16.5/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-nm`

JTAG参数：SDK openocd scripts目录；probes/cmsis_dap.cfg、soc/hpm5e00.cfg、boards/hpm5e00evk.cfg；cmsis-dap vid_pid0x1209 0x6666，backend usb_bulk，reset_config trst_only，adapter speed500。烧录前init/reset init，flash write_image erase指定hex，verify_image同hex，reset run/shutdown。只擦目标镜像覆盖页，不全片擦除。必须检查verified字节数，不能只看退出日志。

固件构建目录：`build/dm_rtt_{poll,event}_{trace,perf}`，实际测量中的96B主→从/400B从→主布局来自bench YAML。build目录内容会被后续编译覆盖，历史hash与结果在仓库归档。

RAM导出：固件 `tools/dump_ethercat_rtt_trace.py ELF OUTPUT --symbol dust_ecat_trace|dust_ecat_sched_stats --nm ... --openocd ... --scripts ../sdk_env/hpm_sdk/boards/openocd`。先确认master Idle，脚本halt/read/resume；必须使用匹配ELF地址。

本机临时文件（可能后续消失，不是唯一资料）：
- `/tmp/dust_can_wake_coordinate.py`、`/tmp/dust-can-wake-run.json`：六轮，run wake-1793194492。
- `/tmp/dust_can_trace_coordinate.py`、`/tmp/dust-can-trace-run.json`：修正后两轮，run wake-trace-1793256152。
- `/tmp/dust-can-{wake,trace}-coordinator.log`。
- 共享固件 `build/bench-host/<run>/`：OpenOCD校验、RAM导出日志、原始bin。
- NUC `~/dust-ecat-bench/<run>/status` 为COMPLETE；每轮CSV/log在bench根目录。
- NUC `ethercat_can_trace.sh` 两轮脚本目前仅在持久bench与共享build中，来源是六轮脚本固定缩减数组，并未作为独立host源码提交。

协调规则：master退出→导出上轮RAM→烧录/verify/reset→连续三次唯一LAB从站PREOP→写精确run:phase:variant release。NUC root脚本还会再次检查两次PREOP。不能重用已结束run的陈旧门闩。最后一次为恢复event_perf写了phase3 release，没有对应待运行root阶段，因此不会启动测试。

交接时没有待用户执行的命令，也没有活跃基准主站；下一位agent可以先只读核对环境，再按以上分析入口继续。
