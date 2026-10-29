# CAN2 故障隔离与 CAN3 外设启动排查

日期：2026-10-30。

## 已确认的启动阻塞

ESM 有界 RAM 记录表明：完整配置的 GenerateMapping 成功（Rx208/Tx896），StartInput 成功并进入 Ready；随后 `io_backend.cpp:890` 因 CAN 故障标志调用 `Core::Protect(Error::Transport)`。StartOutput **实际被调用**，但当时已经 Protected，因此返回 0x25。后来的 0x1b 是后续错误，不能据此将原始阻塞归因于 SSC watchdog。

full-esm-02 的 CanPort RAM 解码（每项2432字节，fault偏移2404）显示：

| CAN | fault | submitted | completed | received | errors | ready |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 0 | 0 | 0 | 0 | 0 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 1 |
| 2 | 1 | 3 | 0 | 0 | 1 | 0 |
| 3 | 0 | 6 | 2 | 436888 | 0 | 1 |

submitted 为帧数，completed 为批完成回调次数，不应直接相减解释丢帧。尚未记录 CAN2 原始驱动错误码，不能断言是无 ACK 或 bus-off。用户确认 CAN2 可能有硬件问题，要求暂时跳过。

## 隔离方式

Mac 固件工程新增独立 `config/can_bench_can3_peripheral.yaml`，从默认 profile 中移除 CAN2，保留 CAN3 三电机白名单和原有保护帧，CAN0/1空设备配置保留。默认 `can_devices.yaml` 不变；未放宽 Core 安全检查。

构建目录 `build/periph_can3`，额外配置 `config/esm_diag.conf`，CMake 参数 `DUST_ADAPTER_CAN_CONFIG` 指向上述独立 profile。ROM182236字节，RAM209416字节；已烧录。

主站独立配置 `~/dust-ecat-bench/ethercat_peripheral_can3.yaml`：125us，TX槽6，CAN3 RX槽8，CAN2 RX槽0，UART2 RX128、UART5 RX32，IMU false。实际 Rx208/Tx688，signature2616507966。

## 启动检查结果

所有检查均使用独立 startup probe，禁用 Arm 和业务命令生成，每周期清空64字节头后的业务区，仅发送 Discover。固件仍会发送规定的启动/停止保护帧。主站 OTHER 调度，结果只用于功能排查，不能作为 RTT 性能数据。

| 方案 | Rx/Tx字节 | 结果 |
| --- | --- | --- |
| CAN3+UART2/5，单域 LRW | 208/688 | OP，WKC0/3，10秒超时 |
| 同配置，收发独立域（LWR/LRD） | 208/688 | OP，两域均WKC0/1，10秒超时 |
| CAN3 RX8，无UART映射 | 176/496 | OP，WKC0，10秒超时 |
| CAN3 RX1，无UART映射 | 176/328 | OP，WKC0，10秒超时 |

CAN3+UART 的 ESM 记录确认 StartOutput 在 Ready 状态调用、返回0，并写入 AL状态8。退出检查时 StopInput/StopOutput 随主站释放发生，不能把退出后的 PREOP/Protected 当作运行期失败原因。

独立域实验期间读到的 FMMU：输出逻辑0、长度208、物理0x1100、type2、active1；输入逻辑208、长度688、物理0x1d00、type1、active1。SM2/3 长度与之匹配。映射寄存器已配置，但逻辑访问仍不产生有效 WKC，原因待查。

## 验证和后续

SSC 原生集成测试5种模式及 ESI/SII 检查通过。Mac 自带 Bash 对空数组与 nounset 的组合不兼容；运行时仅在内存脚本中关闭 nounset，保留 errexit、pipefail、断言和 sanitizer，仓库测试脚本未修改。

下一步检查逻辑数据报在主站发送、ESC处理、主站接收三个阶段的真实内容和 WKC；不要再把“OP成功”当作过程数据有效。UART映射关闭和缩小PDO均未解除故障，暂不依据这些实验修改调度或性能参数。尚未运行电机业务 RTT 测试，也未验证 HI91/SBUS 内容正确性。

原始主站日志见 `startup-logs.txt`。Mac RAM快照：`build/bench-host/full-esm-02`、`build/bench-host/peripheral-can3-01`。当前烧录仍为 CAN3 独立profile；最后检查已退出，主站回到非活动状态。下次独立启动检查前需复位，并等待主站重新发现从站（立即启动可能报零从站）。

## 后续 WKC 周期对照

同一固件、同一 Rx208/Tx688 配置，仍为 OTHER 调度：

| 周期 | 观察 |
| --- | --- |
| 125us | 原检查 OP、WKC0，10秒超时 |
| 250us | WKC0/3反复切换，累计16次有效Ready检查后退出；不代表连续稳定 |
| 500us | 短启动检查 OP/WKC3/Ready通过 |
| 1000us | 短启动检查 OP/WKC3/Ready通过 |

另外构建 `dust_ethercat_startup_hold_probe`：保持125us接收轮询，已有域数据报未完成时不再次复制输出/排队；超过2ms可重新排队以避免启动阶段永久卡住。只在新的完整接收后执行Ready检查，避免把同一个响应计数多次。结果16次新的OP/Ready接收通过，完整响应WKC3；全启动过程共20次完整域接收，主机观察排队到处理完成约247–498us，114次2ms释放计数包含启动阶段，不能解释为OP阶段丢包率。这是诊断性降发送频率，**不等于恢复125us有效PDO更新率**。

NUC保留的IgH源码 `~/dust-ecat-bench/rx-diag/baseline/master/master.c` 中，重复排队会将既有数据报状态设回QUEUED，发送时更新index；收包按index、SENT状态、type和size匹配。因此，“响应晚于下一次排队而无法匹配”已成为主要解释，但仍需实际收发记录进一步验证。500/1000us与hold检查通过，已说明FMMU/PDO路径并非持续失效。

发现重要测试条件差异：本轮非root直接运行startup工具，OTHER调度；I225的 `l1_aspm`、`l1_1_aspm`、`l1_1_pcipm` 均读到1。之前125us成功的性能测试使用 `ethercat_rtt_aspm_probe.sh`，临时关闭这些ASPM并以root运行、可启用FIFO。当前尚未分离ASPM、调度和报文周转各自影响，不能宣称ASPM就是根因。

原始日志和诊断源码归档于 `wkc-cycle-evidence.json`。已准备并语法检查 `tools/experiments/wkc_startup/probe.sh`，部署为NUC `~/dust-ecat-bench/wkc_startup_probe.sh`。它仅运行同一个禁Arm的startup工具，临时关闭三个ASPM开关，退出时恢复原值；root运行时工具会尝试FIFO40，实际调度方式见日志。该对照同时改变ASPM和调度，只用于先检查此前125us测试条件能否重现；若通过，还需单因素对照。

网关已再次复位，确认PREOP，等待用户在NUC执行 `sudo bash ~/dust-ecat-bench/wkc_startup_probe.sh`。`sudo -n true` 明确返回需要密码，尚未执行该特权对照；没有修改驱动或内核模块。

### 用户执行 ASPM关闭+FIFO40 对照：通过

用户已执行首轮脚本：125us、Rx208/Tx688、FIFO40、三个ASPM关闭，OP/WKC3/Ready有效16次检查通过，退出后三个ASPM均恢复1。日志归档 `wkc-aspmoff-fifo-01.txt`。因此原先持续WKC0不能再解释为持久性的ESC映射故障；ASPM和调度的影响尚未分离。16次短检查不足以证明长时间稳定。

保留原始probe二进制，另编译 `dust_ethercat_startup_can3_long_probe`：要求累计1000次有效Ready检查，新增OP阶段总周期和WKC缺失计数；通过行本身不代表零缺失，需同时查看 `OP_CYCLES ... WKC_MISSES ...`。安全命令限制与原probe相同。

脚本新增 `other` 模式，ASPM仍由root临时关闭，probe通过 `runuser -u "$SUDO_USER"` 使用普通用户运行。下一轮命令：`sudo bash ~/dust-ecat-bench/wkc_startup_probe.sh wkc-aspmoff-other-01 other`。延长后的同版probe还需补FIFO对照，以免将测试时长差异当作单因素结论。网关已复位并再次确认PREOP，等待该命令。

### ASPM关闭+OTHER通过，随后ASPM开启+OTHER重现失败

用户执行延长版OTHER对照：125us、Rx208/Tx688，三个ASPM关闭，`OP_CYCLES 1000 WKC_MISSES 0`，1000次Ready检查通过，三个开关退出后均恢复1。日志 `wkc-aspmoff-other-01.txt`。

随后再次复位，以同一个延长版二进制、同一个YAML、普通用户OTHER直接运行，保持三个ASPM开启。设备进入OP，但WKC持续0，10秒超时退出。日志 `wkc-aspm-on-other-long-01.txt`。

当前证据：在相同OTHER条件下，ASPM关闭可通过，恢复开启可重现失败；FIFO并非本轮启动检查通过的必要条件。这已把主要调查方向锁定到专用I225的PCIe节能配置，而非继续修改SSC映射。三个开关作为一组改变，尚未确定单个L1/L1.1选项贡献；尚未通过原始收发数据报证明具体延迟阶段，也未完成长时间带业务负载验证。1000周期约125ms，不能替代长期稳定性测试。

后续CAN3及UART验证应沿用已通过条件：专用I225临时关闭上述ASPM，性能测试仍使用FIFO，退出恢复原值；不得把OTHER下的功能检查当作性能结果。本轮无永久系统设置变更、无驱动或内核模块替换，CAN2仍排除。

### 用户要求持久关闭ASPM：已安装并复测

用户随后明确要求关闭节能。已安装专用I225的ASPM关闭脚本和EtherCAT服务ExecStartPost配置（详见 `tools/system/ethercat-aspm/README.md`），三个开关均验证为0；服务开机启用，尚未实测系统重启。此前“无永久设置变更”仅描述当时实验阶段。

网关复位后不使用临时ASPM包装器，直接普通用户运行同一125us延长版probe：`OP_CYCLES 1000 WKC_MISSES 0`，OP/WKC3/Ready通过，日志 `wkc-persistent-other-01.txt`。本轮仍为禁Arm启动检查，无电机业务RTT数据。后续实验恢复ASPM时应恢复保存的0值。
