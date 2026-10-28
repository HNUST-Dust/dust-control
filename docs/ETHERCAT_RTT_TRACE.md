# DM RTT 分段时间戳

适用于 CAN3 命令 ID `0x003` 的 `FF FF FF FF FF FF FF FD` 失能帧与反馈 ID `0x013`，每次只有一条在途命令。协议、PDO 布局和发送白名单保持不变。DM 没有回显命令序号，因此仍是顺序配对，不能证明电机对每条命令严格一一应答。

## 主机记录

`dust_ethercat_dm_rtt CONFIG.yaml RESULTS.csv 64` 在原 RTT CSV 后追加以下列。所有 `*_ns` 来自同一个 `steady_clock`，整数纳秒；表示读取时钟的分辨率，不表示测量精度达到 1 ns。

| 列 | 测量位置 |
|---|---|
| `boot,session,batch,exchange` | 从站启动编号、会话、命令批次及首次发送交换编号 |
| `publication,can_sequence` | 主机实际读到的发布序号、该 CAN 帧接收序号 |
| `built_ns` | 开始构造命令 |
| `encoded_ns` | CAN 命令编码完成 |
| `output_copied_ns` | RxPDO 拷入 IgH domain 完成 |
| `queued_ns` | `ecrt_domain_queue` 返回 |
| `tx_ns` | 原 RTT 起点，在发送前的台架状态登记之前，保留历史口径 |
| `send_enter_ns,send_return_ns` | `ecrt_master_send` 调用前后 |
| `receive_enter_ns,rx_ns` | 包含目标反馈的 `ecrt_master_receive` 调用前后；`rx_ns` 为原 RTT 终点 |
| `domain_processed_ns` | `ecrt_domain_process` 返回 |
| `image_copied_ns` | TxPDO 从 domain 拷出完成 |
| `decoded_ns` | 反馈头部、身份、状态及配对检查完成 |
| `bench_consumed_ns` | 台架接纳反馈；不是机器人控制器消费时刻 |
| `accepted_observed_ns` | 主机首次观察到对应 accepted batch，0 表示未观察到 |

没有启动实际 1 kHz 控制器，不能据此宣称测到了控制器下一次读取反馈的等待时间；NIC 收发硬件时间戳及内核驱动排队也未单独测量。

## 从站记录

固件增加默认关闭的 `CONFIG_DUST_ECAT_RTT_TRACE=y`。记录在 `dust_ecat_trace` RAM 符号中，128 条上限，满后计数溢出并停止记录，不覆盖旧记录。首轮用 64 条命令，避免溢出。停止主站后再通过 JTAG 暂停、导出、恢复 CPU；测试中不打印日志、不分配内存、不暂停 CPU。

| 阶段 | 含义 |
|---|---|
| `esc_read_begin,esc_read_end` | SSC 读取 SM2 数据前后 |
| `rx_map` | RxPDO 拷入应用镜像完成 |
| `process,process_done` | `Core::Process` 前、成功返回后 |
| `accept` | IO 策略与容量检查通过、准备排队 |
| `enqueue` | CAN 软件队列写入完成 |
| `submit,submitted` | MCAN 批量提交 API 调用前后，并记录返回码 |
| `tx_done` | CAN 发送完成回调入口，记录回调结果 |
| `can_rx` | CAN 反馈回调，记录接收序号 |
| `capture` | 首次捕获包含该接收序号的反馈快照 |
| `publish` | 首次成功发布对应快照，记录发布序号 |
| `input_map` | 对应发布或后续保留发布拷入 SSC 完成 |
| `esc_begin,esc_end` | 对应 SM3 数据实际写入 ESC 前后 |

从站使用 `k_cycle_get_64()`，频率写入二进制头（当前为 24 MHz），不使用业务看门狗的 uptime 微秒时钟。CAN 回调表示软件观察时刻；TX 完成回调与 RX 回调可能因中断处理顺序而交换先后，不能将两者之差直接解释为电机计算时间。`submit → can_rx` 包含 CAN 发送、仲裁、电机应答、反馈传输及中断延迟。

写记录使用短临界区，阶段只写首次时间戳；发布必须在匹配 CAN 序号被捕获后才记录。诊断会增加少量开销，需要后续与关闭诊断的固件对照。

二进制 ABI v1：40 字节头 `<8IQ`，依次为 magic `0x31525444`、版本、记录字节数、容量、数量、溢出次数、时钟 Hz、头长度、boot。每条 168 字节 `<IIQIIIIii16Q`：revision、valid 位图、session、batch、exchange、publication、CAN 序号、提交结果、完成结果、上述 16 个周期计数。revision 奇数表示写入未完成。不要在其他版本之间硬套布局。

## 构建、测试和导出

固件工作区：

```bash
west build -b dust-hpm5e31 -d build/dm_rtt_trace -- \
  -DDUST_ADAPTER_CAN_CONFIG="$PWD/config/can_bench_dm_rtt.yaml" \
  -DCONFIG_DUST_ECAT_RTT_TRACE=y -DCONFIG_BUILD_OUTPUT_BIN=y
```

NUC 已部署工具后，沿用临时 ASPM 脚本，128 次上限内建议采 64 次。第三个参数区分输出文件，默认参数仍兼容原先的 1000 次测试。

```bash
sudo bash /tmp/dust-ecat-bench/ethercat_rtt_aspm_probe.sh 125 64 trace
```

退出后先保存 `/tmp/dust-ecat-bench/dm-rtt-125us-trace.{csv,log}`。下次运行前复位从站。尚未导出时不要复位，否则 RAM 数据丢失。

在固件仓库运行（路径按本机调整）：

```bash
python3 tools/dump_ethercat_rtt_trace.py \
  build/dm_rtt_trace/zephyr/zephyr.elf build/dm-rtt-trace.bin \
  --nm /path/to/riscv64-zephyr-elf-nm \
  --openocd /path/to/openocd \
  --scripts ../sdk_env/hpm_sdk/boards/openocd
```

脚本从配套 ELF 查地址和长度，导出后恢复 CPU，不复位。请先确认主站测试已结束。

主机仓库合并：

```bash
python3 tools/ethercat_rtt_decode.py dm-rtt-trace.bin \
  dm-rtt-125us-trace.csv dm-rtt-125us-stages.csv
python3 -m unittest discover -s tests/tools -v
```

按 `(boot, session, batch)` 关联，再核对命令交换编号先后、CAN 接收序号和发布先后。主机连续发送相同批次，SSC 可能读取到后续交换，因此 `accepted_exchange_advance` 为非负推进量，不强求 exchange 相同。主机可能读到保留快照的后续发布，因此不强求 publication 完全相同。不完整、缺失、错配和溢出都会显式标记；解析器返回非零表示存在问题。

两端时钟没有同步，不直接相减 NUC 与 MCU 的绝对时间戳。分段 CSV 只输出各自时钟内的耗时，保留所有原始周期计数和主机时间戳，便于复核。EtherCAT 单向线缆/网卡耗时不能用这些软件时间戳严格拆出。

## 2026-10-28 首轮实测

125 µs PDO、FIFO40、临时关闭 ASPM、64 条失能命令，0 超时、0 WKC 丢失。固件 trace 开启，RAM 227180/262144 B（86.66%）。所有 64 条记录 16 个点完整、驱动提交和完成返回码均为 0、CAN 序号吻合、无溢出。40 条在首次交换被接纳，24 条在下一次交换被接纳；重复 PDO 使用相同命令批次，不重复发送 CAN。64 条均读取到首次包含该 CAN 反馈的发布。

| 测量段 | 平均 µs | P95 µs | 最大 µs |
|---|---:|---:|---:|
| ESC读取96 B | 31.446 | 31.000 | 50.333 |
| Rx映射→协议处理 | 3.246 | 4.500 | 7.875 |
| 协议处理 | 19.008 | 21.125 | 42.000 |
| IO接纳→CAN提交 | 0.989 | 1.042 | 5.250 |
| CAN队列等待 | 6.329 | 6.917 | 19.250 |
| MCAN提交→反馈回调 | 271.335 | 356.167 | 380.042 |
| 反馈回调→快照捕获 | 168.611 | 205.000 | 242.625 |
| 捕获→发布 | 16.363 | 20.583 | 40.417 |
| 发布→SSC输入映射 | 5.092 | 4.792 | 24.917 |
| ESC写入400 B | 98.114 | 115.458 | 122.958 |
| Rx映射→反馈写入完成 | 594.376 | 629.417 | 689.083 |
| NUC 原口径 RTT | 946.210 | 999.334 | 1124.010 |

各行存在包含关系，不能全部相加。例如 MCAN 提交调用、TX 完成均包含在“MCAN 提交→反馈回调”内。首条冷启动样本没有剔除。样本量 64 只用于分段诊断，不能代替长时间尾延迟验证。

最明确的两个软件改进方向：

1. 当前 ESC 驱动大块写入逐字节进行，400 B 写入平均 98.114 µs；96 B 读取平均 31.446 µs。仅两次搬运已平均 129.560 µs，超过 125 µs 主站周期，尚未计协议处理、主循环、其他 ESC 访问。主站 8 kHz 交换不代表从站应用能以 8 kHz 完整处理每个 PDO。后续应核对 HPM5E31 ESC 的合法访问宽度，再评估对齐批量读写，不能未经芯片约束核查直接替换。
2. CAN IRQ 已收到反馈后，等待捕获平均 168.611 µs、最大 242.625 µs，说明反馈发布受应用处理周期影响。应检查 FreeRun 事件调度和发布路径，再考虑提高主站频率。

主机发送调用平均 0.434 µs，接收调用平均 0.820 µs；`rx_ns` 到台架接纳反馈平均约 0.78 µs。这些用户态函数耗时不包含完整内核/NIC/线上传播延迟。CAN 提交到反馈回调约 271 µs 仍包含总线仲裁、发送、设备应答、反馈与 IRQ，不能全归为电机处理。

原始数据：[主机 CSV](bench-results/2026-10-28-dm-rtt-125us-trace.csv)、[从站 RAM](bench-results/2026-10-28-dm-rtt-125us-trace.bin)、[合并分段 CSV](bench-results/2026-10-28-dm-rtt-125us-stages.csv)、[终端输出](bench-results/2026-10-28-dm-rtt-125us-trace.txt)。

## 2026-10-28 对齐 32 位访问对比

实现位于 hpm_support 的 `drivers/ethercat/esc_hpmicro.c`，开关 `CONFIG_HPM_ESC_PDRAM_WORD_ACCESS`。支持库默认关闭，dust-adapter EtherCAT 应用明确开启。关闭即可回退逐字节路径。只扩大 PDRAM 范围内地址、长度都按 4 字节对齐且长度至少 8 字节的访问；不补齐长度，不修改相邻字节，不扩大寄存器访问。应用缓冲区不对齐仍可使用 word MMIO 和 CPU 侧 memcpy。对齐缓冲区用显式 builtin memcpy 保持类型规则并生成直接 lw/sw。未引入 DMA。

约束依据：[ETG ESC 概览](https://www.ethercat.org/download/documents/ESC_Overview.pdf) 列出 HPM5E00 内部 32 位 PDI；本地 SDK 提供 DWORD ESC 读写及对齐四字节读取实现。手册第 57 章说明 PDRAM 与 SYNC DMA 请求。驱动对新路径增加范围、长度及对齐限制。硬件校验确认了本次 96/400 B PDO 和动态 CoE/邮箱配置，但不等于验证了所有未来 YAML 组合。

三轮均为 125 µs、FIFO40、CPU7、临时关闭 ASPM 并恢复，主机工具、PDO 配置及 DM 失能命令相同。均无超时和 WKC 失败。

| 平均耗时 µs | 逐字节，64 条 | 首版32位，前128条 | 最终32位，64条 |
|---|---:|---:|---:|
| 96 B ESC 读取 | 31.446 | 13.904 | 14.013 |
| 400 B ESC 写入 | 98.114 | 48.930 | 49.343 |
| CAN 提交→反馈回调 | 271.335 | 283.834 | 280.626 |
| CAN 回调→捕获 | 168.611 | 114.182 | 67.471 |
| Rx 映射→ESC反馈写完 | 594.376 | 496.618 | 444.333 |
| NUC RTT（有分段记录部分） | 946.210 | 801.755 | 746.987 |

首版 32 位测试实际运行 1000 条（主机平均 RTT 777.478 µs），从站按容量保留前 128 条并累计 872 次未记录；分段统计只使用这 128 条，不把剩余命令混入分段结果。最终版本采 64 条，64 条均完整匹配、无溢出。

最终版本平均 RTT **746.987 µs**、P95 **874.003 µs**、最大 **998.614 µs**、标准差 **68.073 µs**。与逐字节 64 条运行相比平均 RTT 下降约 21.1%，ESC 读写合计从 **129.560 µs** 降至 **63.356 µs**（约 51.1%）。样本量较小、相位和中断可能变化，因此整体改善不能全部等同于直接节省的搬运时间，更不能据此承诺长期小于 1 ms。

反汇编确认最终对齐路径已移除每四字节的 libc memcpy 调用，但两版 word 路径的 ESC 读写时间接近，未观察到进一步明显下降。剩余延迟可能主要来自 ESC 访问路径与等待；DMA 是否更快仍待独立测量，SYNC DMA 请求存在不等于搬运带宽已经验证。DMA 尚未接入。

测试：实际驱动函数的原生总线模型在 ASan/UBSan 下验证 byte/word 两种模式，检查访问宽度、顺序、计数、范围、零长度、越界/整数溢出拒绝和未对齐应用缓冲区；反汇编确认对齐路径实际使用 lw/sw。固件完整协议/IO 回归及实机收发均通过。

数据：

- 首版：[完整1000条主机CSV](bench-results/2026-10-28-dm-rtt-125us-word.csv)、[前128条主机子集](bench-results/2026-10-28-dm-rtt-125us-word-traced-host.csv)、[128条分段](bench-results/2026-10-28-dm-rtt-125us-word-stages.csv)、[RAM](bench-results/2026-10-28-dm-rtt-125us-word.bin)、[日志](bench-results/2026-10-28-dm-rtt-125us-word.txt)。
- 最终：[64条主机CSV](bench-results/2026-10-28-dm-rtt-125us-wordfast.csv)、[分段CSV](bench-results/2026-10-28-dm-rtt-125us-wordfast-stages.csv)、[RAM](bench-results/2026-10-28-dm-rtt-125us-wordfast.bin)、[日志](bench-results/2026-10-28-dm-rtt-125us-wordfast.txt)、[构建指纹](bench-results/2026-10-28-dm-rtt-125us-wordfast.json)。

复测当前正式 word 路径沿用前述 trace 构建步骤。复现原始 byte 基线时，构建命令额外指定 `-DCONFIG_HPM_ESC_PDRAM_WORD_ACCESS=n`。每轮测试前复位，导出前不要复位。
