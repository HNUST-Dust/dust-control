# 主站选型决策与上下位机最终方案

- 更新日期：2026-10-30
- 决策：上位机 EtherCAT 主站选用 **IgH（IgH EtherCAT Master 1.7）**，不采用 SOEM。

---

## 1. 决策结论

在相同条件下对 IgH 与 SOEM 两个主站做了 A/B 对比后，确定 **继续使用 IgH**。

- SOEM 均值 / 中位数比 IgH 慢约 **230–250 μs（约 2 个 125 μs 周期，均值 +34%）**，超出了“差距不大即可切换”的门槛。
- SOEM 的抖动更低（stddev 13.9 μs vs 78.1 μs），且部署更简单（纯用户态、无内核驱动、无需在 `ec_igc`↔`igc` 之间切换网卡驱动），但本次决策以**均值延迟**为硬指标，故不切换。
- IgH 已是本工程长期生产验证的主站，继续沿用风险最低。

### 对比数据（同固件、同配置、FIFO40/CPU7/125 μs、1000 样本）

| 指标 | IgH | SOEM | 差值 |
| --- | ---: | ---: | ---: |
| mean RTT | 731.357 μs | 981.755 μs | +250 μs (+34%) |
| p50 | 748.621 μs | 981.524 μs | +233 μs |
| p95 | 873.678 μs | 1004.730 μs | +131 μs |
| p99 | 874.271 μs | 1006.959 μs | +133 μs |
| max | 998.992 μs | 1223.609 μs | +225 μs |
| stddev | 78.136 μs | 13.886 μs | SOEM 低 5.6× |
| peak-to-peak | 377.133 μs | 270.643 μs | SOEM 低 28% |
| 超时 / WKC 缺失 | 0 / 0 | 0 / 0 | — |

> 当时使用 DM 失能帧 RTT 台架（CAN3 `0x003→0x013`），周期 125 μs，
> PDO Rx96/Tx400、signature `3858229428`；台架配置已清理。

---

## 2. 上位机（主站）方案

| 项目 | 取值 |
| --- | --- |
| 主机 | wbr-nuc |
| 内核 | `5.15.0-1114-realtime` |
| EtherCAT 主站 | IgH 1.7，链接 `/usr/local/lib/libethercat.so.1` |
| 网卡 | Intel I225（`8086:15f2`），PCI `0000:58:00.0`，驱动 `ec_igc` |
| ASPM | 三项关闭：`l1_aspm=0`、`l1_1_aspm=0`、`l1_1_pcipm=0`（systemd drop-in `60-disable-aspm.conf` 在 `ethercat.service` 启动后持久设置） |
| 调度 | `SCHED_FIFO` priority 40，CPU 7，`mlockall` |
| 同步模式 | FreeRun |
| 周期 | 125 μs |
| 代码仓库 | `dust-control`，分支 `dev/wbr_infantry_ethercat` |
| 测试工具 | 选型使用的 FreeRun DM 台架已从活动工具中移除 |

选型台架及原始实验数据已清理，对比指标保留在本文。

---

## 3. 下位机（从站固件）方案

| 项目 | 取值 |
| --- | --- |
| 板卡 | HPM5E31（`dust-hpm5e31`） |
| 代码仓库 | `dust-adapter`，分支 `ethercat` |
| 当前固件 | 清理版（清理了 USB 通信、RTT trace、ESM 诊断等），已烧录 |
| 固件 HEX SHA-256 | `83df0c61298c53069a6eb26bcd21f79c66c7f98759aa49aad0b67e61306a6c37` |
| 同步模式 | FreeRun（DC / SM 同步关闭，`DC_SUPPORTED=0`） |
| 周期 | 125 μs |
| 从站调度 | 单 SSC 主线程，priority 0；CAN/UART IRQ 入队 |
| 发布优化 | `DUST_ECAT_INPUT_LATE_REFRESH=y` |
| CAN | CAN3 三地址（`0x200` 零力矩、`0x000`/`0x003` 失能）；CAN2 排除（硬件问题） |
| UART | UART2 HI91（921600，RX 128 B）；UART5 SBUS（100000/8E2，RX 32 B） |
| CAN 反馈窗口 | `DUST_ECAT_CAN_FEEDBACK_WINDOW` 默认关闭，独立 `config/can_window_diag.conf` 启用 |
| IMU | 未接入（芯片 WHO_AM_I 0x47 与驱动期望 0x6A 不匹配，且 SPI3 无空闲 DMA） |

构建 / 烧录（Mac）：

```bash
cd /Users/panpoming/Documents/zephyr_projects/dust-adapter-ws/dust-adapter
export PATH=/Users/panpoming/Documents/zephyr_projects/.venv/bin:$PATH
export ZEPHYR_SDK_INSTALL_DIR=/Users/panpoming/Documents/zephyr_projects/zephyr-sdk-0.16.5
export CCACHE_DIR=/Users/panpoming/Documents/zephyr_projects/dust-adapter-ws/.ccache
west build -b dust-hpm5e31 -d build/periph_can3_window -- \
  -DEXTRA_CONF_FILE=config/can_window_diag.conf \
  -DDUST_ADAPTER_CAN_CONFIG=/Users/panpoming/Documents/zephyr_projects/dust-adapter-ws/dust-adapter/config/can_bench_can3_peripheral.yaml
west flash --skip-rebuild -d build/periph_can3_window
```

---

## 4. PDO 布局与同步

- 正式外设配置（CAN3 + UART2/5）：**Rx 208 / Tx 688 字节**，signature `2616507966`，TX 预留 6 槽、实际发送 3 帧。
- DM 台架配置（仅 CAN3）：**Rx 96 / Tx 400 字节**，signature `3858229428`。
- 同步：FreeRun，主站 `ecrt_master_set_send_interval(125)`，从站 `0x1C32/0x1C33 = FreeRun`。
- 从站 SSC：`external/ssc` FreeRun 生成源，`DC_SUPPORTED=0`、`AL_EVENT_ENABLED=1`、`INTERRUPTS_SUPPORTED=1`。

---

## 5. 已定位的关键约束（运行与维护必读）

1. **每次独立实机会话前必须复位网关**（断电重上电，或 Mac 上 `python3 build/reset_periph.py`）。上一轮 `kStop` 后从站核心进入 `Protected`，不复位会导致 `0x2100` SDO 配置被拒（abort `0x08000022`）、从站进不了 OP。
2. **ASPM 三项必须为 0**。开启 ASPM 会在 125 μs 下复现 WKC 持续为 0。
3. **CAN2 保持排除**（用户确认可能硬件问题），不要改回默认 `config/can_devices.yaml` 烧录。
4. **CAN 反馈窗口默认关闭**；窗口固件与正式主站的去重/兼容/丢失计数尚未完成，不能设为默认。
5. **IMU 未接**：芯片不匹配 + 无空闲 DMA，本方案不包含 IMU，相关代码保留但未验证。

---

## 6. 主站选型试验的可复现性

- IgH 对照台架已从活动工具中移除；对比结果保留在本文。
- SOEM 对照台架已随选型结束移除；对比结果保留在本文。对照时 I225 使用标准 `igc` 驱动（`enp88s0`），OP 过渡使用广播写 ALCTL（`BWR`），并在 OP 前写入合法 Discover 帧。

---

## 7. 遗留 / 后续

- 窗口固件与正式主站的协议兼容（能力协商、前缀重叠去重、序号回绕、淘汰计数语义）仍待单独完成，不随本决策推进。
- IMU 接入需先解决芯片 ID 与 DMA 通道分配，独立任务处理。
- SBUS 收到的 `frame_lost` 是接收机载荷标志，不等于 UART 丢块；遥控器关闭时全带 failsafe，属预期。
