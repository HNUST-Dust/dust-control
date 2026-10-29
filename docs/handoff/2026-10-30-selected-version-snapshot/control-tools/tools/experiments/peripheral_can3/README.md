# CAN3 + UART2/5 独立 RTT 台架

当前实验更新：台架已按CAN帧sequence去重，兼容重叠快照窗口；主站正式 `EthercatSession` 的严格跨批次顺序校验尚未修改。固件 `DUST_ECAT_CAN_FEEDBACK_WINDOW` 默认关闭、仅独立 `can_window_diag.conf` 启用。实验不可直接用于正式部署。主机另存最后64个CAN批次的 `.can.csv`，失败时打印命令ACK。

窗口实验FIFO40已完成2000次，超时0/WKC缺失0，HI91 CRC错误0；SBUS收到4个frame_lost标志、failsafe0，UART块无序号缺口。详见 `docs/bench-results/2026-10-30-full-peripheral/can3-window-01`。后续旧段落为早期验证历史，当前固件为窗口实验版，不应按旧段落误判待测状态。

保持现有125us、Rx208/Tx688布局（TX预留6槽，实际仅发3帧），CAN2和IMU排除。固件为Mac `build/periph_can3`。本工具基于此前全外设台架，独立文件不覆盖原工具。

每10ms发送CAN3 `0x200` 八字节0力矩、`0x000`及`0x003`失能FF..FD；以CAN3 `0x013`反馈做单在途RTT配对，检查反馈报告失能状态。网关会Arm业务会话以允许转发，但不发送电机使能帧。失败时保留严格WKC截止期判据和部分数据，不重试命令配对。

新增UART2/5累计ACK，按新序号捕获数据，保存至结果CSV后的 `.uart.csv` 文件。预留40000个块内存，满时明确失败；采样期间不写UART文件、不在线拼帧或算CRC。离线 `analyze_uart.py` 检查HI91帧头/长度/CRC/姿态浮点有限性/设备时间推进，以及SBUS帧头尾、保护标志和块序号。遥控状态仅记录，不连接控制器。

编译（NUC `~/dust-ecat-bench`，已有 `include` 和 `pdo_config.cpp`）：

```bash
g++ -std=c++20 -O2 -Wall -Wextra -I include ethercat_peripheral_can3_rtt.cpp pdo_config.cpp -o dust_ethercat_peripheral_can3_rtt -lethercat -lyaml-cpp -pthread
```

`bench.cpp` 部署为 `ethercat_peripheral_can3_rtt.cpp`，`probe.sh` 部署为 `ethercat_peripheral_can3_probe.sh`。网关每次独立测试前必须复位并等待PREOP。ASPM必须已关闭；脚本只读取校验，不修改节能配置。正式对照：

```bash
sudo bash ~/dust-ecat-bench/ethercat_peripheral_can3_probe.sh 2000 peripheral-can3-fifo-01
```

该命令使用FIFO40，需要交互sudo密码。工具成功只表示RTT判据通过；两路UART是否有效、SBUS是否保护状态需结合离线报告，不能仅凭PASS行判定。

2026-10-30验证：编译及脚本语法检查通过，OTHER调度100次烟测通过，1000次OTHER长测在189次样本后发生一次截止期缺失并明确失败；详见 `docs/bench-results/2026-10-30-full-peripheral/can3-smoke-01` 和 `can3-other-1000-01`。FIFO正式测试待执行。
