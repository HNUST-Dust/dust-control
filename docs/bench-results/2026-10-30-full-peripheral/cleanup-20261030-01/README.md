# 清理后固件实机验证

2026-10-30，用户交互运行 `sudo bash ~/dust-ecat-bench/ethercat_peripheral_can3_probe.sh 2000 cleanup-20261030-01`。测试前重新烧录并复位；主站 Idle/非活动，从站 PREOP，I225 ASPM 三项均关闭。清理后固件 HEX SHA-256 `83df0c61298c53069a6eb26bcd21f79c66c7f98759aa49aad0b67e61306a6c37`，ELF `27ccc5de0525a58ce59d71981093e7e0c87393895ecb72fb13405467660216db`。台架二进制 `613485080b4e8ea5c2649112fe6737137d05d9d95f4237781c31862a5a692f93`，YAML `2592bc03610ab26e98f77297b82f61f33870824b97222157964f9975878249a5`。

配置为 FreeRun、125 μs、Rx208/Tx688、signature 2616507966、FIFO40、CPU7、mlock；仅 CAN3 零力矩/失能和 UART2/5 接收，跳过 CAN2、板载 IMU。测试完成后主站 Idle/非活动。

- 命令 2000，顺序配对 2000，超时 0，WKC 缺失 0。
- RTT μs：均值 1179.660，P50 1123.756，P95 1373.376，P99 1374.101，最大 1498.865。
- CAN3 去重反馈：0x201=21350、0x10=3999、0x13=2000，包含准备阶段。
- HI91：20234 个有效帧，CRC 错误 0、块序号缺口 0、设备时间不推进 0。
- SBUS：1446 个结构有效帧，块序号缺口 0，但 1446 帧均报告 frame_lost 和 failsafe。标志来自接收机载荷，不等于 UART 丢块；用户确认该次测试时遥控器处于关闭状态；该会话未验证遥控器开启时的正常状态。

原始日志与 CSV 随本文件保存；`uart-summary.json` 由 `tools/experiments/peripheral_can3/analyze_uart.py` 离线生成。此结果验证本次约 20 秒会话，不证明长期稳定性或硬实时。清理后 PHY 启动扫描被移除，本次完成启动与会话，但不替代多次冷启动或断电重启验证。
