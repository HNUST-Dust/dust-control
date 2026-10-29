# 独立CAN历史窗口实验：100次烟测通过，待FIFO长测

Mac固件 `build/periph_can3_window`，额外配置 `config/can_window_diag.conf`：ESM/CAN链路诊断及 `DUST_ECAT_CAN_FEEDBACK_WINDOW=y`。该新选项默认n，仅单线程使用，与旧正式主站不兼容（旧主站跨批次重叠序号会报乱序）；正式主站及默认配置均未切换，未提交或合并。

行为：有新CAN帧时，将未超过20ms的旧事件保留在配置容量内（当前8槽），按接收序号排列，满时淘汰最旧事件；原有无新帧时的过期检查保留。不是按CAN ID合并、不加ACK、不增加PDO长度，不能保证任意负载下每帧必达。容量淘汰沿用drops计数，因滚动窗口会经常淘汰已被主站观察过的旧帧，该计数不能直接解释为主站遗漏；正式集成前需处理诊断口径。

配套独立台架跳过已消费的frame sequence，按新帧计数并配对DM；此前8ms反馈超时和严格WKC判据均未放宽。未改变电机发送内容、周期、槽数或UART ACK。正式 EthercatSession 未修改，不得混配部署。

原生验证：默认backend/可选线程变体及窗口变体通过，新断言覆盖后续接收仍保留前一帧、容量2满时淘汰最旧帧、20ms后过期。用mock边界验证，不代表硬件稳定性。

实机OTHER烟测100次通过，timeouts0/WKC_misses0；RTT均值1175.719us、P95 1248.799us、最大1373.359us，仅说明功能可运行，不与FIFO失败会话比较收益。末64个CAN快照count均8，63对相邻快照均存在序号重叠，台架仍仅计100次DM反馈，未因重复快照多计。

诊断RAM/匹配ELF/HEX/.config保存在Mac `build/bench-host/peripheral-can3-window-smoke-01`。固件ROM191316B、RAM217640B。当前仍安装该实验版，已复位到PREOP，等待：

```bash
sudo bash ~/dust-ecat-bench/ethercat_peripheral_can3_probe.sh 2000 peripheral-can3-window-01
```

运行后先不要复位，导出窗口版ELF对应的 `dust_can_flow_diag`、CAN/ESM现场并与主站 `.can.csv` 对齐；后续导出不能使用 `periph_can3_diag` 旧ELF。未完成2000次FIFO验证，不能宣称修复完成。
