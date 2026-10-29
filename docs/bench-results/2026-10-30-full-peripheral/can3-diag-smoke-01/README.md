# DM全链路内存记录：50次烟测通过

固件Mac `build/periph_can3_diag`，CAN profile及业务调度与前版一致。沿用实验专用 `CONFIG_DUST_ECAT_ESM_DIAG=y`，增加 `dust_can_flow_diag` 环形记录（512项，每项16字节，计数器及版本头16字节）。关闭该配置时辅助调用为空；不新增高频打印、不增加ESC读操作、不改反馈保留策略。记录时间为32位k_cycle_get_32原始值，可回绕，分析时须结合构建时钟换算。

事件：99=动作接收(batch,count)，100=含DM的硬件批次提交(count,result)，101=完成回调(error,bus)，102=DM CAN接收(seq,id)，103=入队(seq,result)，104=出队(seq,0)，106=新批次替换旧DM快照(old_seq,new_batch)，107=APPL_InputMapping首次映射该DM(seq,publication)，108=后续映射已不含该DM(old_seq,publication)。107不等于主站已读到，也不直接证明ESC写完成。入队/发送完成只代表各自阶段成功。

现有跟踪标记的DM批识别拓展至三帧批次，实际发送内容不变。IO三种原生模式、boot持久化故障检查、SSC五种原生模式及ESI/SII检查均通过（外设边界为mock，不能代替实机）。ROM191228B，RAM217640B。

主站新增64项CAN快照环形历史（仅新batch记录，退出写 `.can.csv`），超时时打印command_batch、commands_sent、ack_observed、last_accepted、publication和last_can_batch。既有8ms反馈窗口、WKC错误判据、禁止使能和UART ACK未改变。

诊断版已烧录，OTHER烟测50次成功，timeout0/WKC缺失0。均值1123.258us，仅说明诊断可运行，不作优化收益结论。固件458条事件，50条动作接收、50条出队、50条DM映射、50条旧快照替换和后续不含DM映射；启动/停止额外保护批次亦记录。阶段有完整样例，但烟测没有复现目标超时，不能据此确认覆盖就是失败根因。

ELF/HEX/.config及RAM快照保存在Mac `build/bench-host/peripheral-can3-diag-smoke-01`；本目录保存 `can-flow.json`、主站日志、RTT及末64个CAN快照。已再次复位，等待用户执行FIFO诊断：

```bash
sudo bash ~/dust-ecat-bench/ethercat_peripheral_can3_probe.sh 2000 peripheral-can3-diag-01
```

无论通过或失败，执行后先不要复位/烧录；应在主站退出后从当前诊断ELF解析符号，读取 `dust_can_flow_diag` 和CAN/ESM现场，恢复CPU运行，再作下一项测试。不要使用旧 `build/periph_can3` ELF解析新固件地址。
