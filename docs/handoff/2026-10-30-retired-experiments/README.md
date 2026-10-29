# 已退出使用的实验

## I225 接收时间戳诊断

`igc-rx-timestamp.tar.gz` 保存原 `tools/experiments/igc_rx_timestamp/` 和 `tests/tools/test_igc_rx_timestamp_decode.py`。归档前逐文件校验内容一致；SHA-256：`16706e68b6106e5e8f75a22cbde6c20a7597fb985e43c6201f343c5e5cc29796`。需要复现时，在独立目录解压；原实验对应的内核、驱动版本及哈希见包内 manifest。实验结果保留在 `docs/bench-results/2026-10-30-master-rx-diag/`。

当前选用配置使用原网卡驱动，未加载这一诊断模块。归档不改变 NUC 驱动、ASPM 持久配置或选用台架。

## 周期与接收相位实验

`cycle-phase.tar.gz` 保存周期解码器、相位与接收偏移台架脚本、接收相位源码及两个专属测试，归档前逐文件校验。SHA-256：`7ce89081cb8e4fa1f65f3b1135557b31c000493c34b05a267ca0e9ad056a14f0`。历史运行记录位于 `docs/bench-results/2026-10-29-cycle-phase/`；其中命令和源码名称记录的是当时环境，复现需先在独立目录解压对应工具。该归档不包含选用的 CAN3 外设台架。

## DC 与 SM 同步主站实验

`dc-sm-host-tools.tar.gz` 保存 DC profile 解码器及 DC、SM 同步台架脚本；归档前逐文件核验。SHA-256：`04221121a7e3546e3e7cc04cf5d9060510ba7d11626b9014ce68db82bddfa9e9`。历史记录中的工具路径为当时路径，复现或分析需先在独立目录解压归档。此归档不包括共享 RTT 工具、固件模式代码、选用 CAN3 台架或 ASPM 配置。

## SSC 输入 AL 事件读取实验

`ssc-al-read.tar.gz` 保存移除前的 Mac 固件 `Kconfig`、`CMakeLists.txt`、专属配置、构建调整脚本和原生测试；SHA-256：`60b39ddbfe4659c97396015c8da268691dbb6e607b39eb45676e6eb7363eb985`。其中 Kconfig 和 CMake 是混合工作树文件，恢复时只提取与该实验有关的片段，不得覆盖其他改动。选用配置未启用此实验。

## 摘要算法固件 A/B 开关

`reference-digest-before.tar.gz` 保存调整前的固件 Kconfig、Core 和等价测试脚本，SHA-256：`f68808d97b803cb3798169ed131295126a5301e9959ee4684d24332ffbdffe66`。这些文件包含其他工作树改动，只能按片段恢复。固件 Kconfig 的实验开关已移除；参考算法仍作为原生测试对照保留。

## 周期性 PHY 原始寄存器诊断

`phy-raw-before.tar.gz` 保存清理前的应用配置、主循环、PHY 驱动和相关测试；SHA-256：`6342aa3cf2ec544cd220f97dfe852849861facd6b11b6fe8618a53bc87b2e817`。这些文件来自混合工作树，只能按片段恢复。周期性原始诊断和启动时 PHY 扫描均已移除，正常链路维护保留。

## 启动 PHY 扫描与寄存器转储

`phy-startup-diagnostics-before.tar.gz` 保存清理前的 `src/main.c`，SHA-256：`1bc693b9c2eb3e68ee9d610ea2fdff0229fe6df4c18aece9189e6d8c124c7141`。文件包含其他工作树改动，仅可按片段恢复。扫描原含等待与 MDIO 操作；其移除对启动时序的影响仍需实机验证。

## 原生测试脚本调整前快照

`native-test-scripts-before.tar.gz` 保存 IO 和 SSC 测试脚本原版，SHA-256：`e2e369ae706a46ceb79cbd8328b890162d8a7ae96b67028b0deb7ada124fc83c`。当前脚本直接覆盖单线程基线与窗口，并兼容 Mac Bash 3 的空数组和 nounset。

## 固件 RTT 跟踪

`rtt-trace-before.tar.gz` 保存清理前跟踪实现、调用文件、配置与测试（SHA-256 `ea3aee09bb6300df624ab28de0d8f743dc5fff26cefb45b0ea3ea56c137ae436`）；`rtt-trace-test-support.tar.gz` 保存测试入口及 mock 环境（SHA-256 `5c4174c4cd980ced740964f1a7730da1774e1e91ac2819c968b33dab75d421cc`）。文件包含其他工作树改动，仅可按片段恢复。可启用实现、专属测试及调用点均已移除。

调用点清理前文件保存在 `rtt-call-sites-before.tar.gz`，SHA-256 `46a4ed2e642446b24d88cec0269e33d516c1308e129f6359ec2c9902e2a0f173`；文件包含其他改动，仅可按片段恢复。

## USB 固件方案

`usb-before.tar.gz` 保存移除前的 USB 入口、传输、协议、会话、专属驱动及测试，以及与 CAN 配置拆分相关的文件。SHA-256：`d8f67ea0741dfd74b998a6fc3ddd9df47054edc994854f4ed38e9f2827d85c0c`。归档含 EtherCAT 混合工作树文件，恢复只能按片段进行，不得整包覆盖。

## 主站 RTT/MCAN 跟踪与线程对比工具

`host-trace-thread-tools.tar.gz` 保存主站旧跟踪解码器、线程对比脚本、专属测试和删除前测试注册；SHA-256：`c0c1593a5866c753233350abb13acf8e0b54aa6b126c477a64f784b1819cb2fb`。文件包含未提交的实验改动，只能在独立目录恢复或按片段提取；历史结果和选用 CAN3 台架保留。
