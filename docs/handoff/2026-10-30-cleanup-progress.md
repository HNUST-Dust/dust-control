# 选用版本清理进展

2026-10-30。基线和恢复方法见 `2026-10-30-selected-version-refactor.md`；原始固件及快照未改动。

## 已完成

Mac `dust-adapter` 工作树已移除临时 ESM/CAN 链路跟踪、`dust_config_diag`、相关调用和配置；`config/can_window_diag.conf` 仅启用 CAN 反馈窗口。保留保护状态机、CAN3 profile、窗口默认关闭、IMU 待完成工作和 late refresh。没有烧录。

独立构建 `build/periph_can3_window_cleanup` 成功：ROM 180412 B、RAM 208360 B；冻结基线分别为 191316 B、217640 B。原生 IO 基线及窗口变体和 SSC 测试通过。Mac Bash 3 的空数组与 nounset 不兼容，测试采用交接文档中的临时脚本方式；SSC 原始脚本直接运行因该兼容问题失败，兼容方式重跑通过。实机 2000 次及 UART 校验尚未运行。

## 后续清理边界

当前选用构建启用 `DUST_ECAT_INPUT_LATE_REFRESH` 和 `DUST_ECAT_CAN_FEEDBACK_WINDOW`，关闭 CAN RX 线程、RTT 跟踪及 MCAN RX 时间诊断。旧 DC、线程和跟踪方案仍与固件 Kconfig、CMake、原生测试、PHY 和 PLIC 测试相互引用；Linux 的历史解码工具也仍被对应测试引用。应先针对每组完整归档并更新依赖、测试和证据索引，再删除；不能仅因为选用配置未启用就删除共享文件。

窗口固件与正式主站仍不兼容。能力协商、重叠前缀校验、序号回绕与 drops 语义须独立完成；本次清理不改变协议，也不证明长期稳定性。

2026-10-30 后续：I225 RX 时间戳实验及专属测试已逐文件校验归档至 `2026-10-30-retired-experiments/`，活动目录已移除，结果文档引用已更新。保留的 RTT 和周期解码测试 15 项通过。其他周期、DC 和相位工具仍有历史结果引用，待成组归档并建立索引。

周期/接收相位实验及其专属测试已逐文件校验后归档至 `2026-10-30-retired-experiments/cycle-phase.tar.gz`，活动目录已移除；历史记录保留。

DC profile 解码器和 DC/SM 主站实验脚本已逐文件校验后归档至 `2026-10-30-retired-experiments/dc-sm-host-tools.tar.gz`，活动目录已移除；历史记录保留。

SSC 输入映射跳过 AL 事件读取的独立实验已归档并从 Mac 固件构建中移除，相关审计文档已标记退役。选用配置重新构建通过（ROM 180412 B、RAM 208360 B），SSC 集成回归通过；没有烧录。

摘要参考算法已移出固件 Kconfig，改为原生测试专用开关；CRC、Core 合约和完整 PDO 字节等价测试通过。选用固件重新构建通过（ROM 180412 B、RAM 208360 B），未烧录。

已移除应用与 hpm_support 中可选的周期性 PHY 原始寄存器打印和额外读取，保留正常链路维护及启动扫描。实际 PHY 链路检测、锁存低电平、读取失败和速率/双工原生合约通过；选用固件构建通过（ROM 180412 B、RAM 208360 B）。未烧录。

启动 PHY 扫描、寄存器转储和专属状态已移除；正常 PHY 初始化及链路更新保留。选用配置构建成功（ROM 178928 B、RAM 208296 B），PHY 合约通过。扫描原含 MDIO 读取和等待，启动时序须实机验证；尚未烧录。

正式 IO 原生测试脚本已加入单线程窗口变体，IO 和 SSC 脚本的空数组展开兼容 Mac Bash 3，保留 nounset、errexit、pipefail 和 sanitizer。两个脚本均直接运行通过，不再需要临时脚本。

RTT 跟踪的 Kconfig、可启用实现、构建入口和专属测试已移除；保留的无操作调用点仍待后续清理。IO 基线/窗口/线程变体及 SSC 集成测试通过，选用配置构建成功（ROM 178928 B、RAM 208296 B）；未烧录。

RTT 无操作调用、专用条件与 CAN 跟踪状态已移除，活动代码无残留引用。IO（含窗口）和 SSC 测试通过；选用固件构建成功（ROM 178752 B、RAM 208296 B），未烧录。

清理后固件已烧录（HEX SHA-256 `83df0c61298c53069a6eb26bcd21f79c66c7f98759aa49aad0b67e61306a6c37`，ELF `27ccc5de0525a58ce59d71981093e7e0c87393895ecb72fb13405467660216db`）。烧录后主站 Idle/Active no、从站 PREOP，ASPM 三项为 0，台架二进制哈希匹配冻结版本。NUC sudo 需要交互密码，2000 次会话尚未启动；预留标签 `cleanup-20261030-01`。

清理后固件已完成 `cleanup-20261030-01` 实机会话：2000 命令、超时 0、WKC 缺失 0；HI91 CRC/块序号错误为 0。SBUS 1446 个结构有效帧均报告 frame_lost 和 failsafe，原因未确认，遥控状态尚未通过。原始数据与分析见 `docs/bench-results/2026-10-30-full-peripheral/cleanup-20261030-01/`；主站结束后 Idle/非活动。

用户确认 cleanup-20261030-01 运行时遥控器关闭，SBUS failsafe/frame_lost 与该状态一致；本次没有验证遥控器开启时的状态。

已移除固件旧 USB 通信方案、专属驱动/测试及板级 USB 配置；CAN 保护配置改用独立帧结构，保留白名单、帧内容和签名。选用固件构建 HEX SHA-256 仍为 `83df0c61298c53069a6eb26bcd21f79c66c7f98759aa49aad0b67e61306a6c37`，与已烧录并完成 CAN3/WKC 实测的固件字节一致。配置生成器 13 项、IO（含窗口）、SSC、Core、PDO 和 PHY 原生检查通过。完整 `test_protocol.sh` 在 Mac 因既有 `test_sync0_deferred.sh` Bash 3 空数组问题及 hpm_support UART 驱动原生测试 Mach-O section 属性编译失败，不能标记完整套件通过；主站 `architecture_boundaries.py .` 扫描 tests/SOEM 第三方头文件失败，需按工程既有方式或隔离第三方后再检查。

主站旧 RTT/MCAN 跟踪解码与线程对比工具及专属测试已校验归档并从活动目录移除。主站架构检查已排除第三方 tests/SOEM，检查通过；Sync0 测试脚本兼容 Mac Bash 3 并直接通过。选用 CAN3 UART 分析器以本次实机数据运行通过。hpm_support 的 UART 驱动原生测试仍受 Mac Mach-O section 属性编译问题影响，完整 test_protocol 尚未通过。
