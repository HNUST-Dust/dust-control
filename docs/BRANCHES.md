# 兵种分支与版本

| 分支 | 定位 |
|---|---|
| `main` | 项目公共结构和可共享基础设施 |
| `wbr_infantry` | 当前 wbr_infantry 机器人正式代码；生产入口使用 USB Bulk 网关 |
| `dev/wbr_infantry_ethercat` | 同一机器人的 EtherCAT 方案开发，尚不表示已替代 USB 实机路径 |

后续不同兵种的逻辑/算法在对应兵种分支演进。公共修复经审查后按需同步，
不要把一类机器人的增益、方向或控制策略直接覆盖到其他兵种。
EtherCAT 分支生产预设为 production，使用 IgH；USB 后端已移出此分支。
从站固件与硬件验收仍未完成，不能将编译成功当作实机可用。

`v0.1` 是 `wbr_infantry` 当前 USB 方案的带注释发布 tag。tag 固定一个源码提交，
后续修复使用新 tag，不移动已发布 tag。MPC 源码保留在当前分支，生产构建默认关闭。
EtherCAT 实现、头文件、应用、工具和测试仅在开发分支保留，正式 USB 分支已移除
相应代码与构建入口。`v0.1` 仍保留发布当时的源码快照，不随本次分支清理变化。

## CI

本分支 CI 使用 ethercat-ci、development、headless，默认 IgH OFF，测试注入原生 EtherCAT transport。
生产默认 IgH ON，缺少头文件/库配置失败；OFF 的真实启动报错，不回退 USB。
已有版本 tag 与历史 USB 测量继续保留原含义。
