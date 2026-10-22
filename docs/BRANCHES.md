# 兵种分支与版本

| 分支 | 定位 |
|---|---|
| `main` | 项目公共结构和可共享基础设施 |
| `wbr_infantry` | 当前 wbr_infantry 机器人正式代码；生产入口使用 USB Bulk 网关 |
| `dev/wbr_infantry_ethercat` | 同一机器人的 EtherCAT 方案开发，尚不表示已替代 USB 实机路径 |

后续不同兵种的逻辑/算法在对应兵种分支演进。公共修复经审查后按需同步，
不要把一类机器人的增益、方向或控制策略直接覆盖到其他兵种。
EtherCAT 分支默认构建 `ethercat-development` 预设，启用已有 MPC/EtherCAT 模块和
开发应用；EtherCAT 替换底盘 USB 网关的适配仍属开发工作，不能仅凭分支名宣称完成。

`v0.1` 是 `wbr_infantry` 当前 USB 方案的带注释发布 tag。tag 固定一个源码提交，
后续修复使用新 tag，不移动已发布 tag。MPC 源码保留在当前分支，生产构建默认关闭。
EtherCAT 实现、头文件、应用、工具和测试仅在开发分支保留，正式 USB 分支已移除
相应代码与构建入口。`v0.1` 仍保留发布当时的源码快照，不随本次分支清理变化。

## CI/CD

分支 push 和 PR 执行 USB 正式配置、完整开发配置、无 SDK 配置三组构建/测试。
USB CI 预设开启测试但关闭 MPC 开发模块，直接覆盖正式路径。
推送 `v*` tag 后，全部测试通过才执行 production 构建和打包，并创建 GitHub Release。
发布步骤要求 tag 提交属于 `wbr_infantry` 历史，并有 `docs/releases/<tag>.md` 说明。
Release 包含 Linux x86_64 程序、Foxglove 布局/schema、许可证、构建版本信息及 SHA256。

发布自动化使用 GitHub 仓库自带的临时 GITHUB_TOKEN，不保存个人 token。
CI/CD 发布可下载产物，不自动连接或启动机器人。
