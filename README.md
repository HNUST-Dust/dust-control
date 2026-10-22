# dust-control — wbr_infantry EtherCAT development

当前分支：`dev/wbr_infantry_ethercat`，用于 wbr_infantry 机器人的 EtherCAT 下位机通讯方案开发。
当前正式 USB 版本请使用 `wbr_infantry` 分支或 `v0.1` tag。
分支及 CI/CD 约定见 [分支说明](docs/BRANCHES.md)。

## 开发状态

保留完整底盘控制、原生 Foxglove、MPC、MPC Link、EtherCAT 协议/安全策略，
以及 IgH、SOEM、mock 后端。当前 EtherCAT 应用来自既有开发实现，
将其适配为正式底盘 USB 网关的替代链路仍待开发与硬件验证。
本分支不会仅因构建成功就自动替换实际控制算法或通讯路径。

## 构建与测试

```bash
sudo apt-get install build-essential cmake ninja-build pkg-config libeigen3-dev libusb-1.0-0-dev python3-yaml
./scripts/build.sh
./scripts/test.sh
```

本分支两个脚本默认使用 `ethercat-development` 预设，启用 MPC、EtherCAT、开发应用和自动测试。
首次构建获取固定版本 OSQP 和 Foxglove SDK，CMake >=3.21、C++20。
IgH/SOEM 依赖在可用时启用，没有硬件库时仍能构建 mock 并验证协议/安全逻辑。

产物位于 `build/ethercat-development/`：

- `apps/ethercat_bench`：通讯开发与性能研究入口。
- `apps/ethercat_mpc`：已有 EtherCAT/MPC 开发应用。
- `apps/chassis_controller`：保留的 USB 底盘参考入口；还没有完成 EtherCAT 适配。
- `tests/`：模块回归。

正式 USB production 配置仍可显式构建：`./scripts/build.sh production`。
`scripts/run.sh` 仍是 USB 参考应用的启动脚本，不会自动选择 EtherCAT。
任何真实硬件运行均需现场人员按对应应用的参数、网卡及设备协议显式操作。

## 参考

- [模块结构](docs/ARCHITECTURE.md)
- [MPC / EtherCAT 开发边界](docs/MPC_AND_ETHERCAT.md)
- [测试](docs/TESTING.md)
- [Foxglove](tools/foxglove/README.md)

CI 对所有分支/PR 执行 USB、完整开发、无 SDK 三组回归。
此分支不能通过 USB 正式发布的 tag 来源校验；正式版本从 `wbr_infantry` 发布。
