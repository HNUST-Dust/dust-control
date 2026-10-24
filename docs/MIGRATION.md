# 从 wbr_control_companion 迁移

来源：`NoneOfEver/wbr_control_companion` 的当前生产清理基线及本地 Foxglove 工作树。
精确原提交和每个导入文件的原始 SHA256 见 `IMPORT_MANIFEST.json`；旧 MCU 导入来源
见 `MCU_SOURCE_MANIFEST.json`。源仓库、未提交修改、硬件结果和 NUC 均未被改写。
目标仓库保留已有 Apache-2.0 LICENSE，并在其初始 main 提交上前进。

| 旧位置 | 新位置 |
|---|---|
| `chassis_controller/control` | `src/chassis` |
| `chassis_controller/runtime` | `src/main.cpp`、`src/hardware/usb`、`src/telemetry` |
| `chassis_controller/include` | 按职责归入 `src/hardware/motors、src/hardware/remote、src/hardware/imu、src/hardware/referee`、`src/chassis`、`src/hardware/usb` 等目录，与实现相邻 |
| `chassis_controller/config` | 遥控固定标定和其他稳定默认值归使用模块代码；底盘、电机、传感器启动配置在 `config` |
| `chassis_controller/telemetry` | `tools/foxglove` |
| `src/leg_mpc.cpp` | `src/chassis/mpc` |
| `src/mpc_link.cpp` | 旧 MPC Link 协议已移除 |
| `src/ethercat_*` | 仅在 `dev/wbr_infantry_ethercat` 保留 |
| `src/realtime_loop.cpp` | `src/chassis/runtime` |
| 两套测试目录 | `tests/chassis`、`tests/mpc`、`tests/protocols`；共享统计测试在 `tests/runtime` |
| 两个独立顶层 CMake | 单一根 CMake + 模块构建文件 |

原底盘脚本改为 `scripts/build.sh` / `scripts/run.sh`。
当前 USB 分支开发构建覆盖底盘与在研 MPC；EtherCAT 专用代码在独立开发分支。
原基准工具保持单独构建，已更新共享 realtime 源码路径。

本次重构不更改控制增益、符号、零点、输入新鲜度阈值、状态机、输出门控或线协议。
不包括重新刷 MCU、启动真实电机输出或重复实物标定。旧 NUC 验证结论仅对应旧版本，
本次结构重构还需后续在目标硬件确认时序。
