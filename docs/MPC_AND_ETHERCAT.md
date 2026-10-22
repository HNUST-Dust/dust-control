# MPC 与 EtherCAT 开发模块

两个模块均为继续开发的正式方向，默认关闭构建以保持当前 USB 底盘路径稳定。
使用 `development` 预设一次性启用，也可分别设置 `DUST_BUILD_MPC=ON`、
`DUST_BUILD_ETHERCAT=ON`。`DUST_BUILD_TOOLS=ON` 构建对应独立应用。

## MPC

`dust::mpc` 保留现有腿高/横滚线性 MPC：状态为横滚、横滚角速度、高度、高度速度，
输出左右绝对支撑力，采样 100 Hz，预测 15 步。OSQP v1.0.0 求解完整约束 QP，
保留力上下界、变化率限制、重力前馈和未收敛回退策略。当前参数与调参验证边界
沿用原工程，不能由模块化重构推断已完成实机控制验证。

开发程序：`build/development/apps/leg_mpc_demo`、`usb_mpc`；
性能工具：`build/development/tests/leg_mpc_benchmark`。
QP 数学回归、协议与安全测试随开发预设执行。
独立 SciPy 对照可运行
`python3 tests/mpc/verify_leg_qp.py build/development/tests/leg_mpc_qp_test`，需 numpy/scipy。

`modules/mpc_link` 仍是原 MPC Link v2 的单独协议库，不与 USB CAN gateway 协议合并。
MPC 新算法接入当前底盘控制器属于后续独立工作，应先对照/影子运行再显式切换。

## EtherCAT

`dust::ethercat` 保留原协议、主机安全策略、mock、IgH 和 SOEM 后端。
IgH/SOEM 在依赖可用时启用；当前构建是否检测到后端见 CMake 输出。
没有对应硬件库时可构建 mock 并进行协议/安全测试，不代表硬件链路已验证。

开发程序：`build/development/apps/ethercat_bench`；同时开启 MPC 时还会构建
`ethercat_mpc`。原 Active 双门控、序号及故障失能策略保持不变。
`--help` 可查询各程序参数，开发/测试不会自动启动硬件应用。

脚本 `scripts/run_ethercat_matrix.sh` 可通过 `BENCHMARK_BIN` 指定基准程序路径。
内核驱动选择脚本和硬件 commissioning 仍需现场人员按实际网卡与驱动条件执行。
