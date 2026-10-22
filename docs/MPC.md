# MPC 开发模块

MPC 仍是在研控制算法，源码保留在当前 USB 分支；默认生产构建不启用。
使用 `development` 预设，或设置 `DUST_BUILD_MPC=ON`；
`DUST_BUILD_TOOLS=ON` 构建对应开发应用。

`dust::mpc` 保留原腿高/横滚线性 MPC：状态为横滚、横滚角速度、高度、高度速度，
输出左右绝对支撑力，采样 100 Hz，预测 15 步。OSQP v1.0.0 求解完整约束 QP，
保留力上下界、变化率限制、重力前馈和未收敛回退策略。

开发程序：`build/development/apps/leg_mpc_demo`、`usb_mpc`；
性能工具：`build/development/tests/leg_mpc_benchmark`。
QP 回归和 MPC Link 测试随开发预设执行。
独立 SciPy 对照可运行
`python3 tests/mpc/verify_leg_qp.py build/development/tests/leg_mpc_qp_test`，需 numpy/scipy。

`modules/mpc_link` 是原 MPC Link v2 协议库，与 USB CAN gateway 协议分开。
将 MPC 接入当前底盘控制器仍是后续独立工作，应先对照/影子运行再显式切换。
EtherCAT 通讯实现及对应应用仅保留在 `dev/wbr_infantry_ethercat` 分支。
