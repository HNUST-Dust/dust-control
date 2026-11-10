# 底盘代码阅读导航

从 [ChassisController::Tick](chassis_controller.cpp) 开始阅读同步控制周期：

```text
设备物理量快照 → 周期输入 → 支撑力估计与离地检测
              → 状态转换与进入处理 → 动作计算 → 输出策略 → 电机目标
```

线程、设备采样、目标提交和观测发布由 [ChassisRuntime](chassis_runtime.cpp) 负责。
控制核心不执行设备 I/O，也不解析 YAML；完整边界见[架构说明](../../docs/ARCHITECTURE.md)。

## 按职责查找

| 职责 | 文件和阅读入口 |
| --- | --- |
| 周期编排 | [chassis_controller.cpp](chassis_controller.cpp)：`Tick`、状态进入处理和动作分派 |
| 输入整理 | [chassis_input_reader.cpp](chassis_input_reader.cpp)：时间有效性、运动学和周期姿态 |
| 转换策略 | [chassis_state_machine.cpp](chassis_state_machine.cpp)：优先级、请求边沿、故障锁存、动作反馈及冷却 |
| 输出策略 | [chassis_output_policy.cpp](chassis_output_policy.cpp)：关节许可、弹簧补偿、最终限幅、握手和电机槽位 |
| 平衡 | [balance_controller.cpp](balance_controller.cpp)：参考、运动估计、LQR 分配和单腿 VMC |
| 回正 | [recovery_controller.cpp](recovery_controller.cpp)：翻身、云台约束、摆腿脱困及收腿 |
| 腾空与跳跃 | [flight_controller.cpp](flight_controller.cpp)、[jump_controller.cpp](jump_controller.cpp)：离地/落地、预压缩、推力和空中收腿 |
| 爬楼 | [climb_stairs_controller.cpp](climb_stairs_controller.cpp)：两阶段参考与关节控制 |
| 运动学与坐标 | [leg_kinematics.cpp](leg_kinematics.cpp)、[leg_joint_coordinates.h](leg_joint_coordinates.h)：FK/IK、雅可比和角度转换 |
| 力矩与弹簧 | [leg_vmc.cpp](leg_vmc.cpp)、[gas_spring.h](gas_spring.h)：轴向力映射、弹簧几何及配置入口 |
| 状态估计 | [body_motion_estimator.cpp](body_motion_estimator.cpp)、[support_force_estimator.cpp](support_force_estimator.cpp) |
| 增益调度 | [unified_lqr_schedule.cpp](unified_lqr_schedule.cpp)、[系数表](unified_lqr_coefficients.inc) |
| 参数 | [chassis_config.h](chassis_config.h) 定义值，[chassis_config.cpp](chassis_config.cpp) 校验，[chassis_config_loading.cpp](chassis_config_loading.cpp) 加载 YAML |
| 运行与观测 | [chassis_runtime.cpp](chassis_runtime.cpp)、[chassis_observation.hpp](chassis_observation.hpp)：线程寿命和观测队列 |
| 十四状态 LQR 研发 | [unified_lqr14.h](unified_lqr14.h)：14×6 跟踪算法、腿长增益调度和[离线设计说明](../../docs/LQR14.md)，尚未切换生产平衡控制 |
| MPC 研发 | [mpc/](mpc/)：可选独立库；不在当前 `Tick` 调用链中 |

输入输出数据定义分别位于 [controller_inputs.h](controller_inputs.h) 和
[chassis_types.h](chassis_types.h)。前者描述设备快照，后者描述周期输入、控制结果及具名左右/B/D 数据。
角度、几何和力矩的坐标不同，修改转换前先看字段注释及架构说明中的坐标边界表。

## 修改时沿哪条线检查

- 修改状态规则：状态机 → 总控进入处理 → 输出策略；动作结果在下一周期触发整机状态转换。
- 修改动作阶段：从对应 `Update` / `UpdatePhase` 开始；回正/爬楼与跳跃的阶段推进和输出时序不同。
- 修改平衡状态：同时核对 `Reset`、`ResetForEnable`、`SynchronizeReferences` 的参考保留规则。
- 修改关节坐标：核对 FK/IK、VMC、弹簧补偿和输入校准；雅可比列顺序是 `[D,B]`，业务数据使用具名 `[B,D]`。
- 修改参数：同步配置值、加载、校验和 [chassis_params.yaml](../../config/chassis_params.yaml)。
  `default_leg_length` 是使能后的平衡默认腿长，对应现有配置键 `stool.target_leg_length_m`；
  `stool` 参数组还提供回正/爬楼使用的关节 PID 参数。

## 构建与验证

[CMakeLists.txt](CMakeLists.txt) 按控制主线、动作和共享算法排列。
`dust_chassis` 包含同步核心；`dust_chassis_runtime` 包含业务运行和 YAML 加载；
`DUST_BUILD_MPC` 决定是否构建独立 MPC 库。

[测试说明](../../docs/TESTING.md) 列出验证范围，关键回归为：

- `chassis.controller_sequence`：总控状态和电机输出。
- `chassis.state_priorities`：竞争条件、反馈时序、锁存和冷却。
- `chassis.action_interfaces`：动作阶段、完成/失败、脱困和输出边界。
- `chassis.balance_sequence`：平衡参考、估计、外部支撑力及复位规则。
- `chassis.coordinate_boundaries`：单位、坐标、虚功关系和补偿/限幅。

这些程序支持 `--trace`，可在结构调整前后对照输出；离线回归不替代实机验收。
