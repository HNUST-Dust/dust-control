# YAML 参数第一轮筛选

> 历史记录：保留当时的决策、路径、命令和结果，不作为当前使用说明。
> 当前入口见 [文档索引](../README.md)。

本清单已实施：YAML 参数从 202 项缩减至 71 项。遥控 4 项固定标定已迁入 SbusDecoder，
删除遥控 YAML/schema 及最后的参数头生成流程。底盘配置已迁至 config 并改为启动时加载，
44 项不再生成编译期常量，字段与默认数值保持不变。电机 YAML 同样只在启动时加载，
内建基线直接归电机模块代码，不再保留电机 schema 与参数头生成。下表记录迁移前数量与最终去向，
不包含 schema_version，schema 文件不重复计数。遥控开关阈值保留独立常量。

筛选目标：YAML 保留机器人差异、安装标定、主要控制调参和工作限值；较稳定的算法细节归所属模块 C++ 代码中的私有 constexpr 或现有共享定义。迁回代码不代表失去可调能力，也不集中到新的全局常量表。

稳定参数的实际归属：控制器、估计器、VMC 与输出策略的私有常量直接位于对应 .cpp；
共用物理量、坐标约定、关节控制限值与周期归现有 chassis_config.h。
未新增 *_defaults.hpp。DJI 与 DM 型号规格由电机模块共享型号定义维护。

## 数量

| 文件 | 迁移前 | 保留 YAML | 迁回代码 | 改为推导 | 删除候选 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `config/chassis_params.yaml` | 154 | 44 | 103 | 1 | 6 |
| 遥控（原 remote_input_params.yaml，已删除） | 5 | 0 | 5 | 0 | 0 |
| `src/hardware/imu/hi91_imu_params.yaml` | 1 | 0 | 1 | 0 | 0 |
| `config/motors_params.yaml` | 40 | 25 | 13 | 2 | 0 |
| `config/sensors_params.yaml` | 2 | 2 | 0 | 0 | 0 |
| 合计 | 202 | 71 | 122 | 3 | 6 |

## 实施边界

- 先删除无使用者的参数，再迁移稳定常量，最后收敛推导项；每步保持当前数值和控制输出。
- 当前底盘循环直接使用 1 ms 周期，timing.control_period_ms 并未控制实际调度。保留一个代码周期常量，再由它推导默认 dt，避免伪配置。
- 状态机时间、容差和速率直接放在各控制器现有代码文件中；后续有明确调试需求时再开放具体字段。
- 相同数值不代表相同语义。各状态的最大/最小腿长、阶段目标腿长，以及 stool.joint_torque_max_nm 与 actuator.joint_torque_limit_nm 暂不合并。
- SBUS 开关阈值保留两个独立常量，未将开关输出与摇杆标定强行关联。
- DM 量程属于当前电机型号，迁移时建立型号定义；不能将其当成所有 DM/CubeMars 电机通用的协议常量。本次不增加多型号配置能力。
- 状态机、滤波、观测器参数迁回代码后仍保留原值。尤其不因 acceleration_measurement_noise 很大、部分积分增益为零，就删除相应算法分支。
- sensors freshness 与 SBUS frame_lost 策略不同，不互相推导，也不随本轮删减改变阈值。
- 本清单中的“保留 YAML”是建议保留的上限；如当前只维护单一机器人，可再将静态机构尺寸收进机器人模型。

## config/chassis_params.yaml

| 参数 | 当前值 | 建议 | 理由 |
| --- | --- | --- | --- |
| `mechanics.joint_b_zero_offset_rad` | `0.0` | 保留 YAML | 机构/安装标定 |
| `mechanics.joint_d_zero_offset_rad` | `3.141592653589793` | 保留 YAML | 机构/安装标定 |
| `mechanics.wheel_reduction` | `15.764705882352942` | 保留 YAML | 机构/安装标定 |
| `mechanics.wheel_radius_m` | `0.058` | 保留 YAML | 机构/安装标定 |
| `mechanics.half_wheel_track_m` | `0.215` | 保留 YAML | 机构/安装标定 |
| `mechanics.gravity_mps2` | `9.80665` | 迁回代码 | 固定物理常量 |
| `mechanics.robot_mass_kg` | `12.054` | 保留 YAML | 机构/安装标定 |
| `mechanics.leg_geometry_m.ab` | `0.0945` | 保留 YAML | 机构/安装标定 |
| `mechanics.leg_geometry_m.bc` | `0.1125` | 保留 YAML | 机构/安装标定 |
| `mechanics.leg_geometry_m.cd` | `0.116` | 保留 YAML | 机构/安装标定 |
| `mechanics.leg_geometry_m.ad` | `0.09` | 保留 YAML | 机构/安装标定 |
| `mechanics.leg_geometry_m.ag` | `0.21` | 保留 YAML | 机构/安装标定 |
| `mechanics.leg_geometry_m.gh` | `0.25` | 保留 YAML | 机构/安装标定 |
| `timing.default_dt_s` | `0.001` | 改为推导 | 由底盘运行周期常量换算；不使用未生效的 control_period_ms |
| `timing.control_period_ms` | `1` | 删除候选 | 当前生成符号在 src/tests 中无使用者 |
| `timing.realtime_status_period_ticks` | `100` | 删除候选 | 当前生成符号在 src/tests 中无使用者 |
| `balance.theta_reference_rate_rad_s` | `1.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.pitch_reference_rate_rad_s` | `0.6` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.theta_bias_deg` | `5.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.position_error_limit_m` | `0.15` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.remote_deadband` | `0.05` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.leg_length_command_rate_mps` | `0.24` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.min_leg_length_m` | `0.15` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.max_leg_length_m` | `0.33` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.max_body_speed_mps` | `1.6` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.body_acceleration_limit_mps2` | `0.8` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.max_turn_wheel_speed_mps` | `2.7` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.turn_acceleration_limit_mps2` | `1.5` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.turn_torque_limit_nm` | `1.2` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.yaw_tracking_error_limit_deg` | `45.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.yaw_rate_filter_cutoff_hz` | `20.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.differential_leg_torque_limit_nm` | `4.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.hold_lqr_translation_error_at_zero` | `false` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.roll_reference_rate_deg_s` | `30.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `balance.roll_compensation_kp_n_per_rad` | `80.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.roll_compensation_force_limit_n` | `20.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `balance.roll_compensation_sign` | `1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `stool.target_leg_length_m` | `0.15` | 保留 YAML | 主要调参项或整机工作限值 |
| `stool.joint_velocity_filter_cutoff_hz` | `35.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.ready_theta_tolerance_deg` | `5.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.leg_length_reference_rate_mps` | `1.2` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.swing_start_length_tolerance_m` | `0.05` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.joint_position_kp` | `20.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `stool.joint_position_ki` | `0.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.joint_position_max_speed_rad_s` | `44.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `stool.joint_speed_kp` | `4.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `stool.joint_speed_ki` | `0.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.joint_integral_torque_limit_nm` | `20.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `stool.joint_torque_max_nm` | `54.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.intermediate_leg_length_m` | `0.33` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.retracted_leg_length_m` | `0.15` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.leg_length_rate_mps` | `1.8` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.swing_theta_rad` | `1.45` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.motion1_angle_rate_deg_s` | `300.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.motion2_angle_rate_deg_s` | `270.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.length_tolerance_m` | `0.05` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.motion1_angle_tolerance_deg` | `20.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.motion2_angle_tolerance_deg` | `15.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.ready_hold_ms` | `30` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.motion1_timeout_ms` | `1000` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `climb_stairs.motion2_timeout_ms` | `800` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `gas_spring.left_spring_force_n` | `300.0` | 保留 YAML | 机构/安装标定 |
| `gas_spring.right_spring_force_n` | `300.0` | 保留 YAML | 机构/安装标定 |
| `gas_spring.left_spring_upper_offset_m` | `0.05` | 保留 YAML | 机构/安装标定 |
| `gas_spring.right_spring_upper_offset_m` | `-0.05` | 保留 YAML | 机构/安装标定 |
| `gas_spring.spring_lower_distance_m` | `0.05` | 保留 YAML | 机构/安装标定 |
| `recovery.upright_accel_sign` | `1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `recovery.upright_accel_threshold_g` | `0.4` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.maximum_leg_length_m` | `0.33` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.minimum_leg_length_m` | `0.15` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.swing_theta_rad` | `1.45` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.turnover_sweep_rate_deg_s` | `180.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.swing_angle_rate_deg_s` | `180.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.drawback_angle_rate_deg_s` | `200.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.extend_leg_rate_mps` | `1.2` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.escape_leg_rate_mps` | `1.8` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.drawback_leg_rate_mps` | `1.2` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.turnover_stuck_speed_deg_s` | `35.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.swing_stuck_speed_deg_s` | `15.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.stuck_timeout_ms` | `600` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.upright_hold_ms` | `20` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.ready_hold_ms` | `50` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.turnover_timeout_ms` | `4000` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.swing_timeout_ms` | `2500` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.drawback_timeout_ms` | `1500` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `recovery.maximum_turnover_reversals` | `3` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `support_force_estimator.wheel_mass_kg` | `0.6` | 保留 YAML | 主要调参项或整机工作限值 |
| `support_force_estimator.world_vertical_specific_force_sign` | `1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `support_force_estimator.left_joint_torque_sign` | `1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `support_force_estimator.right_joint_torque_sign` | `1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `support_force_estimator.derivative_filter_cutoff_hz` | `18.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `support_force_estimator.force_filter_cutoff_hz` | `25.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `support_force_estimator.jacobian_damping` | `1e-06` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `support_force_estimator.maximum_abs_force_n` | `300.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `support_force_estimator.feedback_gain` | `0.1` | 删除候选 | 当前生成符号在 src/tests 中无使用者 |
| `support_force_estimator.feedback_correction_limit_n` | `5.0` | 删除候选 | 当前生成符号在 src/tests 中无使用者 |
| `support_force_estimator.feedback_minimum_force_n` | `20.0` | 删除候选 | 当前生成符号在 src/tests 中无使用者 |
| `support_force_estimator.feedback_maximum_force_n` | `200.0` | 删除候选 | 当前生成符号在 src/tests 中无使用者 |
| `flight.liftoff_force_ratio` | `0.2` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.liftoff_hold_ms` | `20` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.minimum_detection_leg_length_m` | `0.17` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.airborne_push_force_n` | `80.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `flight.maximum_leg_length_m` | `0.33` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.maximum_length_margin_m` | `0.01` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.touchdown_retract_speed_mps` | `-0.05` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.touchdown_peak_min_mps` | `0.1` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.touchdown_peak_drop_mps` | `0.15` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.touchdown_length_margin_m` | `0.03` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.touchdown_hold_ms` | `10` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `flight.timeout_ms` | `1500` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.precompress_leg_length_m` | `0.15` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.precompress_length_tolerance_m` | `0.01` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.precompress_speed_tolerance_mps` | `0.2` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.precompress_timeout_ms` | `200` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.thrust_torque_nm` | `30.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `jump.joint_b_thrust_sign` | `1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `jump.joint_d_thrust_sign` | `-1` | 迁回代码 | 当前模型的坐标约定；更换坐标模型时与代码一起审查 |
| `jump.thrust_exit_leg_length_m` | `0.33` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.thrust_timeout_ms` | `300` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.air_retract_leg_length_m` | `0.16` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.air_retract_rate_mps` | `2.5` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.air_retract_gravity_feedforward_n` | `-12.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.air_retract_length_tolerance_m` | `0.01` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.air_retract_hold_ms` | `100` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.air_retract_timeout_ms` | `500` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.cooldown_ms` | `3000` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `jump.total_timeout_ms` | `2000` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `vmc.leg_length_kp` | `600.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `vmc.leg_length_ki` | `100.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `vmc.leg_length_kd` | `120.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `vmc.retract_feedforward_n` | `10.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `vmc.retract_feedforward_deadband_m` | `0.003` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `vmc.extend_velocity_feedforward` | `60.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `vmc.velocity_feedforward_deadband_mps` | `0.005` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `vmc.integral_force_limit_n` | `60.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `vmc.force_limit_n` | `200.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `motion_estimator.process_noise` | `0.1` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `motion_estimator.speed_measurement_noise` | `100.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `motion_estimator.acceleration_measurement_noise` | `1000000000000.0` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `actuator.joint_torque_limit_nm` | `54.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `safety.tilt_cutoff_deg` | `75.0` | 保留 YAML | 主要调参项或整机工作限值 |
| `safety.dm_clear_ticks` | `50` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `safety.dm_arm_ticks` | `300` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `safety.dm_mode_period_ticks` | `10` | 迁回代码 | 所属控制器的算法、滤波或状态机默认值 |
| `hardware.left.kinematic_branch` | `1` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.left.leg_angle_offset_rad` | `-0.036063` | 保留 YAML | 机构/安装标定 |
| `hardware.left.leg_coordinate_sign` | `-1.0` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.left.wheel_feedback_sign` | `1.0` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.left.vmc_torque_sign` | `-1.0` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.right.kinematic_branch` | `-1` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.right.leg_angle_offset_rad` | `-3.12101` | 保留 YAML | 机构/安装标定 |
| `hardware.right.leg_coordinate_sign` | `1.0` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.right.wheel_feedback_sign` | `-1.0` | 迁回代码 | 当前机构模型的分支与坐标约定 |
| `hardware.right.vmc_torque_sign` | `1.0` | 迁回代码 | 当前机构模型的分支与坐标约定 |

## 遥控固定标定（已归 SbusDecoder）

| 参数 | 当前值 | 建议 | 理由 |
| --- | --- | --- | --- |
| `remote_input.wfly_sbus_mid` | `1024.0` | 迁回代码 | 当前遥控器中心与满量程标定 |
| `remote_input.wfly_sbus_scale` | `670.0` | 迁回代码 | 当前遥控器中心与满量程标定 |
| `remote_input.wfly_switch_low_mid_threshold` | `689` | 迁回代码 | 独立开关标定，不依赖摇杆量程 |
| `remote_input.wfly_switch_mid_high_threshold` | `1359` | 迁回代码 | 独立开关标定，不依赖摇杆量程 |
| `remote_input.wfly_frame_lost_timeout_ms` | `1000` | 迁回代码 | SBUS frame_lost 的失能策略；不合并传感器 freshness |

## src/hardware/imu/hi91_imu_params.yaml

| 参数 | 当前值 | 建议 | 理由 |
| --- | --- | --- | --- |
| `hi91_imu.maximum_valid_pitch_deg` | `100.0` | 迁回代码 | Hi91Decoder 有效性检查规则 |

## config/motors_params.yaml

| 参数 | 当前值 | 建议 | 理由 |
| --- | --- | --- | --- |
| `left_wheel.command_sign` | `-1.0` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_wheel.freshness_deadline_us` | `3000` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_wheel.bus` | `3` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_wheel.command_id` | `512` | 迁回代码 | 当前 DJI encoder 固定使用 0x200 命令组 |
| `left_wheel.feedback_id` | `513` | 改为推导 | 现有校验已要求 feedback_id = 0x201 + command_slot |
| `left_wheel.command_slot` | `0` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_wheel.command_sign` | `1.0` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_wheel.freshness_deadline_us` | `3000` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_wheel.bus` | `2` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_wheel.command_id` | `512` | 迁回代码 | 当前 DJI encoder 固定使用 0x200 命令组 |
| `right_wheel.feedback_id` | `513` | 改为推导 | 现有校验已要求 feedback_id = 0x201 + command_slot |
| `right_wheel.command_slot` | `0` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_b.freshness_deadline_us` | `3000` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_b.bus` | `3` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_b.command_id` | `0` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_b.feedback_id` | `16` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_d.freshness_deadline_us` | `3000` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_d.bus` | `3` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_d.command_id` | `3` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `left_joint_d.feedback_id` | `19` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_b.freshness_deadline_us` | `3000` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_b.bus` | `2` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_b.command_id` | `1` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_b.feedback_id` | `17` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_d.freshness_deadline_us` | `3000` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_d.bus` | `2` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_d.command_id` | `2` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `right_joint_d.feedback_id` | `18` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `dji.current_per_nm` | `3450.0` | 保留 YAML | 接线映射、安装方向、型号换算或输入期限 |
| `dji.protocol_current_limit` | `16384` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.position_min_rad` | `-12.56637` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.position_max_rad` | `12.56637` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.velocity_min_rad_s` | `-45.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.velocity_max_rad_s` | `45.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.kp_min` | `0.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.kp_max` | `500.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.kd_min` | `0.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.kd_max` | `5.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.torque_min_nm` | `-54.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |
| `dm_range.torque_max_nm` | `54.0` | 迁回代码 | 当前电机型号的协议规格；由型号定义提供编解码共享值 |

## config/sensors_params.yaml

| 参数 | 当前值 | 建议 | 理由 |
| --- | --- | --- | --- |
| `hi91.freshness_deadline_us` | `3000` | 保留 YAML | 系统输入失效期限 |
| `remote.freshness_deadline_us` | `1000000` | 保留 YAML | 系统输入失效期限 |
