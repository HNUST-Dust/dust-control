# 控制样本的 ROS 时间；monotonic_ns 保留原始单调时钟，不能与 ROS 时间直接相减。
std_msgs/Header header
uint32 version 7
uint64 sequence
uint64 monotonic_ns
bool simulated
bool imu_fresh
bool remote_fresh
bool feedback_valid
uint8 motor_stale_mask
uint8 state
uint64 missed_releases
# 底盘观测队列满时的累计丢样，随下一成功入队样本上报。
uint64 queue_drops
# 最近控制样本是否在 200 ms 内；各传感器有效性仍以各自标志为准。
bool source_connected
float64 transport_age_ms
float64 control_duration_us
# 无效或过期数值为 NaN；左右腿独立判定新鲜度。
float64 imu_age_us
float64 pitch_deg
float64 roll_deg
float64 yaw_deg
bool left_leg_fresh
bool right_leg_fresh
float64 left_length_m
float64 right_length_m
float64 left_height_m
float64 right_height_m
float64 left_angle_rad
float64 right_angle_rad
float64 left_length_rate_m_s
float64 right_length_rate_m_s

# 14 状态控制诊断；status 与 LqrStatus 对应，blend 是实际混合接管比例。
uint8 lqr_status
float64 lqr_blend
bool hip_height_valid
float64 hip_height_m
float64 left_hip_height_m
float64 right_hip_height_m
float64 hip_height_rate_m_s
float64 hip_height_reference_m
float64 hip_height_rate_reference_m_s
float64 flat_ground_mismatch_m
float64 left_support_force_n
float64 right_support_force_n
float64 lqr_left_force_target_n
float64 lqr_right_force_target_n
# 轴向限幅后、关节限幅前的模型预测值，不是实际地面反力传感器测量。
float64 lqr_left_force_allocated_n
float64 lqr_right_force_allocated_n

# 平衡计算诊断；失能、recover 或源过期时无效，数值为 NaN。
bool motion_diagnostics_valid
float64 raw_body_speed_mps
float64 estimated_body_speed_mps
float64 target_body_speed_mps
float64 estimated_body_position_m
float64 target_body_position_m
float64 body_position_error_m
float64 body_speed_error_mps
float64 pitch_reference_deg
float64 theta_reference_deg
float64 roll_reference_deg
float64 remote_chassis_x

# 未限幅位置误差及校准腿角；角度与 theta_reference_deg 同定义。
float64 body_position_error_unclamped_m
float64 left_theta_deg
float64 right_theta_deg
# 单位 N*m，数组按14状态顺序，-K*error 的分项，未分配/限幅。
# position,speed,yaw,yaw_rate,left_theta,left_theta_rate,right_theta,
# right_theta_rate,pitch,pitch_rate,roll,roll_rate,height,height_rate
float64[14] lqr_left_wheel_contribution_nm
float64[14] lqr_right_wheel_contribution_nm
float64 lqr_left_wheel_requested_nm
float64 lqr_right_wheel_requested_nm
float64 lqr_left_wheel_allocated_nm
float64 lqr_right_wheel_allocated_nm
# 混合后控制命令，不是电机力矩实测；仅在LQR active时发布。
float64 left_wheel_output_command_nm
float64 right_wheel_output_command_nm

# 执行器诊断。数组顺序：左轮、右轮、左B、左D、右B、右D。
# mode=0 None,1 Torque,2 Disable,3 ClearError,4 Enable；非Torque的命令数值为NaN。
uint8[6] motor_target_mode
float64[6] motor_feedback_age_us
# C620原始电流数值和原始rpm；保留电机协议符号，不直接等同于A或轮端力矩。
float64[2] wheel_current_feedback_raw
float64[2] wheel_speed_feedback_rpm
# 已应用安装方向、3450 count/Nm、±16384限幅及int16截断，与编码值一致。
# 是提交时协议命令的重建值，不是下位机接收确认。
float64[2] wheel_current_command_raw
# 最终安全策略之后、协议编码之前的机体坐标轮力矩命令。
float64[2] wheel_final_command_nm
# 四关节顺序：左B、左D、右B、右D；电机坐标；命令包含弹簧补偿及最终限幅。
float64[4] joint_final_command_nm
float64[4] joint_torque_feedback_nm
float64[4] joint_position_feedback_rad
float64[4] joint_velocity_feedback_rad_s
float64[2] theta_rate_rad_s
float64 pitch_rate_rad_s

# status=9: 同一14状态增益的降级反馈+腿长控制；blend仅表示高度接管比例。
bool lqr_endpoint_gain
bool lqr_height_control_active
