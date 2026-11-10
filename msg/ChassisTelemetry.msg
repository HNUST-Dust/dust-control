# 控制样本的 ROS 时间；monotonic_ns 保留原始单调时钟，不能与 ROS 时间直接相减。
std_msgs/Header header
uint32 version 3
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

# 14 状态控制诊断；status 与 Lqr14Status 对应，blend 是实际混合接管比例。
uint8 lqr14_status
float64 lqr14_blend
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
float64 lqr14_left_force_target_n
float64 lqr14_right_force_target_n
# 轴向限幅后、关节限幅前的模型预测值，不是实际地面反力传感器测量。
float64 lqr14_left_force_allocated_n
float64 lqr14_right_force_allocated_n

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
