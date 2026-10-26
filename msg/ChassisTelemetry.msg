# 控制样本的 ROS 时间；monotonic_ns 保留原始单调时钟，不能与 ROS 时间直接相减。
std_msgs/Header header
uint32 version 1
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
