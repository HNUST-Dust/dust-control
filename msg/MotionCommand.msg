# Motion targets in base_link. Empty frame_id also means base_link.
# Freshness uses local monotonic receipt time; ROS stamps do not renew safety.
std_msgs/Header header
float64 forward_velocity_m_s
float64 yaw_rate_rad_s
float64 leg_length_rate_m_s
