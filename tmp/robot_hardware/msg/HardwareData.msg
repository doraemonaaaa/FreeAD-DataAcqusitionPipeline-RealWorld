int16 real_left_speed_rpm     # 左轮实际速度
int16 real_right_speed_rpm      # 右轮实际速度
uint8 real_left_neg_flag    # 左轮负向标志    1表示正转，0为反转
uint8 real_right_neg_flag   # 右轮负向标志

int16 expect_left_speed_rpm     # 左轮期望速度
int16 expect_right_speed_rpm    # 右轮期望速度
uint8 expect_left_neg_flag  # 左轮负向标志
uint8 expect_right_neg_flag # 右轮负向标志


# IMU data fields
float64 imu_orientation_x
float64 imu_orientation_y
float64 imu_orientation_z
float64 imu_orientation_w
float64 imu_linear_acceleration_x
float64 imu_linear_acceleration_y
float64 imu_linear_acceleration_z
float64 imu_angular_velocity_x
float64 imu_angular_velocity_y
float64 imu_angular_velocity_z

# Magnetic field data fields
float64 magnetic_field_x
float64 magnetic_field_y
float64 magnetic_field_z