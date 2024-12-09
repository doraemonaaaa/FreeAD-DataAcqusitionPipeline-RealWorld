# ROS2 for RobotAD

DJm3508 P19电机数据：
    uint16_t ecd;            // 编码器值
    int16_t speed_rpm;       // 电机速度
    int16_t given_current;   // 给定电流
    uint8_t temperate;       // 电机温度
    int16_t last_ecd;        // 上一次编码器值
    int64_t sum_ecd;         // 编码器值累计和
ros2换算m/s的speed<----->rpm


雷达数据：
通过设置config.yaml的
send_point_cloud_ros  是否为true表示是否让lidar的数据有ros topic,有了之后启动用topic list即可查询