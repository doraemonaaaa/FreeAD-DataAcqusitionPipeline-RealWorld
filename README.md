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

lidar have 4 dimension data (x, y, z, intensity)

# start up

-- install tmux
sudo apt install ros-humble-twist-mux

-- install ros2_control

-- insta robot_localization
sudo apt install ros-humble-robot-localization

-- install nav2
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-dwa-local-planner
sudo apt install ros-humble-amcl
sudo apt install ros-humble-map-server
sudo apt install ros-humble-slam-toolbox

-- install pointcloude to laserscan
sudo apt install ros-humble-pointcloud-to-laserscan

-- use rosdep to install all dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

-- set up the lidar ETH connection
https://www.ncnynl.com/archives/202209/5470.html

-- set up Real-Time Publish-Subscribe, Fast RTPS, used for RT communication
    https://blog.csdn.net/m0_73800387/article/details/144001925

    option 1: shared memory
    sudo apt-get install ros-humble-rmw-fastrtps-cpp
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export RMW_FASTRTPS_USE_SHM=1
    export RMW_FASTRTPS_SHM_MAX_SIZE=100000000  # 设置共享内存的最大大小为 100MB
    export RMW_FASTRTPS_SHM_ID="ros2_shared_memory"  # 设置共享内存的ID
    ros2 doctor --report # this can check which RMW MIDDLEWARE we use

    option 2: network communication, will slow down when subscriber more
    sudo apt-get install ros-humble-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 doctor --report # this can check which RMW MIDDLEWARE we use

-- hardware setup
    1. plugin stm32 USB
    2. plugin imu USB
    3. start up the whole baterry which have lidar and cameras

# build
-- build robot_hardware packages at first
colcon build --packages-select robot_hardware --symlink-install

-- then build others
colcon build --symlink-install

-- set the env PATH
source install/setup.bash
