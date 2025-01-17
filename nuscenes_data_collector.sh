#!/bin/bash

# 参数：选择要运行的版本，默认为 'issac'（即 'nuscenes_data_collector'）
collector_type=${1:-defalut}
# default : real world
# issac :   issac sim

# 输出选择的 collector 类型
echo "Running with collector type: $collector_type"

# 使用单括号进行字符串比较（兼容 sh）
if [ "$collector_type" = "issac" ]; then
    # 如果选择 issac 版本，运行对应的 launch 文件
    ros2 launch /home/pengyh/documents/ros2_ws/RobotAD/src/nuscenes_data_collector/launch/launch.py  data_collector_type:=nuscenes_data_collector_issac_node
else
    # 默认选择 default 版本，运行默认的 launch 文件
    ros2 launch /home/pengyh/documents/ros2_ws/RobotAD/src/nuscenes_data_collector/launch/launch.py  data_collector_type:=nuscenes_data_collector_node
fi
