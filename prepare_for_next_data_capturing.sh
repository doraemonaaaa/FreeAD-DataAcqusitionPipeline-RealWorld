#!/bin/bash

# 获取当前时间（年-月-日_小时-分钟-秒）
cur_system_time=$(date +"%Y%m%d_%H%M%S")

# 设置目录路径
capturing_location="AIR_F1"
work_dir="/home/pengyh/documents/ros2_ws/RobotAD/NuscenesData"
save_dir="/home/pengyh/documents/dataset/${capturing_location}/${cur_system_time}"
template_dir="/home/pengyh/documents/ros2_ws/RobotAD/NuscenesData_copy"

# 创建目标保存目录
mkdir -p "$save_dir"

# 将 work_dir 中的文件夹剪切到 save_dir
mv "$work_dir"/* "$save_dir"

# 复制 template_dir 到 save_dir
if [ -d "$template_dir" ]; then
    echo "复制模板目录内容..."
    cp -r "$template_dir"/* "$work_dir"  # 复制内容而非整个文件夹
    echo "模板目录已成功复制到 $work_dir"
else
    echo "错误：找不到模板目录 $template_dir"
    exit 1
fi

echo "操作完成，文件已成功移动并复制到 $save_dir"
