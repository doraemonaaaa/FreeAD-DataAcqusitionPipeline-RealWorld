# check_path.py
from ament_index_python.packages import get_package_share_directory
print(get_package_share_directory('pointcloud_to_laserscan'))
