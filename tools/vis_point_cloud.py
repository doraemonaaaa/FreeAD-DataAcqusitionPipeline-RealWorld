import numpy as np
import open3d as o3d

def visualize_nuscenes_bin(file_path):
    # 从 .bin 文件读取点云数据
    points = np.fromfile(file_path, dtype=np.float32)
    
    # 将数据 reshape 为 (N, 4)，即每点 4 个 float32: x, y, z, intensity
    points = points.reshape(-1, 4)
    
    # 提取坐标 (x, y, z)
    xyz = points[:, :3]
    
    # 提取强度 (intensity)，并归一化到 0-1
    intensity = points[:, 3]
    intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())
    
    # 创建点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    
    # 将强度映射为点云的颜色
    colors = np.zeros((xyz.shape[0], 3))  # 创建颜色矩阵
    colors[:, 0] = intensity_normalized  # 将强度映射为红色通道
    colors[:, 2] = 1 - intensity_normalized  # 将反强度映射为蓝色通道
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 可视化点云
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # 替换为你的文件路径
    file_path = "/home/pengyh/documents/ros2_ws/RobotAD/tools/2025-01-19 17:47:57__LIDAR_TOP__1737280078_49187422.bin"
    visualize_nuscenes_bin(file_path)
