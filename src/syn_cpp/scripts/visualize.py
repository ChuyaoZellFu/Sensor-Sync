import open3d as o3d
import numpy as np

# 加载点云数据
point_cloud_data = np.load('/home/hjyeee/Data/Dataset/rpf_dataset/2024-07-04-21-12-25/ouster_pcs/1716454402018465598.npy')

# 检查点云数据的形状（确保它是N x 3或N x 6的数组）
print(f"Point cloud shape: {point_cloud_data.shape}")

# 如果点云数据只有点的坐标 (N x 3)
if point_cloud_data.shape[1] == 3:
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_cloud_data)

# 如果点云数据包含颜色信息 (N x 6)，前3列是坐标，后3列是颜色
elif point_cloud_data.shape[1] == 6:
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_cloud_data[:, :3])
    point_cloud.colors = o3d.utility.Vector3dVector(point_cloud_data[:, 3:6] / 255.0)  # 假设颜色值在0-255之间

# 可视化点云
o3d.visualization.draw_geometries([point_cloud])
