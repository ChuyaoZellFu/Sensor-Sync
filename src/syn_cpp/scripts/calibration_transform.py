import numpy as np
from scipy.spatial.transform import Rotation as R

def create_transformation_matrix(rotation_quat, translation_vector):
    """
    创建从旋转四元数和平移向量构造的 4x4 变换矩阵。
    
    :param rotation_quat: 旋转四元数 (numpy array)
    :param translation_vector: 平移向量 (numpy array)
    :return: 4x4 变换矩阵 (numpy array)
    """
    # 使用旋转四元数创建旋转对象
    rotation = R.from_quat(rotation_quat)
    
    # 获取 3x3 的旋转矩阵
    rotation_matrix = rotation.as_matrix()
    
    # 创建 4x4 变换矩阵
    transformation_matrix = np.eye(4)
    
    # 将旋转矩阵填充到左上角 3x3 部分
    transformation_matrix[:3, :3] = rotation_matrix
    
    # 将平移向量填充到最后一列
    transformation_matrix[:3, 3] = translation_vector
    
    return transformation_matrix


### PREVIOUS ###
# Q_lidar_theta= np.array([0.505, -0.488, 0.495, 0.512])  # 旋转四元数
# T_lidar_theta = np.array([-0.012, 0.212, 0.053])  # 平移向量
Q_lidar_theta= np.array([-0.505, 0.488, -0.495, 0.512])  # 旋转四元数
T_lidar_theta = np.array([-0.056, -0.013, 0.212])  # 平移向量

T_lidar_theta = create_transformation_matrix(Q_lidar_theta, T_lidar_theta)

print("\n[Q_lidar_theta] Rotation Quaternion: ", R.from_matrix(T_lidar_theta[:3, :3]).as_quat())
print("[t_lidar_theta] Translation Vector: ", T_lidar_theta[:3, 3])


### CURRENT ###
# 使用已经提取的旋转矩阵 R 和平移向量 t
np.set_printoptions(suppress=True)
R_matrix_cam0_cam1 = np.array([
    [0.99940384, -0.01588918, -0.0306511],
    [0.01972025, 0.99144304, 0.12904187],
    [0.02833845, -0.12956939, 0.99116533]
])

t_vector_cam0_cam1 = np.array([0.11540335, -0.31534779, -0.06535279])

T_theta_zed = np.eye(4)
T_theta_zed[:3, :3] = R_matrix_cam0_cam1
T_theta_zed[:3, 3] = t_vector_cam0_cam1

# Extract rotation quaternion and translation vector from transformation matrix
Q_theta_zed = R.from_matrix(T_theta_zed[:3, :3]).as_quat()
t_theta_zed = T_theta_zed[:3, 3]
print("\n[Q_theta_zed] Rotation Quaternion: ", Q_theta_zed)
print("[t_theta_zed] Translation Vector: ", t_theta_zed)

T_lidar_zed = T_lidar_theta * T_theta_zed
# Extract rotation quaternion and translation vector from transformation matrix
Q_lidar_zed = R.from_matrix(T_lidar_zed[:3, :3]).as_quat()
t_lidar_zed = T_lidar_zed[:3, 3]
print("\n[Q_lidar_zed] Rotation Quaternion: ", Q_lidar_zed)
print("[t_lidar_zed] Translation Vector: ", t_lidar_zed)





