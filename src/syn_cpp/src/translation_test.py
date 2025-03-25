import tf
import numpy as np
init_ext=np.asarray([
        [ 0.00349169, 0.24438457,   0.9696721,   0.05704335],
        [-0.99996054, -0.00706836,  0.00538218,  0.05],
        [ 0.00816931, -0.96965262,  0.24435025, -0.06868874],
        [ 0.,          0.,          0.,          1.        ]
        ])
inv_ext = np.linalg.inv(init_ext)
# print(init_ext)
# print(inv_ext)
rotation = tf.transformations.quaternion_from_matrix(inv_ext)
trans = inv_ext[:3, 3]
# zed2_left_camera_optical_frame -> os_sensor
print("zed2_left_camera_optical_frame -> os_sensor:")
print("t:", trans)
print("q:", rotation)


init_ext = [
        [0.9995169881205799, -0.007426524805013413, -0.0301767656930135],
        [0.014180100376933024, 0.9730379085900248, 0.2302089337972279],
        [0.027653484620760182, -0.2305256497140326, 0.9726732285882337]
    ]
    
# 将旋转矩阵转换为四元数
rotation = tf.transformations.quaternion_from_matrix(
np.vstack([np.hstack([init_ext, [[0], [0], [0]]]), [0, 0, 0, 1]])
)
# 定义 q 数组和其误差
# q = np.array([0.11598044, 0.01455757, -0.00543902, 0.99312992])

# 定义 t 数组和其误差
trans = np.array([0.06528398, -0.2988811, -0.04110306])

# zed2_left_camera_optical_frame -> theta_camera_optical_frame
print("zed2_left_camera_optical_frame -> theta_camera_optical_frame:")
print("t:", trans)
print("q:", rotation)
