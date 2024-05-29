import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np


def calculate_weights(points):
    """
    计算点云中每个点到质心的距离的倒数作为权重。
    :param points: Nx3的点云数组。
    :return: Nx1的权重数组。
    """
    centroid = np.mean(points, axis=0)
    distances = np.linalg.norm(points - centroid, axis=1)
    weights = 1.0 / (distances + 1e-6)  # 避免除以零
    return weights


def icp(A, B, max_iterations=20, tolerance=1e-6):
    assert A.shape == B.shape
    # 计算权重
    weights = calculate_weights(A)
    prev_error = 0
    iteration_clouds = []

    for i in range(max_iterations):
        centroid_A = np.average(A, axis=0, weights=weights)
        centroid_B = np.average(B, axis=0, weights=weights)
        AA = A - centroid_A
        BB = B - centroid_B

        W = np.diag(weights)  # 创建一个权重矩阵
        H = AA.T @ W @ BB  # 计算加权协方差矩阵

        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T
        t = centroid_B - R @ centroid_A
        A = (R @ A.T).T + t

        iteration_clouds.append(A)  # 记录变换后的源点云
        mean_error = np.average(np.linalg.norm(B - A, axis=1), weights=weights)
        print(prev_error - mean_error)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return iteration_clouds, R, t


def generate_point_cloud(point_cloud, transform=True, rotation=np.eye(3), translation=np.array([300, 300, 50])):
    """
    Transforms a given point cloud by applying a rotation and translation.

    :param point_cloud: The input point cloud as a Nx3 numpy array.
    :param transform: Whether to apply a transformation.
    :param rotation: Rotation matrix for the transformation.
    :param translation: Translation vector for the transformation.
    :return: Transformed point cloud as a Nx3 numpy array.
    """
    # If transformation is needed, apply it
    if transform:
        transformed_point_cloud = np.dot(point_cloud, rotation.T) + translation
    else:
        transformed_point_cloud = point_cloud

    return transformed_point_cloud


def visualize_icp_iterations(source, target, iteration_clouds):
    fig = plt.figure(figsize=(20, 10))

    # 计算子图排列的行数和列数
    rows = 2
    cols = (len(iteration_clouds) + 1) // 2  # 确保所有迭代都能被适当展示

    # 首先绘制初始的源点云和目标点云对比
    ax = fig.add_subplot(rows, cols, 1, projection='3d')
    ax.scatter(source[:, 0], source[:, 1], source[:, 2], color='green', label='Source Initial', alpha=0.5, s=0.1)  # 减小点的大小
    ax.scatter(target[:, 0], target[:, 1], target[:, 2], color='red', label='Target', alpha=0.5, s=0.1)  # 减小点的大小
    ax.set_title('Initial Source and Target')
    ax.legend()

    # 循环遍历每次迭代后的点云
    for i, iter_cloud in enumerate(iteration_clouds):
        ax = fig.add_subplot(rows, cols, i + 2, projection='3d')  # 注意索引 i+2 因为初始对比图占据了第一个位置
        ax.scatter(target[:, 0], target[:, 1], target[:, 2], color='red', label='Target', alpha=0.5, s=0.1)  # 减小点的大小
        ax.scatter(iter_cloud[:, 0], iter_cloud[:, 1], iter_cloud[:, 2], color='blue', label='Source Iteration {}'.format(i+1), alpha=0.5, s=0.1)  # 减小点的大小
        ax.set_title('Iteration {}'.format(i+1))
        ax.legend()

    plt.tight_layout()
    plt.show()


def read_ply(filepath):
    points = []
    in_vertex_section = False
    vertices_expected = 0
    vertices_read = 0

    with open(filepath, 'r') as file:
        for line in file:
            if line.startswith("element vertex"):
                vertices_expected = int(line.split()[2])  # 获取顶点的数量
            elif line.startswith("end_header"):
                in_vertex_section = True
                continue  # 跳过头部结束行

            if in_vertex_section and vertices_read < vertices_expected:
                components = line.strip().split()
                if len(components) >= 5:  # 确保至少包含x, y, z, confidence, intensity数据
                    # 提取x, y, z坐标并将它们转换为浮点数
                    x, y, z = map(float, components[:3])
                    points.append([x, y, z])
                    vertices_read += 1
                if vertices_read >= vertices_expected:
                    break  # 读取完所有顶点后停止

    # 将points列表转换为NumPy数组
    return np.array(points)



# Example usage: Assuming you have a PLY file named 'stanford_bunny.ply'
# bunny_points = read_ply('stanford_bunny.ply')

# Due to the limitations of this environment, we cannot directly read from files.
# You would need to run this code on your local machine where the PLY file is stored.
# After reading the point cloud, you can follow the steps mentioned previously to apply transformations and use the ICP algorithm.

# Generate source and target point clouds
source_point_cloud = read_ply('bun_zipper.ply')

rotation_matrix = np.array([[0.6, -0.8, 0], [0.8, 0.6, 0], [0, 0, 1]])  # Example rotation
translation_vector = np.array([0,  0, 0])  # Example translation
target_point_cloud = generate_point_cloud(source_point_cloud, transform=True, rotation=rotation_matrix, translation=translation_vector)

iteration_clouds, R_icp_visualized, t_icp_visualized = icp(source_point_cloud, target_point_cloud)
visualize_icp_iterations(source_point_cloud, target_point_cloud, iteration_clouds)

# # Output the rotation and translation found by ICP
# R_icp, t_icp

