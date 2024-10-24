import numpy as np

def quad_pos(pos, rot, L, H=0.05):
    """
    计算四旋翼在世界坐标系中的位置坐标。
    
    参数：
    pos : 3x1 的位置向量 [x, y, z]，表示四旋翼的当前位置。
    rot : 3x3 的旋转矩阵，表示从机体坐标系到世界坐标系的旋转。
    L   : 四旋翼臂的长度（机臂的半径，假设四臂长度相同）。
    H   : 高度，表示四旋翼的垂直尺寸，默认为 0.05 米。
    
    返回值：
    quad : 3x6 矩阵，表示四旋翼在世界坐标系中 6 个关键点的坐标：
           - 四个电机的位置
           - 四旋翼中心位置
           - 四旋翼的中心顶部位置
    """

    # 构建 4x4 的齐次变换矩阵（homogeneous transformation matrix），从机体坐标系到世界坐标系
    wHb = np.vstack((np.hstack((rot, pos.reshape(3, 1))), [0, 0, 0, 1]))

    # 定义四旋翼在机体坐标系下的关键点坐标：
    # quadBodyFrame 的每一列分别对应：电机1, 电机2, 电机3, 电机4, 中心位置, 中心顶部位置
    quadBodyFrame = np.array([
        [ L,  0,  0, 1],   # 电机1
        [ 0,  L,  0, 1],   # 电机2
        [-L,  0,  0, 1],   # 电机3
        [ 0, -L,  0, 1],   # 电机4
        [ 0,  0,  0, 1],   # 中心位置
        [ 0,  0,  H, 1]    # 中心顶部位置
    ]).T  # 转置以确保每列表示一个点

    # 将机体坐标系下的点通过齐次变换矩阵转换到世界坐标系下
    quadWorldFrame = np.dot(wHb, quadBodyFrame)

    # 提取转换后的点的 x, y, z 坐标，忽略齐次坐标的最后一行
    quad = quadWorldFrame[:3, :]

    return quad
