import numpy as np
from scipy.spatial.distance import pdist

def collision_check(p, margin):
    """
    检查四旋翼是否发生碰撞。通过检查它们之间的距离是否小于给定的碰撞阈值（margin）。
    
    参数：
    p : 2D 数组 (n, 3)，表示 n 个四旋翼的位置，每一行是 [x, y, z] 坐标。
    margin : 碰撞阈值（碰撞半径）。如果两台四旋翼之间的距离小于 2 * margin，则认为发生碰撞。
    
    返回值：
    collide : 布尔值，1 表示发生碰撞，0 表示没有碰撞。
    """

    # 如果只有一个四旋翼，直接返回没有碰撞
    if p.shape[0] <= 1:
        return 0

    # 将 z 轴缩放 1/3，用来模拟椭圆体的碰撞检测（z 轴上距离的影响较小）
    p[:, 2] = p[:, 2] / 3

    # 计算所有四旋翼之间的成对距离
    dis = pdist(p)

    # 如果最小的距离小于 2 倍的 margin，表示发生碰撞
    if np.min(dis) < 2 * margin:
        return 1  # 发生碰撞
    else:
        return 0  # 没有碰撞
