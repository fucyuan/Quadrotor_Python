import numpy as np

def RotToRPY_ZXY(R):
    """
    从世界到机体的旋转矩阵中提取 ZXY 顺序的欧拉角（滚转角、俯仰角、偏航角）。
    
    参数：
    R : 3x3 的旋转矩阵，表示从世界坐标系到机体坐标系的旋转。

    返回值：
    phi   : 滚转角 (绕 x 轴的旋转)
    theta : 俯仰角 (绕 y 轴的旋转)
    psi   : 偏航角 (绕 z 轴的旋转)
    """

    # 计算滚转角 phi（绕 x 轴的旋转角度），通过 R(2,3) 提取
    phi = np.arcsin(R[1, 2])

    # 计算偏航角 psi（绕 z 轴的旋转角度），通过 atan2 提取
    # 注意我们需要用 cos(phi) 来消除滚转角的影响
    psi = np.arctan2(-R[1, 0] / np.cos(phi), R[1, 1] / np.cos(phi))

    # 计算俯仰角 theta（绕 y 轴的旋转角度），同样使用 atan2 来计算
    theta = np.arctan2(-R[0, 2] / np.cos(phi), R[2, 2] / np.cos(phi))

    return phi, theta, psi
