import numpy as np
from utils import RPYtoRot_ZXY, RotToQuat  # 假设这些函数已经在 utils.py 中定义

def init_state(start, yaw):
    """
    初始化四旋翼的 13x1 状态向量。
    
    参数：
    start : 3x1 向量，表示四旋翼的初始位置 [x, y, z]。
    yaw   : 四旋翼的初始偏航角（绕 z 轴的旋转角度）。

    返回值：
    s : 13x1 状态向量，包含位置、速度、四元数和角速度。
        - s[0:3] 为位置 (x, y, z)
        - s[3:6] 为速度 (xdot, ydot, zdot)
        - s[6:10] 为四元数 (qw, qx, qy, qz)
        - s[10:13] 为角速度 (p, q, r)
    """

    # 初始化 13x1 的状态向量 s
    s = np.zeros(13)

    # 初始化欧拉角
    phi0 = 0.0     # 绕 x 轴的滚转角 (roll)
    theta0 = 0.0   # 绕 y 轴的俯仰角 (pitch)
    psi0 = yaw     # 绕 z 轴的偏航角 (yaw)

    # 通过 ZXY 顺序的欧拉角转换为旋转矩阵
    Rot0 = RPYtoRot_ZXY.RPYtoRot_ZXY(phi0, theta0, psi0)

    # 将旋转矩阵转换为四元数
    Quat0 = RotToQuat.RotToQuat(Rot0)

    # 将初始位置填入状态向量
    s[0] = start[0]  # x 位置
    s[1] = start[1]  # y 位置
    s[2] = start[2]  # z 位置

    # 初始速度为 0
    s[3] = 0  # x 方向的速度
    s[4] = 0  # y 方向的速度
    s[5] = 0  # z 方向的速度

    # 填入四元数 (qw, qx, qy, qz)
    s[6] = Quat0[0]  # qw
    s[7] = Quat0[1]  # qx
    s[8] = Quat0[2]  # qy
    s[9] = Quat0[3]  # qz

    # 初始角速度为 0
    s[10] = 0  # 绕 x 轴的角速度 (p)
    s[11] = 0  # 绕 y 轴的角速度 (q)
    s[12] = 0  # 绕 z 轴的角速度 (r)

    return s
