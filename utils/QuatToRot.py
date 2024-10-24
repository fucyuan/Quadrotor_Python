import numpy as np

def QuatToRot(q):
    """
    QuatToRot 将四元数转换为旋转矩阵。
    
    参数：
    q : 1x4 的四元数，格式为 [qw, qx, qy, qz]，其中 qw 是实部，qx、qy、qz 是虚部。
    
    返回值：
    R : 3x3 的旋转矩阵，表示从四元数转换而来的方向。
    """

    # 归一化四元数 q，确保其模长为 1，避免误差积累
    q = q / np.sqrt(np.sum(q ** 2))

    # 构造反对称矩阵 qahat (3x3)
    qahat = np.zeros((3, 3))  # 初始化为 3x3 的零矩阵

    # 填入反对称矩阵的值，按照四元数的虚部计算
    qahat[0, 1] = -q[3]   # -qz
    qahat[0, 2] = q[2]    # qy
    qahat[1, 2] = -q[1]   # -qx
    qahat[1, 0] = q[3]    # qz
    qahat[2, 0] = -q[2]   # -qy
    qahat[2, 1] = q[1]    # qx

    # 计算旋转矩阵 R，公式为 R = I + 2*qahat*qahat + 2*qw*qahat
    R = np.eye(3) + 2 * np.dot(qahat, qahat) + 2 * q[0] * qahat

    return R
