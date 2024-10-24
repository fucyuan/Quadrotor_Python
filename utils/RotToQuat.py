import numpy as np

def RotToQuat(R):
    """
    RotToQuat 将旋转矩阵转换为四元数。
    
    参数：
    R : 3x3 的旋转矩阵。
    
    返回值：
    q : 1x4 的四元数 [qw, qx, qy, qz]，其中 qw 是实部，qx、qy、qz 是虚部。
    """

    # 计算旋转矩阵的迹（即对角线元素之和）
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    # 如果迹大于0，说明旋转角度较小，可以直接从迹计算
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    # 如果 R[0, 0] 最大，计算 qx
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4 * qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    # 如果 R[1, 1] 最大，计算 qy
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4 * qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    # 否则，R[2, 2] 最大，计算 qz
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4 * qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    # 返回归一化后的四元数，并保证 qw 为正
    q = np.array([qw, qx, qy, qz])
    q = q * np.sign(qw)

    return q
