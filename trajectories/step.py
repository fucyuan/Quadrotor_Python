import numpy as np

def step(t, qn):
    """
    为四旋翼生成一个简单的悬停轨迹。
    
    参数：
    t  : 当前时间。
    qn : 四旋翼的编号（如果有多个四旋翼，这个参数用于区分）。
    
    返回值：
    desired_state : 一个字典，包含目标位置、速度、加速度、偏航角和偏航角速度。
    """

    # 设定时间阈值和移动距离
    time_tol = 5  # 时间阈值（未使用，但可以扩展）
    length = 5    # 移动长度（未使用，但可以扩展）

    # 初始化悬停轨迹的状态变量
    if t <= 0:
        # 如果时间小于或等于 0，四旋翼处于初始位置 [0, 0, 0]
        pos = np.array([0, 0, 0])  # 位置
        vel = np.array([0, 0, 0])  # 速度
        acc = np.array([0, 0, 0])  # 加速度
    else:
        # 在其他时间点，四旋翼悬停在位置 [1, 0, 0]
        pos = np.array([1, 0, 0])  # 悬停位置
        vel = np.array([0, 0, 0])  # 速度为 0
        acc = np.array([0, 0, 0])  # 加速度为 0

    # 偏航角（yaw）和偏航角速度（yawdot），保持为 0
    yaw = 0
    yawdot = 0

    # 将计算的状态存储到字典中
    desired_state = {
        'pos': pos,      # 目标位置
        'vel': vel,      # 目标速度
        'acc': acc,      # 目标加速度
        'yaw': yaw,      # 目标偏航角
        'yawdot': yawdot # 目标偏航角速度
    }

    return desired_state
