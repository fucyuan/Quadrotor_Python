import numpy as np

def diamond(t, qn):
    """
    生成四旋翼沿菱形轨迹的期望状态。
    
    参数：
    t  : 当前时间。
    qn : 四旋翼编号（在此实现中未使用）。
    
    返回值：
    desired_state : 包含目标位置、速度、加速度、偏航角和偏航角速度的字典。
    """

    # 轨迹总时间
    time_tol = 12
    dt = 0.0001  # 时间步长，用于数值计算速度和加速度

    # 辅助函数：通过线性插值计算位置和速度
    def tj_from_line(start, end, total_time, t):
        """
        线性插值生成轨迹，从起始位置 `start` 移动到终止位置 `end`，在 `total_time` 内。
        返回当前位置、速度和加速度（加速度在此未使用）。
        """
        pos = start + (end - start) * t / total_time
        vel = (end - start) / total_time
        return pos, vel, np.zeros(3)  # 返回位置、速度和加速度（加速度为零）

    # 生成给定时间下的位置和速度
    def get_pos_vel(t):
        """
        根据时间 t 计算当前位置和速度。轨迹为菱形路径。
        """
        if t >= time_tol:
            # 如果时间超过总时间，位置固定在终点
            pos = np.array([1, 0, 0])
            vel = np.array([0, 0, 0])
        elif 0 <= t < time_tol / 4:
            # 从 [0,0,0] 到 [1/4, sqrt(2), sqrt(2)] 的第一段
            pos, vel, _ = tj_from_line(np.array([0, 0, 0]), np.array([1/4, np.sqrt(2), np.sqrt(2)]), time_tol / 4, t)
        elif time_tol / 4 <= t < time_tol / 2:
            # 从 [1/4, sqrt(2), sqrt(2)] 到 [1/2, 0, 2*sqrt(2)] 的第二段
            pos, vel, _ = tj_from_line(np.array([1/4, np.sqrt(2), np.sqrt(2)]), np.array([1/2, 0, 2 * np.sqrt(2)]), time_tol / 4, t - time_tol / 4)
        elif time_tol / 2 <= t < 3 * time_tol / 4:
            # 从 [1/2, 0, 2*sqrt(2)] 到 [3/4, -sqrt(2), sqrt(2)] 的第三段
            pos, vel, _ = tj_from_line(np.array([1/2, 0, 2 * np.sqrt(2)]), np.array([3/4, -np.sqrt(2), np.sqrt(2)]), time_tol / 4, t - time_tol / 2)
        else:
            # 从 [3/4, -sqrt(2), sqrt(2)] 到 [1, 0, 0] 的第四段
            pos, vel, _ = tj_from_line(np.array([3/4, -np.sqrt(2), np.sqrt(2)]), np.array([1, 0, 0]), time_tol / 4, t - 3 * time_tol / 4)
        return pos, vel

    # 计算位置、速度和加速度
    if t >= time_tol:
        pos = np.array([1, 0, 0])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    else:
        # 计算当前位置和速度
        pos, vel = get_pos_vel(t)
        # 使用数值微分计算加速度
        _, vel1 = get_pos_vel(t + dt)
        acc = (vel1 - vel) / dt

    # 偏航角和偏航角速度（保持为 0）
    yaw = 0
    yawdot = 0

    # 返回期望状态
    desired_state = {
        'pos': pos,      # 目标位置
        'vel': vel,      # 目标速度
        'acc': acc,      # 目标加速度
        'yaw': yaw,      # 目标偏航角
        'yawdot': yawdot # 目标偏航角速度
    }

    return desired_state
