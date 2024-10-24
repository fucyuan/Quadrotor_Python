import numpy as np

def circle(t, qn):
    """
    为四旋翼生成圆形轨迹。
    
    参数：
    t  : 当前时间。
    qn : 四旋翼的编号（用于区分多个四旋翼，但在此简单实现中未使用）。
    
    返回值：
    desired_state : 一个字典，包含目标位置、速度、加速度、偏航角和偏航角速度。
    """

    # 时间和圆的参数
    time_tol = 12   # 轨迹总时间
    radius = 5      # 圆的半径
    dt = 0.0001     # 时间步长，用于数值计算速度和加速度

    # 根据给定角度计算位置
    def pos_from_angle(a):
        """
        根据角度 a 计算位置。轨迹是一个圆，z 轴随着角度线性增加。
        """
        return np.array([radius * np.cos(a), radius * np.sin(a), 2.5 * a / (2 * np.pi)])

    # 根据当前时间计算角度变化
    def tj_from_line(start, end, total_time, t):
        """
        线性插值计算角度变化。
        """
        return start + (end - start) * t / total_time

    # 计算速度
    def get_vel(t):
        """
        根据时间 t 计算速度，使用数值微分。
        """
        angle1 = tj_from_line(0, 2 * np.pi, time_tol, t)
        pos1 = pos_from_angle(angle1)
        angle2 = tj_from_line(0, 2 * np.pi, time_tol, t + dt)
        vel = (pos_from_angle(angle2) - pos1) / dt
        return vel

    # 计算加速度
    def get_acc(t):
        """
        根据时间 t 计算加速度，使用数值微分。
        """
        vel1 = get_vel(t)
        vel2 = get_vel(t + dt)
        acc = (vel2 - vel1) / dt
        return acc

    # 根据时间 t 生成轨迹
    if t > time_tol:
        # 当时间超过 time_tol 时，四旋翼固定在圆的末端位置
        pos = np.array([radius, 0, 2.5])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    else:
        # 在轨迹时间范围内，计算当前位置、速度和加速度
        angle = tj_from_line(0, 2 * np.pi, time_tol, t)
        pos = pos_from_angle(angle)
        vel = get_vel(t)
        acc = get_acc(t)

    # 偏航角和偏航角速度
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
