import numpy as np

# def diamond(t, qn):
#     """
#     生成四旋翼沿菱形轨迹的期望状态。
    
#     参数：
#     t  : 当前时间。
#     qn : 四旋翼编号（在此实现中未使用）。
    
#     返回值：
#     desired_state : 包含目标位置、速度、加速度、偏航角和偏航角速度的字典。
#     """

#     # 轨迹总时间
#     time_tol = 12
#     dt = 0.0001  # 时间步长，用于数值计算速度和加速度

#     # 辅助函数：通过线性插值计算位置和速度
#     def tj_from_line(start, end, total_time, t):
#         """
#         线性插值生成轨迹，从起始位置 `start` 移动到终止位置 `end`，在 `total_time` 内。
#         返回当前位置、速度和加速度（加速度在此未使用）。
#         """
#         pos = start + (end - start) * t / total_time
#         vel = (end - start) / total_time
#         return pos, vel, np.zeros(3)  # 返回位置、速度和加速度（加速度为零）

#     # 生成给定时间下的位置和速度
#     def get_pos_vel(t):
#         """
#         根据时间 t 计算当前位置和速度。轨迹为菱形路径。
#         """
#         if t >= time_tol:
#             # 如果时间超过总时间，位置固定在终点
#             pos = np.array([1, 0, 0])
#             vel = np.array([0, 0, 0])
#         elif 0 <= t < time_tol / 4:
#             # 从 [0,0,0] 到 [1/4, sqrt(2), sqrt(2)] 的第一段
#             pos, vel, _ = tj_from_line(np.array([0, 0, 0]), np.array([1/4, np.sqrt(2), np.sqrt(2)]), time_tol / 4, t)
#         elif time_tol / 4 <= t < time_tol / 2:
#             # 从 [1/4, sqrt(2), sqrt(2)] 到 [1/2, 0, 2*sqrt(2)] 的第二段
#             pos, vel, _ = tj_from_line(np.array([1/4, np.sqrt(2), np.sqrt(2)]), np.array([1/2, 0, 2 * np.sqrt(2)]), time_tol / 4, t - time_tol / 4)
#         elif time_tol / 2 <= t < 3 * time_tol / 4:
#             # 从 [1/2, 0, 2*sqrt(2)] 到 [3/4, -sqrt(2), sqrt(2)] 的第三段
#             pos, vel, _ = tj_from_line(np.array([1/2, 0, 2 * np.sqrt(2)]), np.array([3/4, -np.sqrt(2), np.sqrt(2)]), time_tol / 4, t - time_tol / 2)
#         else:
#             # 从 [3/4, -sqrt(2), sqrt(2)] 到 [1, 0, 0] 的第四段
#             pos, vel, _ = tj_from_line(np.array([3/4, -np.sqrt(2), np.sqrt(2)]), np.array([1, 0, 0]), time_tol / 4, t - 3 * time_tol / 4)
#         return pos, vel

#     # 计算位置、速度和加速度
#     if t >= time_tol:
#         pos = np.array([1, 0, 0])
#         vel = np.array([0, 0, 0])
#         acc = np.array([0, 0, 0])
#     else:
#         # 计算当前位置和速度
#         pos, vel = get_pos_vel(t)
#         # 使用数值微分计算加速度
#         _, vel1 = get_pos_vel(t + dt)
#         acc = (vel1 - vel) / dt

#     # 偏航角和偏航角速度（保持为 0）
#     yaw = 0
#     yawdot = 0

#     # 返回期望状态
#     desired_state = {
#         'pos': pos,      # 目标位置
#         'vel': vel,      # 目标速度
#         'acc': acc,      # 目标加速度
#         'yaw': yaw,      # 目标偏航角
#         'yawdot': yawdot # 目标偏航角速度
#     }

#     return desired_state



# def circle(t, qn):
#     """
#     为四旋翼生成圆形轨迹。
    
#     参数：
#     t  : 当前时间。
#     qn : 四旋翼的编号（用于区分多个四旋翼，但在此简单实现中未使用）。
    
#     返回值：
#     desired_state : 一个字典，包含目标位置、速度、加速度、偏航角和偏航角速度。
#     """

#     # 时间和圆的参数
#     time_tol = 12   # 轨迹总时间
#     radius = 5      # 圆的半径
#     dt = 0.0001     # 时间步长，用于数值计算速度和加速度

#     # 根据给定角度计算位置
#     def pos_from_angle(a):
#         """
#         根据角度 a 计算位置。轨迹是一个圆，z 轴随着角度线性增加。
#         """
#         return np.array([radius * np.cos(a), radius * np.sin(a), 2.5 * a / (2 * np.pi)])

#     # 根据当前时间计算角度变化
#     def tj_from_line(start, end, total_time, t):
#         """
#         线性插值计算角度变化。
#         """
#         return start + (end - start) * t / total_time

#     # 计算速度
#     def get_vel(t):
#         """
#         根据时间 t 计算速度，使用数值微分。
#         """
#         angle1 = tj_from_line(0, 2 * np.pi, time_tol, t)
#         pos1 = pos_from_angle(angle1)
#         angle2 = tj_from_line(0, 2 * np.pi, time_tol, t + dt)
#         vel = (pos_from_angle(angle2) - pos1) / dt
#         return vel

#     # 计算加速度
#     def get_acc(t):
#         """
#         根据时间 t 计算加速度，使用数值微分。
#         """
#         vel1 = get_vel(t)
#         vel2 = get_vel(t + dt)
#         acc = (vel2 - vel1) / dt
#         return acc

#     # 根据时间 t 生成轨迹
#     if t > time_tol:
#         # 当时间超过 time_tol 时，四旋翼固定在圆的末端位置
#         pos = np.array([radius, 0, 2.5])
#         vel = np.array([0, 0, 0])
#         acc = np.array([0, 0, 0])
#     else:
#         # 在轨迹时间范围内，计算当前位置、速度和加速度
#         angle = tj_from_line(0, 2 * np.pi, time_tol, t)
#         pos = pos_from_angle(angle)
#         vel = get_vel(t)
#         acc = get_acc(t)

#     # 偏航角和偏航角速度
#     yaw = 0
#     yawdot = 0

#     # 将计算的状态存储到字典中
#     desired_state = {
#         'pos': pos,      # 目标位置
#         'vel': vel,      # 目标速度
#         'acc': acc,      # 目标加速度
#         'yaw': yaw,      # 目标偏航角
#         'yawdot': yawdot # 目标偏航角速度
#     }

#     return desired_state


# def step(t, qn):
#     """
#     为四旋翼生成一个简单的悬停轨迹。
    
#     参数：
#     t  : 当前时间。
#     qn : 四旋翼的编号（如果有多个四旋翼，这个参数用于区分）。
    
#     返回值：
#     desired_state : 一个字典，包含目标位置、速度、加速度、偏航角和偏航角速度。
#     """

#     # 设定时间阈值和移动距离
#     time_tol = 5  # 时间阈值（未使用，但可以扩展）
#     length = 5    # 移动长度（未使用，但可以扩展）

#     # 初始化悬停轨迹的状态变量
#     if t <= 0:
#         # 如果时间小于或等于 0，四旋翼处于初始位置 [0, 0, 0]
#         pos = np.array([0, 0, 0])  # 位置
#         vel = np.array([0, 0, 0])  # 速度
#         acc = np.array([0, 0, 0])  # 加速度
#     else:
#         # 在其他时间点，四旋翼悬停在位置 [1, 0, 0]
#         pos = np.array([1, 0, 0])  # 悬停位置
#         vel = np.array([0, 0, 0])  # 速度为 0
#         acc = np.array([0, 0, 0])  # 加速度为 0

#     # 偏航角（yaw）和偏航角速度（yawdot），保持为 0
#     yaw = 0
#     yawdot = 0

#     # 将计算的状态存储到字典中
#     desired_state = {
#         'pos': pos,      # 目标位置
#         'vel': vel,      # 目标速度
#         'acc': acc,      # 目标加速度
#         'yaw': yaw,      # 目标偏航角
#         'yawdot': yawdot # 目标偏航角速度
#     }

#     return desired_state



import numpy as np

def diamond(t, qn):
    """
    生成钻石形轨迹的期望状态。
    
    参数:
        t (float): 当前时间
        qn (int): 无人机编号
    
    返回:
        desired_state (dict): 期望状态，包括位置、速度、加速度、偏航角和偏航角速度
    """
    
    time_tol = 12
    dt = 0.0001

    def get_pos_vel(t):
        if t >= time_tol:
            pos = np.array([1, 0, 0])
            vel = np.array([0, 0, 0])
        elif 0 <= t < time_tol / 4:
            # 从 [0, 0, 0] 到 [1/4, sqrt(2), sqrt(2)]
            pos, vel, _ = tj_from_line(np.array([0, 0, 0]), np.array([1/4, np.sqrt(2), np.sqrt(2)]), time_tol / 4, t)
        elif time_tol / 4 <= t < time_tol / 2:
            # 从 [1/4, sqrt(2), sqrt(2)] 到 [1/2, 0, 2*sqrt(2)]
            pos, vel, _ = tj_from_line(np.array([1/4, np.sqrt(2), np.sqrt(2)]), np.array([1/2, 0, 2*np.sqrt(2)]), time_tol / 4, t - time_tol / 4)
        elif time_tol / 2 <= t < time_tol * 3 / 4:
            # 从 [1/2, 0, 2*sqrt(2)] 到 [3/4, -sqrt(2), sqrt(2)]
            pos, vel, _ = tj_from_line(np.array([1/2, 0, 2*np.sqrt(2)]), np.array([3/4, -np.sqrt(2), np.sqrt(2)]), time_tol / 4, t - time_tol / 2)
        else:
            # 从 [3/4, -sqrt(2), sqrt(2)] 到 [1, 0, 0]
            pos, vel, _ = tj_from_line(np.array([3/4, -np.sqrt(2), np.sqrt(2)]), np.array([1, 0, 0]), time_tol / 4, t - time_tol * 3 / 4)
        
        return pos, vel

    if t >= time_tol:
        pos = np.array([1, 0, 0])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    else:
        pos, vel = get_pos_vel(t)
        _, vel1 = get_pos_vel(t + dt)
        acc = (vel1 - vel) / dt
    
    yaw = 0
    yawdot = 0

    # 返回期望状态
    desired_state = {
        'pos': pos,
        'vel': vel,
        'acc': acc,
        'yaw': yaw,
        'yawdot': yawdot
    }

    return desired_state

def tj_from_line(start_pos, end_pos, time_ttl, t_c):
    """
    生成从 start_pos 到 end_pos 的直线轨迹，计算位置和速度。

    参数:
        start_pos (ndarray): 起始位置
        end_pos (ndarray): 终止位置
        time_ttl (float): 轨迹总时间
        t_c (float): 当前时间
    
    返回:
        pos (ndarray): 当前的位置
        vel (ndarray): 当前的速度
        acc (ndarray): 当前的加速度（固定为 0）
    """
    
    v_max = (end_pos - start_pos) * 2 / time_ttl
    
    if 0 <= t_c < time_ttl / 2:
        vel = v_max * t_c / (time_ttl / 2)
        pos = start_pos + t_c * vel / 2
        acc = np.array([0, 0, 0])
    else:
        vel = v_max * (time_ttl - t_c) / (time_ttl / 2)
        pos = end_pos - (time_ttl - t_c) * vel / 2
        acc = np.array([0, 0, 0])
    
    return pos, vel, acc


import numpy as np

def circle(t, qn):
    """
    生成圆形轨迹的期望状态。
    
    参数:
        t (float): 当前时间
        qn (int): 无人机编号
    
    返回:
        desired_state (dict): 期望状态，包括位置、速度、加速度、偏航角和偏航角速度
    """
    
    time_tol = 12
    radius = 5
    dt = 0.0001


    def pos_from_angle(a):
        """
        根据角度 a 计算圆形轨迹的三维位置。

        参数:
            a (float): 当前角度

        返回:
            ndarray: 位置 [x, y, z]
        """
        return np.array([radius * np.cos(a), radius * np.sin(a), 2.5 * a / (2 * np.pi)])

    def get_vel(t):
        """
        根据时间 t 计算圆形轨迹的速度。

        参数:
            t (float): 当前时间

        返回:
            ndarray: 速度 [vx, vy, vz]xiuxii
        """
        angle1, _, _ = tj_from_line(0, 2 * np.pi, time_tol, t)
        pos1 = pos_from_angle(angle1)
        angle2, _, _ = tj_from_line(0, 2 * np.pi, time_tol, t + dt)
        return (pos_from_angle(angle2) - pos1) / dt

    if t > time_tol:
        pos = np.array([radius, 0, 2.5])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    else:
        angle, _, _ = tj_from_line(0, 2 * np.pi, time_tol, t)
        pos = pos_from_angle(angle)
        vel = get_vel(t)
        acc = (get_vel(t + dt) - get_vel(t)) / dt

    yaw = 0
    yawdot = 0

    # 返回期望状态
    desired_state = {
        'pos': pos,
        'vel': vel,
        'acc': acc,
        'yaw': yaw,
        'yawdot': yawdot
    }

    return desired_state


def eight_shape(t, qn, time_tol=24, radius=5):
    """
    生成连续且可跟随的8字形轨迹的期望状态。
    
    参数:
        t (float): 当前时间
        qn (int): 无人机编号
        time_tol (float): 轨迹的总时间，增大该值可以减缓运动速度
        radius (float): 轨迹的半径大小
    
    返回:
        desired_state (dict): 期望状态，包括位置、速度、加速度、偏航角和偏航角速度
    """
    
    dt = 0.001  # 时间步长
    
    # 计算归一化的当前时间（让时间在 0 到 2π 之间）
    t_normalized = (t % time_tol) / time_tol * 2 * np.pi
    
    # 使用参数方程生成8字形轨迹
    def pos_from_angle(a):
        """
        根据角度 a 计算8字形轨迹的三维位置。
        
        参数:
            a (float): 当前的参数角度
        
        返回:
            ndarray: 位置 [x, y, z]
        """
        x = radius * np.sin(a)
        y = radius * np.sin(2 * a) / 2  # 使用 sin(2a) 生成8字形
        z = 2.5 * (a / (2 * np.pi))  # 线性上升的z坐标
        return np.array([x, y, z])
    
    # 计算速度
    def get_vel(t):
        """
        根据时间 t 计算8字形轨迹的速度。
        
        参数:
            t (float): 当前时间
        
        返回:
            ndarray: 速度 [vx, vy, vz]
        """
        angle1 = t_normalized
        pos1 = pos_from_angle(angle1)
        angle2 = t_normalized + dt * 2 * np.pi / time_tol
        pos2 = pos_from_angle(angle2)
        return (pos2 - pos1) / dt
    
    # 计算加速度
    def get_acc(t):
        """
        根据时间 t 计算8字形轨迹的加速度。
        
        参数:
            t (float): 当前时间
        
        返回:
            ndarray: 加速度 [ax, ay, az]
        """
        vel1 = get_vel(t)
        vel2 = get_vel(t + dt)
        return (vel2 - vel1) / dt
    
    # 限制最大速度
    max_speed = 1.5  # 最大速度，根据无人机的能力设置
    vel = get_vel(t)
    speed = np.linalg.norm(vel)
    if speed > max_speed:
        vel = vel / speed * max_speed  # 将速度限制在最大值范围内
    
    # 计算当前的期望位置、速度和加速度
    pos = pos_from_angle(t_normalized)
    acc = get_acc(t)
    
    yaw = 0
    yawdot = 0

    # 返回期望状态
    desired_state = {
        'pos': pos,
        'vel': vel,
        'acc': acc,
        'yaw': yaw,
        'yawdot': yawdot
    }

    return desired_state
