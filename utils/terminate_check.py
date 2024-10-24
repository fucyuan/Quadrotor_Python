import numpy as np
from utils import collision_check  # 假设 collision_check 函数已经定义

def terminate_check(x, time, stop, pos_tol, vel_tol, time_tol):
    """
    检查终止条件，包括位置、速度和时间的容差。
    
    参数：
    x        : 包含每个四旋翼状态的列表，每个状态是一个向量，包含位置和速度。
    time     : 当前仿真时间。
    stop     : 每个四旋翼的目标位置列表。
    pos_tol  : 位置容差，用于判断是否到达目标位置。
    vel_tol  : 速度容差，用于判断是否静止。
    time_tol : 时间限制，用于判断仿真时间是否超时。

    返回值：
    terminate_cond : 一个整数，表示终止条件的状态：
                     1 - 机器人到达目标并静止（成功）。
                     2 - 机器人在给定时间内未到达目标（超时）。
                     3 - 机器人相互碰撞。
                     0 - 仿真继续进行，未达到终止条件。
    """
    
    nquad = len(stop)  # 四旋翼数量

    # 初始化位置和速度检查标志
    pos_check = True
    vel_check = True
    pos_col_check = np.zeros((nquad, 3))  # 用于碰撞检测的坐标矩阵

    # 遍历每个四旋翼，检查位置和速度
    for qn in range(nquad):
        # 计算当前位置与目标位置之间的距离，并与位置容差比较
        pos_check = pos_check and (np.linalg.norm(x[qn][0:3] - stop[qn]) < pos_tol)
        # 计算当前速度的模长，并与速度容差比较
        vel_check = vel_check and (np.linalg.norm(x[qn][3:6]) < vel_tol)
        # 保存当前位置，用于后续碰撞检测
        pos_col_check[qn, :] = x[qn][0:3]

    # 检查仿真时间是否超过给定的时间限制
    time_check = time > time_tol

    # 检查四旋翼是否相互碰撞
    col_check = collision_check(pos_col_check, 0.3)  # 假设 0.3 米是碰撞距离的阈值

    # 根据不同的终止条件返回对应的状态
    if pos_check and vel_check:
        terminate_cond = 1  # 四旋翼到达目标并静止，仿真成功完成
    elif time_check:
        terminate_cond = 2  # 四旋翼在规定时间内未到达目标，仿真超时
    elif col_check:
        terminate_cond = 3  # 四旋翼发生碰撞，仿真终止
    else:
        terminate_cond = 0  # 仿真继续进行

    return terminate_cond
