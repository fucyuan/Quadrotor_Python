import numpy as np
from utils import collision_check  # 假设 collision_check 函数已经定义
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import QuatToRot, quad_pos  # 假设这些函数已定义
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


def RPYtoRot_ZXY(phi, theta, psi):
    """
    将欧拉角（滚转角、俯仰角、偏航角）转换为从机体坐标系到世界坐标系的旋转矩阵。
    
    参数：
    phi   : 滚转角（绕 x 轴的旋转）
    theta : 俯仰角（绕 y 轴的旋转）
    psi   : 偏航角（绕 z 轴的旋转）
    
    返回值：
    R     : 旋转矩阵，从机体坐标系转换到世界坐标系的 3x3 矩阵。
    """

    # 计算旋转矩阵 R，按照 ZXY 顺序，Z 表示偏航角，X 表示滚转角，Y 表示俯仰角
    R = np.array([
        [np.cos(psi) * np.cos(theta) - np.sin(phi) * np.sin(psi) * np.sin(theta),
         np.cos(theta) * np.sin(psi) + np.cos(psi) * np.sin(phi) * np.sin(theta),
         -np.cos(phi) * np.sin(theta)],
        
        [-np.cos(phi) * np.sin(psi),
         np.cos(phi) * np.cos(psi),
         np.sin(phi)],
        
        [np.cos(psi) * np.sin(theta) + np.cos(theta) * np.sin(phi) * np.sin(psi),
         np.sin(psi) * np.sin(theta) - np.cos(psi) * np.cos(theta) * np.sin(phi),
         np.cos(phi) * np.cos(theta)]
    ])
    
    return R



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




class QuadPlot:
    def __init__(self, qn, state, wingspan, height, color, max_iter, ax=None):
        self.qn = qn
        self.state = state
        self.wingspan = wingspan
        self.color = color
        self.height = height
        self.rot = QuatToRot(self.state[6:10])
        self.motor = quad_pos(self.state[:3], self.rot, self.wingspan, self.height)
        self.text_dist = self.wingspan / 3
        self.des_state = self.state[:6]

        self.max_iter = max_iter
        self.state_hist = np.zeros((6, max_iter))
        self.state_des_hist = np.zeros((6, max_iter))
        self.time_hist = np.zeros(max_iter)
        self.k = 0
        self.time = 0

        if ax is None:
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
        else:
            self.ax = ax

        self.h_m13, = self.ax.plot(self.motor[0, [0, 2]], self.motor[1, [0, 2]], self.motor[2, [0, 2]], '-ko', markerfacecolor=self.color, markersize=5)
        self.h_m24, = self.ax.plot(self.motor[0, [1, 3]], self.motor[1, [1, 3]], self.motor[2, [1, 3]], '-ko', markerfacecolor=self.color, markersize=5)
        self.h_qz, = self.ax.plot(self.motor[0, [4, 5]], self.motor[1, [4, 5]], self.motor[2, [4, 5]], color=self.color, linewidth=2)
        self.h_qn = self.ax.text(self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist, self.motor[2, 4] + self.text_dist, str(qn))
        self.h_pos_hist, = self.ax.plot([self.state[0]], [self.state[1]], [self.state[2]], 'r.')
        self.h_pos_des_hist, = self.ax.plot([self.des_state[0]], [self.des_state[1]], [self.des_state[2]], 'b.')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def update_quad_state(self, state, time):
        self.state = state
        self.time = time
        self.rot = QuatToRot(state[6:10])

    def update_desired_quad_state(self, des_state):
        self.des_state = des_state

    def update_quad_hist(self):
        self.k += 1
        self.time_hist[self.k] = self.time
        self.state_hist[:, self.k] = self.state[:6]
        self.state_des_hist[:, self.k] = self.des_state[:6]

    def update_motor_pos(self):
        self.motor = quad_pos(self.state[:3], self.rot, self.wingspan, self.height)

    def truncate_hist(self):
        self.time_hist = self.time_hist[:self.k]
        self.state_hist = self.state_hist[:, :self.k]
        self.state_des_hist = self.state_des_hist[:, :self.k]

    def update_quad_plot(self, state, des_state, time):
        self.update_quad_state(state, time)
        self.update_desired_quad_state(des_state)
        self.update_quad_hist()
        self.update_motor_pos()

        self.h_m13.set_data(self.motor[0, [0, 2]], self.motor[1, [0, 2]])
        self.h_m13.set_3d_properties(self.motor[2, [0, 2]])
        self.h_m24.set_data(self.motor[0, [1, 3]], self.motor[1, [1, 3]])
        self.h_m24.set_3d_properties(self.motor[2, [1, 3]])
        self.h_qz.set_data(self.motor[0, [4, 5]], self.motor[1, [4, 5]])
        self.h_qz.set_3d_properties(self.motor[2, [4, 5]])
        self.h_qn.set_position((self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist))
        self.h_qn.set_3d_properties(self.motor[2, 4] + self.text_dist)
        
        self.h_pos_hist.set_data(self.state_hist[0, :self.k], self.state_hist[1, :self.k])
        self.h_pos_hist.set_3d_properties(self.state_hist[2, :self.k])
        
        self.h_pos_des_hist.set_data(self.state_des_hist[0, :self.k], self.state_des_hist[1, :self.k])
        self.h_pos_des_hist.set_3d_properties(self.state_des_hist[2, :self.k])

        plt.draw()
