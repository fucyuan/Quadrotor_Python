import numpy as np
# from utils import collision_check  # 假设 collision_check 函数已经定义
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import pdist
# from utils import RPYtoRot_ZXY, RotToQuat  # 假设这些函数已经在 utils.py 中定义


# from utils import QuatToRot, quad_pos  # 假设这些函数已定义
def terminate_check(x, current_time, stop, pos_tol, vel_tol, time_tol):
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
    time_check = current_time> time_tol

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
    norm_q = np.sqrt(np.sum(q ** 2))  # 计算四元数的模

    # 如果模太小，避免归一化，防止除以零的情况
    if norm_q > 1e-6:
        q = q / norm_q
    else:
        q = np.array([1, 0, 0, 0])  # 或者将四元数重置为单位四元数


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




# class QuadPlot:
#     def __init__(self, qn, state, wingspan, height, color, max_iter, ax=None):
#         self.qn = qn
#         self.state = state
#         self.wingspan = wingspan
#         self.color = color
#         self.height = height
#         self.rot = QuatToRot(self.state[6:10])
#         self.motor = quad_pos(self.state[:3], self.rot, self.wingspan, self.height)
#         self.text_dist = self.wingspan / 3
#         self.des_state = self.state[:6]

#         self.max_iter = max_iter
#         self.state_hist = np.zeros((6, max_iter))
#         self.state_des_hist = np.zeros((6, max_iter))
#         self.time_hist = np.zeros(max_iter)
#         self.k = 0
#         self.time = 0

#         if ax is None:
#             self.fig = plt.figure()
#             self.ax = self.fig.add_subplot(111, projection='3d')
#         else:
#             self.ax = ax

#         self.h_m13, = self.ax.plot(self.motor[0, [0, 2]], self.motor[1, [0, 2]], self.motor[2, [0, 2]], '-ko', markerfacecolor=self.color, markersize=5)
#         self.h_m24, = self.ax.plot(self.motor[0, [1, 3]], self.motor[1, [1, 3]], self.motor[2, [1, 3]], '-ko', markerfacecolor=self.color, markersize=5)
#         self.h_qz, = self.ax.plot(self.motor[0, [4, 5]], self.motor[1, [4, 5]], self.motor[2, [4, 5]], color=self.color, linewidth=2)
#         self.h_qn = self.ax.text(self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist, self.motor[2, 4] + self.text_dist, str(qn))
#         self.h_pos_hist, = self.ax.plot([self.state[0]], [self.state[1]], [self.state[2]], 'r.')
#         self.h_pos_des_hist, = self.ax.plot([self.des_state[0]], [self.des_state[1]], [self.des_state[2]], 'b.')

#         self.ax.set_xlabel('X')
#         self.ax.set_ylabel('Y')
#         self.ax.set_zlabel('Z')

#     def update_quad_state(self, state, current_time):
#         self.state = state
#         self.current_time = current_time
#         self.rot = QuatToRot(state[6:10])

#     def update_desired_quad_state(self, des_state):
#         self.des_state = des_state

#     def update_quad_hist(self):
#         self.k += 1
#         self.time_hist[self.k] = self.current_time
#         self.state_hist[:, self.k] = self.state[:6]
#         self.state_des_hist[:, self.k] = self.des_state[:6]

#     def update_motor_pos(self):
#         self.motor = quad_pos(self.state[:3], self.rot, self.wingspan, self.height)

#     def truncate_hist(self):
#         self.time_hist = self.time_hist[:self.k]
#         self.state_hist = self.state_hist[:, :self.k]
#         self.state_des_hist = self.state_des_hist[:, :self.k]

#     def update_quad_plot(self, state, des_state, time):
#         self.update_quad_state(state, time)
#         self.update_desired_quad_state(des_state)
#         self.update_quad_hist()
#         self.update_motor_pos()

#         self.h_m13.set_data(self.motor[0, [0, 2]], self.motor[1, [0, 2]])
#         self.h_m13.set_3d_properties(self.motor[2, [0, 2]])
#         self.h_m24.set_data(self.motor[0, [1, 3]], self.motor[1, [1, 3]])
#         self.h_m24.set_3d_properties(self.motor[2, [1, 3]])
#         self.h_qz.set_data(self.motor[0, [4, 5]], self.motor[1, [4, 5]])
#         self.h_qz.set_3d_properties(self.motor[2, [4, 5]])
#         self.h_qn.set_position((self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist))
#         self.h_qn.set_3d_properties(self.motor[2, 4] + self.text_dist)
        
#         self.h_pos_hist.set_data(self.state_hist[0, :self.k], self.state_hist[1, :self.k])
#         self.h_pos_hist.set_3d_properties(self.state_hist[2, :self.k])
        
#         self.h_pos_des_hist.set_data(self.state_des_hist[0, :self.k], self.state_des_hist[1, :self.k])
#         self.h_pos_des_hist.set_3d_properties(self.state_des_hist[2, :self.k])

#         plt.draw()




class QuadPlot:
    """
    QuadPlot 类用于四旋翼的可视化。
    """

    def __init__(self, qn, state, wingspan, height, color, max_iter, ax=None):
        """
        初始化 QuadPlot 对象。

        参数:
            qn (int): 四旋翼编号。
            state (ndarray): 当前状态向量。
            wingspan (float): 四旋翼的翼展。
            height (float): 四旋翼的高度。
            color (str): 四旋翼的颜色。
            max_iter (int): 最大迭代次数，用于存储历史数据。
            ax (Axes3D): 3D 图形轴，如果未提供，则使用当前的轴。
        """
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

        # 如果没有提供3D坐标轴，使用当前的图形坐标轴
        if ax is None:
            ax = plt.gca(projection='3d')
        self.ax = ax
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')

        # 初始化绘图句柄
        self.h_pos_hist, = ax.plot(self.state[0], self.state[1], self.state[2], 'r.')
        self.h_pos_des_hist, = ax.plot(self.des_state[0], self.des_state[1], self.des_state[2], 'b.')
        self.h_m13, = ax.plot(self.motor[0, [0, 2]], self.motor[1, [0, 2]], self.motor[2, [0, 2]], '-ko',
                              markerfacecolor=self.color, markersize=5)
        self.h_m24, = ax.plot(self.motor[0, [1, 3]], self.motor[1, [1, 3]], self.motor[2, [1, 3]], '-ko',
                              markerfacecolor=self.color, markersize=5)
        self.h_qz, = ax.plot(self.motor[0, [4, 5]], self.motor[1, [4, 5]], self.motor[2, [4, 5]],
                             color=self.color, linewidth=2)
        self.h_qn = ax.text(self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist,
                            self.motor[2, 4] + self.text_dist, str(qn))

    def update_quad_state(self, state, time):
        """
        更新四旋翼的当前状态。

        参数:
            state (ndarray): 当前状态向量。
            time (float): 当前时间。
        """
        self.state = state
        self.time = time
        self.rot = QuatToRot(state[6:10]).T  # 旋转矩阵从机体到世界坐标系

    def update_desired_quad_state(self, des_state):
        """
        更新四旋翼的期望状态。

        参数:
            des_state (ndarray): 期望状态向量。
        """
        self.des_state = des_state

    def update_quad_hist(self):
        """
        更新四旋翼的历史记录（状态和时间）。
        """
        self.k += 1
        self.time_hist[self.k] = self.time
        self.state_hist[:, self.k] = self.state[:6]
        self.state_des_hist[:, self.k] = self.des_state[:6]

    def update_motor_pos(self):
        """
        更新四旋翼的电机位置。
        """
        self.motor = quad_pos(self.state[:3], self.rot, self.wingspan, self.height)

    def truncate_hist(self):
        """
        截断四旋翼的历史记录，只保留有效数据。
        """
        self.time_hist = self.time_hist[:self.k]
        self.state_hist = self.state_hist[:, :self.k]
        self.state_des_hist = self.state_des_hist[:, :self.k]

    # def update_quad_plot(self, state, des_state, time):
        
    #     """
    #     更新四旋翼的可视化图像。

    #     参数:
    #         state (ndarray): 当前状态。
    #         des_state (ndarray): 期望状态。
    #         time (float): 当前时间。
    #     """
    #     self.update_quad_state(state, time)
    #     self.update_desired_quad_state(des_state)
    #     self.update_quad_hist()
    #     self.update_motor_pos()

    #     # 更新电机位置的绘图
    #     self.h_m13.set_data(self.motor[0, [0, 2]], self.motor[1, [0, 2]])
    #     self.h_m13.set_3d_properties(self.motor[2, [0, 2]])

    #     self.h_m24.set_data(self.motor[0, [1, 3]], self.motor[1, [1, 3]])
    #     self.h_m24.set_3d_properties(self.motor[2, [1, 3]])

    #     self.h_qz.set_data(self.motor[0, [4, 5]], self.motor[1, [4, 5]])
    #     self.h_qz.set_3d_properties(self.motor[2, [4, 5]])

    #     # 更新四旋翼编号的文本位置
    #     self.h_qn.set_position((self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist))
    #     self.h_qn.set_3d_properties(self.motor[2, 4] + self.text_dist)

    #     # 更新位置历史记录
    #     self.h_pos_hist.set_data(self.state_hist[0, :self.k], self.state_hist[1, :self.k])
    #     self.h_pos_hist.set_3d_properties(self.state_hist[2, :self.k])

    #     self.h_pos_des_hist.set_data(self.state_des_hist[0, :self.k], self.state_des_hist[1, :self.k])
    #     self.h_pos_des_hist.set_3d_properties(self.state_des_hist[2, :self.k])


    #     plt.draw()  # 刷新图像
        
    def update_quad_plot(self, state, des_state, time):
        """
        更新四旋翼的可视化图像。

        参数:
            state (ndarray): 当前状态。
            des_state (ndarray): 期望状态。
            time (float): 当前时间。
        """
        self.update_quad_state(state, time)
        self.update_desired_quad_state(des_state)
        self.update_quad_hist()
        self.update_motor_pos()

        # # 更新电机位置的绘图
        # self.h_m13.set_data(self.motor[0, [0, 2]], self.motor[1, [0, 2]])
        # self.h_m13.set_3d_properties(self.motor[2, [0, 2]])

        # self.h_m24.set_data(self.motor[0, [1, 3]], self.motor[1, [1, 3]])
        # self.h_m24.set_3d_properties(self.motor[2, [1, 3]])

        # self.h_qz.set_data(self.motor[0, [4, 5]], self.motor[1, [4, 5]])
        # self.h_qz.set_3d_properties(self.motor[2, [4, 5]])

        # # 更新四旋翼编号的文本位置
        # self.h_qn.set_position((self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist))
        # self.h_qn.set_3d_properties(self.motor[2, 4] + self.text_dist)

        # # 更新位置历史记录
        # self.h_pos_hist.set_data(self.state_hist[0, :self.k], self.state_hist[1, :self.k])
        # self.h_pos_hist.set_3d_properties(self.state_hist[2, :self.k])

        # self.h_pos_des_hist.set_data(self.state_des_hist[0, :self.k], self.state_des_hist[1, :self.k])
        # self.h_pos_des_hist.set_3d_properties(self.state_des_hist[2, :self.k])

        # 更新电机位置的绘图
        # self.h_m13 表示电机 1 和 3 之间的连线，它是一个 3D 图形对象
        self.h_m13.set_data(self.motor[0, [0, 2]], self.motor[1, [0, 2]])  # 设置电机 1 和 3 的 X 和 Y 轴数据
        self.h_m13.set_3d_properties(self.motor[2, [0, 2]])  # 设置电机 1 和 3 的 Z 轴数据

        # self.h_m24 表示电机 2 和 4 之间的连线
        self.h_m24.set_data(self.motor[0, [1, 3]], self.motor[1, [1, 3]])  # 设置电机 2 和 4 的 X 和 Y 轴数据
        self.h_m24.set_3d_properties(self.motor[2, [1, 3]])  # 设置电机 2 和 4 的 Z 轴数据

        # self.h_qz 表示四旋翼的 Z 轴连线，可能用于表示方向或者姿态信息
        self.h_qz.set_data(self.motor[0, [4, 5]], self.motor[1, [4, 5]])  # 设置电机 5 和 6 的 X 和 Y 轴数据
        self.h_qz.set_3d_properties(self.motor[2, [4, 5]])  # 设置电机 5 和 6 的 Z 轴数据

        # 更新四旋翼的编号显示
        # self.h_qn 是用于显示四旋翼编号的文本对象
        # text_dist 表示文本对象距离电机的距离，便于清晰显示编号
        self.h_qn.set_position((self.motor[0, 4] + self.text_dist, self.motor[1, 4] + self.text_dist))  # 设置文本的 X 和 Y 轴位置
        self.h_qn.set_text(f"({self.motor[0, 4]:.2f}, {self.motor[1, 4]:.2f}, {self.motor[2, 4]:.2f})")  # 更新文本内容，显示电机 5 的位置数据

        # 更新无人机的历史位置轨迹
        # self.h_pos_hist 是一个 3D 图形对象，表示无人机的实际飞行轨迹
        self.h_pos_hist.set_data(self.state_hist[0, :self.k], self.state_hist[1, :self.k])  # 设置历史位置的 X 和 Y 轴数据
        self.h_pos_hist.set_3d_properties(self.state_hist[2, :self.k])  # 设置历史位置的 Z 轴数据

        # 更新无人机的期望位置轨迹
        # self.h_pos_des_hist 表示无人机的期望飞行轨迹
        self.h_pos_des_hist.set_data(self.state_des_hist[0, :self.k], self.state_des_hist[1, :self.k])  # 设置期望历史位置的 X 和 Y 轴数据
        self.h_pos_des_hist.set_3d_properties(self.state_des_hist[2, :self.k])  # 设置期望历史位置的 Z 轴数据


        # 动态更新XYZ轴的范围，根据状态或历史记录的最大最小值
        x_min, x_max = min(self.state_hist[0, :self.k]), max(self.state_hist[0, :self.k])
        y_min, y_max = min(self.state_hist[1, :self.k]), max(self.state_hist[1, :self.k])
        z_min, z_max = min(self.state_hist[2, :self.k]), max(self.state_hist[2, :self.k])

        # 设置一些边界值，使范围稍微大于实际值，避免图像太过紧凑
        margin = 0.5
        self.ax.set_xlim([x_min - margin, x_max + margin])
        self.ax.set_ylim([y_min - margin, y_max + margin])
        self.ax.set_zlim([z_min - margin, z_max + margin])
        
        plt.draw()  # 刷新图像
        plt.pause(0.001)  # 暂停以更新图像
        




def quadEOM(t, s, qn, controlhandle, trajhandle, params):
    """
    quadEOM: Wrapper function for solving quadrotor equation of motion in Python.

    Inputs:
        t             - float, time
        s             - numpy array of shape (13,), state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
        qn            - int, quad number (used for multi-robot simulations)
        controlhandle - function handle of the controller
        trajhandle    - function handle of the trajectory generator
        params        - dict, parameters needed for the quadrotor, including physical properties

    Outputs:
        sdot          - numpy array of shape (13,), derivative of the state vector s
    """

    # Convert state to quad struct for control
    qd = stateToQd(s)

    # Get desired state
    desired_state = trajhandle(t, qn)

    # Set the desired states (position, velocity, acceleration, yaw, yawdot)
    qd['pos_des'] = desired_state['pos']
    qd['vel_des'] = desired_state['vel']
    qd['acc_des'] = desired_state['acc']
    qd['yaw_des'] = desired_state['yaw']
    qd['yawdot_des'] = desired_state['yawdot']

    # Get control outputs (thrust, moments, trpy, drpy)
    F, M, trpy, drpy = controlhandle(qd, t, qn, params)

    # Compute the derivative of the state vector
    sdot = quadEOM_readonly(t, s, F, M, params)

    return sdot


import numpy as np

def quadEOM_readonly(t, s, F, M, params):
    """
    计算四旋翼飞行器的运动方程，并计算状态向量的导数。

    参数:
        t (float): 当前时间
        s (np.ndarray): 状态向量，大小为 (13,) = [x, y, z, x_dot, y_dot, z_dot, qw, qx, qy, qz, p, q, r]
        F (float): 来自控制器的推力输出
        M (np.ndarray): 来自控制器的力矩输出，大小为 (3,)
        params (dict): 四旋翼的参数，包括物理属性，如质量、惯性矩阵、臂长、最大最小推力等

    返回值:
        np.ndarray: 状态向量 s 的导数，大小为 (13,)
    """

    # ************ 运动方程 ************************
    # 限制推力和力矩，考虑到执行器的限制
    A = np.array([[0.25,                      0, -0.5/params['arm_length']],
                  [0.25,  0.5/params['arm_length'],                      0],
                  [0.25,                      0,  0.5/params['arm_length']],
                  [0.25, -0.5/params['arm_length'],                      0]])

    # 计算每个螺旋桨的推力，忽略 Z 轴的力矩
    prop_thrusts = A @ np.array([F, M[0], M[1]])  
    # 限制每个螺旋桨的推力，确保在最小和最大推力范围内
    prop_thrusts_clamped = np.clip(prop_thrusts, params['minF']/4, params['maxF']/4)

    # 重新计算总推力和力矩
    B = np.array([[                 1,                 1,                 1,                  1],
                  [                 0, params['arm_length'],                 0, -params['arm_length']],
                  [-params['arm_length'],                 0, params['arm_length'],                 0]])

    # 重新计算总推力
    F = B[0, :] @ prop_thrusts_clamped

    # 重新计算力矩
    new_M = np.zeros(3)  # 初始化一个大小为 3 的力矩数组
    new_M[0:2] = B[1:3, :] @ prop_thrusts_clamped  # 计算 M 的前两个分量
    new_M[2] = M[2]  # 保持 M[2] 不变
    M = new_M

    # ************ 状态分配 ************************
    # 将状态向量中的位置、速度、四元数、角速度等分配给不同的变量
    x, y, z = s[0], s[1], s[2]  # 位置 (x, y, z)
    xdot, ydot, zdot = s[3], s[4], s[5]  # 速度 (xdot, ydot, zdot)
    qW, qX, qY, qZ = s[6], s[7], s[8], s[9]  # 四元数表示 (qw, qx, qy, qz)
    p, q, r = s[10], s[11], s[12]  # 角速度 (p, q, r)

    # 当前的四元数表示
    quat = np.array([qW, qX, qY, qZ])
    # 使用四元数将机体坐标系的旋转矩阵转换为世界坐标系
    bRw = QuatToRot(quat)
    # 计算世界坐标系到机体坐标系的旋转矩阵
    wRb = bRw.T

    # ************ 计算加速度 ************************
    # 使用推力和重力计算加速度，推力方向为机体坐标系下的 Z 轴方向
    accel = (1 / params['mass']) * (wRb @ np.array([0, 0, F]) - np.array([0, 0, params['mass'] * params['grav']]))

    # ************ 计算四元数导数 ************************
    K_quat = 2
    quaterror = 1 - (qW**2 + qX**2 + qY**2 + qZ**2)  # 计算模误差，用于纠正四元数
    qdot = -0.5 * np.array([
        [0, -p, -q, -r],
        [p,  0, -r,  q],
        [q,  r,  0, -p],
        [r, -q,  p,  0]
    ]) @ quat + K_quat * quaterror * quat
    # 确保四元数归一化
    norm_quat = np.linalg.norm(quat)
    if norm_quat < 1e-6:
            norm_quat = 1e-6  # 防止零除
    quat = quat / norm_quat  # 归一化四元数


    # ************ 计算角加速度 ************************
    omega = np.array([p, q, r])
    pqrdot = np.linalg.inv(params['I']) @ (M - np.cross(omega, params['I'] @ omega))

    # ************ 组装 sdot 向量 ************************
    sdot = np.zeros(13)
    sdot[0:3] = s[3:6]  # 位置导数，等于当前速度 [xdot, ydot, zdot]
    sdot[3:6] = accel  # 速度导数，等于当前加速度 [accel_x, accel_y, accel_z]
    sdot[6:10] = qdot  # 四元数导数 [qW_dot, qX_dot, qY_dot, qZ_dot]
    sdot[10:13] = pqrdot  # 角速度导数 [p_dot, q_dot, r_dot]

    return sdot






def quatMultiply(q, r):
    """
    Multiplies two quaternions q and r.
    """
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        -x0 * x1 - y0 * y1 - z0 * z1,
         w0 * x1 + y0 * z1 - z0 * y1,
         w0 * y1 + z0 * x1 - x0 * z1,
         w0 * z1 + x0 * y1 - y0 * x1
    ])


# def stateToQd(s):
#     """
#     将状态向量转换为四旋翼的状态字典。

#     参数：
#     s : 状态向量，形状为 (13,)，包含 [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]。

#     返回值：
#     qd : 四旋翼的状态字典，包含位置、速度、姿态和角速度。
#     """
#     qd = {
#         'pos': s[0:3],          # 位置 [x, y, z]
#         'vel': s[3:6],          # 速度 [xd, yd, zd]
#         'quat': s[6:10],        # 四元数 [qw, qx, qy, qz]
#         'omega': s[10:13]       # 角速度 [p, q, r]
#     }
#     return qd

# def stateToQd(s):
#     """
#     将状态向量转换为四旋翼的状态字典。
#     s : 状态向量，形状为 (13,)，包含 [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]。

#     返回值：
#     qd : 四旋翼的状态字典，包含位置、速度、姿态和角速度。
#     """
#     qd = {
#         'pos': s[0:3],          # 位置 [x, y, z]
#         'vel': s[3:6],          # 速度 [xd, yd, zd]
#         'quat': s[6:10],        # 四元数 [qw, qx, qy, qz]
#         'omega': s[10:13],      # 角速度 [p, q, r]
#     }
    
#     # 从四元数计算欧拉角
#     qd['euler'] = quat_to_euler(qd['quat'])  # 使用一个函数将四元数转换为欧拉角
#     return qd

# def quat_to_euler(quat):
#     """
#     将四元数转换为欧拉角 [roll, pitch, yaw]。
#     """
#     qw, qx, qy, qz = quat
#     # 欧拉角计算公式
#     roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
#     pitch = np.arcsin(2 * (qw * qy - qz * qx))
#     yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    
#     return np.array([roll, pitch, yaw])


def stateToQd(x):
    """
    将仿真使用的状态向量转换为硬件中使用的四旋翼状态字典。
    
    参数:
    x : 1x13 的状态向量，包含 [位置, 速度, 四元数, 角速度]
    
    返回:
    qd : 字典，包含四旋翼的状态信息，字段包括：
        - pos: 位置 [x, y, z]
        - vel: 速度 [xd, yd, zd]
        - euler: 欧拉角 [roll, pitch, yaw]
        - omega: 角速度 [p, q, r]
    """
    # 提取位置 [x, y, z]
    qd = {}
    qd['pos'] = x[0:3]
    
    # 提取速度 [xd, yd, zd]
    qd['vel'] = x[3:6]
    
    # 从四元数转换为旋转矩阵
    Rot = quat_to_rot(x[6:10])
    
    # 从旋转矩阵转换为欧拉角 (roll, pitch, yaw)
    phi, theta, yaw = rot_to_rpy_zxy(Rot)
    
    # 设置欧拉角 [roll, pitch, yaw]
    qd['euler'] = np.array([phi, theta, yaw])
    
    # 设置角速度 [p, q, r]
    qd['omega'] = x[10:13]
    
    return qd

def quat_to_rot(q):
    """
    将四元数转换为旋转矩阵。
    
    参数:
    q: 四元数，形状为 (4,)，包含 [qw, qx, qy, qz]
    
    返回:
    R: 3x3 的旋转矩阵
    """
    norm_q = np.linalg.norm(q)

    # 如果模接近零，直接将 q 重置为单位四元数
    if norm_q < 1e-6:  # 阈值可以根据需要调整
        # print("Warning: 四元数的模接近零，重置为单位四元数")
        q = np.array([1, 0, 0, 0])  # 返回单位四元数
    else:
        q = q / norm_q  # 正常归一化

    
    # q = q / np.linalg.norm(q)  # 正则化四元数
    q0, q1, q2, q3 = q
    
    R = np.array([[1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                  [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
                  [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]])
    
    return R

def rot_to_rpy_zxy(R):
    """
    将旋转矩阵转换为欧拉角 (ZXY 旋转顺序)。
    
    参数:
    R: 3x3 的旋转矩阵
    
    返回:
    phi: 绕 x 轴的滚转角度 (roll)
    theta: 绕 y 轴的俯仰角度 (pitch)
    yaw: 绕 z 轴的偏航角度 (yaw)
    """
    # 计算欧拉角
    phi = np.arcsin(R[1, 2])  # roll
    yaw = np.arctan2(-R[1, 0], R[1, 1])  # yaw
    theta = np.arctan2(-R[0, 2], R[2, 2])  # pitch
    
    return phi, theta, yaw






def quad_pos(pos, rot, L, H=0.05):
    """
    计算四旋翼在世界坐标系中的位置坐标。
    
    参数：
    pos : 3x1 的位置向量 [x, y, z]，表示四旋翼的当前位置。
    rot : 3x3 的旋转矩阵，表示从机体坐标系到世界坐标系的旋转。
    L   : 四旋翼臂的长度（机臂的半径，假设四臂长度相同）。
    H   : 高度，表示四旋翼的垂直尺寸，默认为 0.05 米。
    
    返回值：
    quad : 3x6 矩阵，表示四旋翼在世界坐标系中 6 个关键点的坐标：
           - 四个电机的位置
           - 四旋翼中心位置
           - 四旋翼的中心顶部位置
    """

    # 构建 4x4 的齐次变换矩阵（homogeneous transformation matrix），从机体坐标系到世界坐标系
    wHb = np.vstack((np.hstack((rot, pos.reshape(3, 1))), [0, 0, 0, 1]))

    # 定义四旋翼在机体坐标系下的关键点坐标：
    # quadBodyFrame 的每一列分别对应：电机1, 电机2, 电机3, 电机4, 中心位置, 中心顶部位置
    quadBodyFrame = np.array([
        [ L,  0,  0, 1],   # 电机1
        [ 0,  L,  0, 1],   # 电机2
        [-L,  0,  0, 1],   # 电机3
        [ 0, -L,  0, 1],   # 电机4
        [ 0,  0,  0, 1],   # 中心位置
        [ 0,  0,  H, 1]    # 中心顶部位置
    ]).T  # 转置以确保每列表示一个点

    # 将机体坐标系下的点通过齐次变换矩阵转换到世界坐标系下
    quadWorldFrame = np.dot(wHb, quadBodyFrame)

    # 提取转换后的点的 x, y, z 坐标，忽略齐次坐标的最后一行
    quad = quadWorldFrame[:3, :]

    return quad



# def plot_state(h_fig, state, time, name='pos', plot_type='vic', view='sep'):
#     """
#     可视化状态数据，包括位置、速度、欧拉角等信息。

#     参数：
#     h_fig     : 图形句柄，如果为 None，则创建一个新的图形。
#     state     : 状态数据，形状为 (3, N)，其中 N 是时间步长。
#     time      : 时间序列，形状为 (N,)。
#     name      : 状态名称，'pos'（位置）、'vel'（速度）或 'euler'（欧拉角），用于设置标签。
#     plot_type : 状态类型，用于设置线条颜色，'vic'（实际值）、'des'（期望值）或 'est'（估计值）。
#     view      : 图形视图，'sep' 表示分别绘制 3 个子图，'3d' 表示绘制 3D 轨迹。

#     返回值：
#     h_fig : 图形句柄，用于后续绘图操作。
#     """
    
#     # 设置线条颜色根据状态类型
#     if plot_type == 'vic':
#         line_color = 'r'  # 实际值用红色
#     elif plot_type == 'des':
#         line_color = 'b'  # 期望值用蓝色
#     elif plot_type == 'est':
#         line_color = 'g'  # 估计值用绿色

#     # 根据状态名称设置轴标签
#     if name == 'pos':
#         labels = ['x [m]', 'y [m]', 'z [m]']
#     elif name == 'vel':
#         labels = ['xdot [m/s]', 'ydot [m/s]', 'zdot [m/s]']
#     elif name == 'euler':
#         labels = ['roll [rad]', 'pitch [rad]', 'yaw [rad]']

#     # 如果没有提供图形句柄，则创建一个新的图形
#     if h_fig is None:
#         h_fig = plt.figure()

#     # 绘图
#     if view == 'sep':
#         # 分别绘制 3 个子图
#         for i in range(3):
#             ax = h_fig.add_subplot(3, 1, i+1)
#             ax.plot(time, state[i, :], line_color, linewidth=2)
#             ax.set_xlim([time[0], time[-1]])
#             ax.grid(True)
#             ax.set_xlabel('time [s]')
#             ax.set_ylabel(labels[i])
#     elif view == '3d':
#         # 3D 轨迹绘制
#         ax = h_fig.add_subplot(111, projection='3d')
#         ax.plot(state[0, :], state[1, :], state[2, :], line_color, linewidth=2)
#         ax.set_xlabel(labels[0])
#         ax.set_ylabel(labels[1])
#         ax.set_zlabel(labels[2])
#         ax.grid(True)

#     # # 显示图形
#     # plt.show()

#     return h_fig



def plot_state(h_fig, state, time, name='pos', plot_type='vic', view='sep', ax=None, alpha=0.8):
    """
    可视化状态数据，包括位置、速度、欧拉角等信息。

    参数：
    h_fig     : 图形句柄，如果为 None，则创建一个新的图形。
    state     : 状态数据，形状为 (3, N)，其中 N 是时间步长。
    time      : 时间序列，形状为 (N,)。
    name      : 状态名称，'pos'（位置）、'vel'（速度）或 'euler'（欧拉角），用于设置标签。
    plot_type : 状态类型，用于设置线条颜色，'vic'（实际值）、'des'（期望值）或 'est'（估计值）。
    view      : 图形视图，'sep' 表示分别绘制 3 个子图，'3d' 表示绘制 3D 轨迹。
    ax        : Axes对象，用于在指定的轴上绘图。如果为 None，则创建新的轴。
    alpha     : 线条透明度，范围为 0 到 1（可选参数，默认为 0.8）。
    
    返回值：
    h_fig : 图形句柄，用于后续绘图操作。
    """
    
    # 设置线条颜色根据状态类型
    if plot_type == 'vic':
        line_color = 'r'  # 实际值用红色
    elif plot_type == 'des':
        line_color = 'b'  # 期望值用蓝色
    elif plot_type == 'est':
        line_color = 'g'  # 估计值用绿色

    # 根据状态名称设置轴标签
    if name == 'pos':
        labels = ['x [m]', 'y [m]', 'z [m]']
    elif name == 'vel':
        labels = ['xdot [m/s]', 'ydot [m/s]', 'zdot [m/s]']
    elif name == 'euler':
        labels = ['roll [rad]', 'pitch [rad]', 'yaw [rad]']

    # 如果没有提供图形句柄，则创建一个新的图形
    if h_fig is None:
        h_fig = plt.figure()

    # 绘图
    if view == 'sep':
        # 分别绘制 3 个子图
        for i in range(3):
            if ax is None:
                ax = h_fig.add_subplot(3, 1, i+1)  # 如果未提供 Axes，则创建新轴
            ax[i].plot(time, state[i, :], line_color, linewidth=2, alpha=alpha)  # 设置透明度
            ax[i].set_xlim([time[0], time[-1]])
            ax[i].grid(True)
            ax[i].set_xlabel('time [s]')
            ax[i].set_ylabel(labels[i])
    elif view == '3d':
        # 3D 轨迹绘制
        if ax is None:
            ax = h_fig.add_subplot(111, projection='3d')
        ax.plot(state[0, :], state[1, :], state[2, :], line_color, linewidth=2, alpha=alpha)  # 设置透明度
        ax.set_xlabel(labels[0])
        ax.set_ylabel(labels[1])
        ax.set_zlabel(labels[2])
        ax.grid(True)

    return h_fig




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
    Rot0 = RPYtoRot_ZXY(phi0, theta0, psi0)

    # 将旋转矩阵转换为四元数
    Quat0 = RotToQuat(Rot0)

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




def crazyflie():
    """
    返回 Crazyflie 2.0 四旋翼飞行器的物理参数。
    
    参数基于物理测量，包括：
    - 电机、支架和 Vicon 标记的质量点
    - 从中心到质量点的臂长
    - 电池组和主板组合成一个立方体，具有指定的尺寸和质量
    
    返回值：
    params : 一个包含 Crazyflie 参数的字典，包括质量、惯性矩阵、臂长、最大角度、最大推力和最小推力等。
    """

    # 质量（kg），包括 5 个 Vicon 标记（每个约 0.25g）
    m = 0.030

    # 重力加速度（m/s^2）
    g = 9.81

    # 惯性矩阵（kg·m^2），表示四旋翼的惯性张量
    I = np.array([[1.43e-5, 0, 0],    # 绕 x 轴的惯性
                  [0, 1.43e-5, 0],    # 绕 y 轴的惯性
                  [0, 0, 2.89e-5]])   # 绕 z 轴的惯性

    # 臂长（m），从中心到电机的距离
    L = 0.046

    # 创建包含所有参数的字典
    params = {
        'mass': m,                          # 四旋翼的质量
        'I': I,                             # 惯性矩阵
        'invI': np.linalg.inv(I),           # 惯性矩阵的逆矩阵
        'grav': g,                          # 重力加速度
        'arm_length': L,                    # 四旋翼臂长
        'maxangle': 40 * np.pi / 180,       # 最大允许的倾斜角度（弧度）
        'maxF': 2.5 * m * g,                # 最大推力
        'minF': 0.05 * m * g                # 最小推力
    }

    # 返回包含 Crazyflie 参数的字典
    return params



def collision_check(p, margin):
    """
    检查四旋翼是否发生碰撞。通过检查它们之间的距离是否小于给定的碰撞阈值（margin）。
    
    参数：
    p : 2D 数组 (n, 3)，表示 n 个四旋翼的位置，每一行是 [x, y, z] 坐标。
    margin : 碰撞阈值（碰撞半径）。如果两台四旋翼之间的距离小于 2 * margin，则认为发生碰撞。
    
    返回值：
    collide : 布尔值，1 表示发生碰撞，0 表示没有碰撞。
    """

    # 如果只有一个四旋翼，直接返回没有碰撞
    if p.shape[0] <= 1:
        return 0

    # 将 z 轴缩放 1/3，用来模拟椭圆体的碰撞检测（z 轴上距离的影响较小）
    p[:, 2] = p[:, 2] / 3

    # 计算所有四旋翼之间的成对距离
    dis = pdist(p)

    # 如果最小的距离小于 2 倍的 margin，表示发生碰撞
    if np.min(dis) < 2 * margin:
        return 1  # 发生碰撞
    else:
        return 0  # 没有碰撞
