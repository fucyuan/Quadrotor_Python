import numpy as np
# from utils import collision_check  # 假设 collision_check 函数已经定义
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import pdist
# from utils import RPYtoRot_ZXY, RotToQuat  # 假设这些函数已经在 utils.py 中定义
import matplotlib.pyplot as plt

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

    def update_quad_state(self, state, current_time):
        self.state = state
        self.current_time = current_time
        self.rot = QuatToRot(state[6:10])

    def update_desired_quad_state(self, des_state):
        self.des_state = des_state

    def update_quad_hist(self):
        self.k += 1
        self.time_hist[self.k] = self.current_time
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




def quadEOM(t, s, qn, controlhandle, trajhandle, params):
    """
    用于求解四旋翼运动方程的包装函数。
    
    参数：
    t             : 当前时间。
    s             : 状态向量，形状为 (13,)，包含 [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]。
    qn            : 四旋翼编号，用于多机器人仿真。
    controlhandle : 控制器函数的句柄。
    trajhandle    : 轨迹生成器函数的句柄。
    params        : 四旋翼的物理参数。

    返回值：
    sdot          : 状态向量的导数，形状为 (13,)，包含状态的时间导数。
    """
    
    # 将状态向量 s 转换为四旋翼的状态字典，供控制器使用
    qd = stateToQd(s)

    # 获取期望状态
    desired_state = trajhandle(t, qn)

    # 设置期望状态
    qd['pos_des'] = desired_state['pos']        # 期望位置
    qd['vel_des'] = desired_state['vel']        # 期望速度
    qd['acc_des'] = desired_state.get('acc', np.zeros(3))        # 期望加速度，若不存在则为 0
    qd['yaw_des'] = desired_state.get('yaw', 0)        # 期望偏航角，若不存在则为 0
    qd['yawdot_des'] = desired_state.get('yawdot', 0)  # 期望偏航角速度，若不存在则为 0

    # 获取控制输出（推力和力矩）
    F, M, trpy, drpy = controlhandle(qd, t, qn, params)

    # 计算状态向量的导数
    sdot = quadEOM_readonly(t, s, F, M, params)

    return sdot

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
    q = q / np.linalg.norm(q)  # 正则化四元数
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


def quadEOM_readonly(t, s, F, M, params):
    """
    四旋翼的运动方程，计算状态向量的导数。

    参数：
    t      : 当前时间。
    s      : 状态向量，形状为 (13,)，包含 [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]。
    F      : 控制器生成的推力。
    M      : 控制器生成的力矩。
    params : 四旋翼的物理参数。

    返回值：
    sdot   : 状态向量的导数，形状为 (13,)，包含 [xdot, ydot, zdot, ...]。
    """
    # 这里 quadEOM_readonly 仅是占位符，你需要在具体实现中计算四旋翼的运动方程。
    # 例如，使用牛顿-欧拉方程计算位置、速度、角速度的导数。
    sdot = np.zeros(13)  # 占位
    return sdot




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



def plot_state(h_fig, state, time, name='pos', plot_type='vic', view='sep'):
    """
    可视化状态数据，包括位置、速度、欧拉角等信息。

    参数：
    h_fig     : 图形句柄，如果为 None，则创建一个新的图形。
    state     : 状态数据，形状为 (3, N)，其中 N 是时间步长。
    time      : 时间序列，形状为 (N,)。
    name      : 状态名称，'pos'（位置）、'vel'（速度）或 'euler'（欧拉角），用于设置标签。
    plot_type : 状态类型，用于设置线条颜色，'vic'（实际值）、'des'（期望值）或 'est'（估计值）。
    view      : 图形视图，'sep' 表示分别绘制 3 个子图，'3d' 表示绘制 3D 轨迹。

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
            ax = h_fig.add_subplot(3, 1, i+1)
            ax.plot(time, state[i, :], line_color, linewidth=2)
            ax.set_xlim([time[0], time[-1]])
            ax.grid(True)
            ax.set_xlabel('time [s]')
            ax.set_ylabel(labels[i])
    elif view == '3d':
        # 3D 轨迹绘制
        ax = h_fig.add_subplot(111, projection='3d')
        ax.plot(state[0, :], state[1, :], state[2, :], line_color, linewidth=2)
        ax.set_xlabel(labels[0])
        ax.set_ylabel(labels[1])
        ax.set_zlabel(labels[2])
        ax.grid(True)

    # 显示图形
    plt.show()

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
