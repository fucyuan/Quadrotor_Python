import numpy as np

def controller(qd, t, qn, params):
    """
    四旋翼的控制器函数。
    
    参数：
    qd     : 四旋翼的当前和期望状态，包含位置、速度、欧拉角、角速度等信息。
    t      : 当前时间（在此实现中未使用）。
    qn     : 四旋翼的编号（在此实现中未使用）。
    params : 包含四旋翼物理参数的字典，如质量、惯性矩阵、重力加速度等。
    
    返回值：
    F      : 生成的总推力。
    M      : 生成的控制力矩（即 x, y, z 方向的力矩）。
    trpy   : 输出的期望推力和欧拉角，用于硬件。
    drpy   : 输出的期望欧拉角速度，用于硬件。
    """

    # 位置控制器参数
    Kp = np.array([15, 15, 30])   # 位置控制的比例增益
    Kd = np.array([12, 12, 10])   # 速度控制的微分增益

    # 姿态控制器参数
    KpM = np.array([3000, 3000, 3000])  # 姿态控制的比例增益
    KdM = np.array([300, 300, 300])     # 角速度控制的微分增益

    # 计算期望加速度
    acc_des = qd['acc_des'] + Kd * (qd['vel_des'] - qd['vel']) + Kp * (qd['pos_des'] - qd['pos'])

    # 计算期望的滚转角 (phi) 和俯仰角 (theta)
    phi_des = 1 / params['grav'] * (acc_des[0] * np.sin(qd['yaw_des']) - acc_des[1] * np.cos(qd['yaw_des']))
    theta_des = 1 / params['grav'] * (acc_des[0] * np.cos(qd['yaw_des']) + acc_des[1] * np.sin(qd['yaw_des']))
    psi_des = qd['yaw_des']  # 期望偏航角 (yaw)

    # 期望的欧拉角和角速度
    euler_des = np.array([phi_des, theta_des, psi_des])
    pqr_des = np.array([0, 0, qd['yawdot_des']])

    # 计算推力 F
    F = params['mass'] * (params['grav'] + acc_des[2])

    # 计算力矩 M（根据角速度误差和欧拉角误差）
    M = np.dot(params['I'], KdM * (pqr_des - qd['omega']) + KpM * (euler_des - qd['euler']))

    # 输出期望推力和欧拉角
    trpy = [F, phi_des, theta_des, psi_des]
    
    # 输出期望欧拉角速度（此处设为 0）
    drpy = [0, 0, 0, 0]

    return F, M, trpy, drpy
