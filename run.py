import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.integrate import odeint
from scipy.integrate import solve_ivp  # solve_ivp 是 ode45 的等效函数
import time
from utils import QuadPlot, init_state, crazyflie, terminate_check, plot_state, quadEOM,calculate_euler_angles
from trajectories import  circle,diamond,eight_shape,step
from controller import controller
# from utils.crazyflie import crazyflie
# 你可以在这里更改轨迹
# trajhandle = step
# trajhandle = circle
# trajhandle = diamond  # 使用 diamond 轨迹
trajhandle = eight_shape

# 控制器
controlhandle = controller

# 实时运行
real_time = True

# *********** 你不需要修改以下任何内容 **********
# 四旋翼数量(只能是1)
nquad = 1

# 最大时间
time_tol = 25  # 仿真最大时间限制

# 仿真参数
params = crazyflie()  # 调用模块内的 crazyflie 参数

# **************************** 图像设置 *****************************
print('初始化图像...')
fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')  # 3D 绘图
ax.set_box_aspect([0.1, 0.1, 100])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.grid(True)
# 设置图像窗口大小，使其与视频分辨率匹配 (640x480)
fig.set_size_inches(6.4, 4.8)  # 以英寸为单位设置窗口大小，确保宽高比是 640x480
fig.set_dpi(100)  # 设置每英寸像素点，确保图像是 640x480 像素

# 使用 'hsv' 色彩映射，nquad 是四旋翼数量
quadcolors = plt.get_cmap('hsv', nquad)

# *********************** 初始条件设置 ***********************
print('设置初始条件...')
max_iter = 5000  # 最大迭代次数
starttime = 0  # 仿真开始时间（秒）
tstep = 0.01  # 时间步长
cstep = 0.05  # 图像捕获时间间隔
nstep = int(cstep / tstep)  # 每个时间间隔步数
current_time = starttime  # 当前仿真时间
err = None

# 初始化每个四旋翼的状态
xtraj = []
ttraj = []
# 每个列表将用于动态存储该无人机的 [roll, pitch, yaw] 数据
desired_Euler = [[] for _ in range(nquad)]
x0 = []
stop = []
for qn in range(nquad):
    des_start = trajhandle(0, qn)  # 轨迹起始位置
    des_stop = trajhandle(np.inf, qn)  # 轨迹结束位置
    stop.append(des_stop['pos'])  # 停止位置
    x0.append(init_state(des_start['pos'], 0))  # 初始状态
    xtraj.append(np.zeros((max_iter * nstep, len(x0[qn]))))  # 初始化轨迹存储
    ttraj.append(np.zeros(max_iter * nstep))  # 初始化时间存储



x = x0  # 当前状态
pos_tol = 0.01  # 位置容差
vel_tol = 0.01  # 速度容差
print(f"x[{qn}] = {x[qn]}")

# ************************* 运行仿真 *************************
OUTPUT_TO_VIDEO = 1
if OUTPUT_TO_VIDEO == 1:
    import cv2
    video_filename = 'diamond.avi'
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 20.0, (640, 480))

print('仿真运行中....')
for iter in range(max_iter):
    timeint = np.arange(current_time, current_time + cstep + tstep, tstep)
    # print(timeint)
    tic = time.time()
    
    # print(tic)
    # 遍历每个四旋翼
    for qn in range(nquad):
    
        # 初始化四旋翼图像
        if iter == 0:  # 注意这里需要修正为 iter == 0，第一次绘图
            QP = QuadPlot(qn, x0[qn], 0.1, 0.04, quadcolors(qn), max_iter, ax)
            desired_state = trajhandle(current_time, qn)
            euler_angles = calculate_euler_angles(desired_state)

            # 将欧拉角 [roll, pitch, yaw] 以列表形式存储到对应无人机的欧拉角列表中
            desired_Euler[qn].append([euler_angles['roll'], euler_angles['pitch'], euler_angles['yaw']])
            QP.update_quad_plot(x[qn], np.hstack([desired_state['pos'], desired_state['vel']]), current_time)
        # 使用 odeint 进行仿真
        def quad_eom(s, t, qn, controlhandle, trajhandle, params):
            return quadEOM(t, s, qn, controlhandle, trajhandle, params)
    
        
        # 仿真四旋翼动力学
        # xsave = odeint(quadEOM, x[qn], timeint, args=(qn, controlhandle, trajhandle, params), atol=1e-9, rtol=1e-6)
        xsave = odeint(quad_eom, x[qn], timeint, args=(qn, controlhandle, trajhandle, params), atol=1e-5, rtol=1e-3)
        x[qn] = xsave[-1, :]  # 更新状态

        if xtraj[qn] is None:
            xtraj[qn] = np.zeros((nstep * (iter + 1), xsave.shape[1]))  # 初始化
        if ttraj[qn] is None:
            ttraj[qn] = np.zeros(nstep * (iter + 1))  # 初始化

        # 计算索引，iter 从 0 开始
        start_idx = iter * nstep
        end_idx = (iter + 1) * nstep
        xtraj[qn][start_idx:end_idx+1, :] = xsave[0:6, :]  # 保留 xsave 的前 n-1 行
        ttraj[qn][start_idx:end_idx+1] = timeint[0:6]  # 保留 tsave 的前 n-1 行
        

        # 更新四旋翼图像
        desired_state = trajhandle(current_time + cstep, qn)
        # 计算期望的欧拉角，假设 calculate_euler_angles 函数基于每次迭代的状态
        euler_angles = calculate_euler_angles(desired_state)

        # 将欧拉角 [roll, pitch, yaw] 以列表形式存储到对应无人机的欧拉角列表中
        desired_Euler[qn].append([euler_angles['roll'], euler_angles['pitch'], euler_angles['yaw']])
        QP.update_quad_plot(x[qn], np.hstack([desired_state['pos'], desired_state['vel']]), current_time + cstep)
        
        # if OUTPUT_TO_VIDEO == 1:
        #     fig.canvas.draw()  # 更新绘图
        #     frame = np.array(fig.canvas.buffer_rgba())  # 获取图像数据
        #     out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))  # 保存为视频
        if OUTPUT_TO_VIDEO == 1:
             # 获取当前图像的宽度和高度 
            fig.canvas.draw()  # 更新绘图 
            width, height = fig.canvas.get_width_height()

            # 从绘图缓冲区中提取图像数据，去掉 reshape 操作
            frame = np.array(fig.canvas.buffer_rgba())  # 获取图像数据
           
            # 使用 OpenCV 强制调整图像大小为 640x480
            frame_resized = cv2.resize(frame, (640, 480))  # 强制调整图像大小到 640x480

            # 转换图像为 BGR 格式并写入视频
            out.write(cv2.cvtColor(frame_resized, cv2.COLOR_RGB2BGR))  # 写入视频帧




    
    current_time += cstep  # 更新仿真时间
    elapsed = time.time() - tic  # 计算仿真步长耗时
    # print(elapsed)

    # 检查 odeint 是否超时
    if elapsed > cstep * 50:
        err = 'odeint 不稳定'
        break

    # 实时暂停
    if real_time and (elapsed < cstep):
        time.sleep(cstep - elapsed)

    # 检查是否满足终止条件
    if terminate_check(x, current_time, stop, pos_tol, vel_tol, time_tol):
        break
for qn in range(nquad):
    desired_Euler[qn] = np.array(desired_Euler[qn])
    print(desired_Euler[qn][:, 0])
    print(desired_Euler[qn][:, 1])
    print(desired_Euler[qn][:, 2])
if OUTPUT_TO_VIDEO == 1:
    out.release()  # 关闭视频文件

# ************************* 后处理 *************************
# 截取 xtraj 和 ttraj
for qn in range(nquad):
    xtraj[qn] = xtraj[qn][:iter * nstep, :]  # 截取轨迹数据
    ttraj[qn] = ttraj[qn][:iter * nstep]  # 截取时间数据
    # 假设有 nquad 个无人机

    # plt.tight_layout()  # 调整布局，避免重叠
    # plt.show()


# 遍历每个无人机
for qn in range(nquad):
    # 取出当前无人机的 x, y, z 轨迹（前3列）
    x = xtraj[qn][:, 0]  # extract x-axis position
    y = xtraj[qn][:, 1]  # extract y-axis position
    z = xtraj[qn][:, 2]  # extract z-axis position

    # 取出时间数据
    t = ttraj[qn]

    # 创建新的子图来绘制
    plt.figure(figsize=(10, 6))

    # 绘制 x, y, z 分别随时间变化的图
    plt.subplot(3, 1, 1)
    plt.plot(t, x, label=f"Drone {qn} - X")
    plt.ylabel("X Position (m)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, y, label=f"Drone {qn} - Y")
    plt.ylabel("Y Position (m)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, z, label=f"Drone {qn} - Z")
    plt.ylabel("Z Position (m)")
    plt.xlabel("Time (s)")
    plt.legend()


# # 遍历每个无人机
# for qn in range(nquad):
#     # 取出当前无人机的四元数数据 (6 到 10 列)
#     quaternions = xtraj[qn][:, 6:10]  # quaternions q0, q1, q2, q3
#     print(quaternions)
#     # 将四元数转换为欧拉角（滚转角, 俯仰角, 偏航角）
#     r = R.from_quat(quaternions)  # quaternion format is [q1, q2, q3, q0]
#     euler_angles = r.as_euler('xyz', degrees=True)  # 'xyz' represents roll, pitch, yaw order, returns in degrees

#     # 拆分成滚转角（roll）、俯仰角（pitch）、偏航角（yaw）
#     roll = euler_angles[:, 0]
#     pitch = euler_angles[:, 1]
#     yaw = euler_angles[:, 2]

#     # 取出时间数据
#     t = ttraj[qn]

#     # 创建新的子图来绘制
#     plt.figure(figsize=(10, 8))

#     # 绘制滚转角 roll 随时间变化的图
#     plt.subplot(3, 1, 1)
#     plt.plot(t, roll, label=f"Drone {qn} - Roll")
#     plt.ylabel("Roll (degrees)")
#     plt.legend()

#     # 绘制俯仰角 pitch 随时间变化的图
#     plt.subplot(3, 1, 2)
#     plt.plot(t, pitch, label=f"Drone {qn} - Pitch")
#     plt.ylabel("Pitch (degrees)")
#     plt.legend()

#     # 绘制偏航角 yaw 随时间变化的图
#     plt.subplot(3, 1, 3)
#     plt.plot(t, yaw, label=f"Drone {qn} - Yaw")
#     plt.ylabel("Yaw (degrees)")
#     plt.xlabel("Time (s)")
#     plt.legend()

# 遍历每个无人机
for qn in range(nquad):
    # 取出当前无人机的四元数数据 (6 到 10 列)
    quaternions = xtraj[qn][:, 6:10]  # 四元数 [q0, q1, q2, q3]
    
    # 将四元数转换为欧拉角（滚转角, 俯仰角, 偏航角）
    r = R.from_quat(quaternions)  # 四元数格式为 [q1, q2, q3, q0]
    euler_angles = r.as_euler('xyz', degrees=True)  # 'xyz' 表示欧拉角的顺序：roll, pitch, yaw
    
    # 拆分实际的滚转角（roll）、俯仰角（pitch）、偏航角（yaw）
    roll = euler_angles[:, 0]
    pitch = euler_angles[:, 1]
    yaw = euler_angles[:, 2]
    
    # 从 desired_Euler[qn] 取出期望的滚转角、俯仰角、偏航角
    desired_roll = desired_Euler[qn][0]    # 期望的滚转角
    desired_pitch = desired_Euler[qn][1]   # 期望的俯仰角
    desired_yaw = desired_Euler[qn][2]     # 期望的偏航角

    # 取出时间数据，每隔 5 个点取一个
    t = ttraj[qn][::5]
    roll = roll[::5]   # 每隔 5 个点取一个
    pitch = pitch[::5] # 每隔 5 个点取一个
    yaw = yaw[::5]     # 每隔 5 个点取一个

    # 创建新的子图来绘制
    plt.figure(figsize=(10, 8))

    # 绘制滚转角 roll 随时间变化的图 (实际 vs 期望)
    plt.subplot(3, 1, 1)
    plt.plot(t, roll, label=f"无人机 {qn} - 实际 Roll")
    plt.plot(t, [desired_roll] * len(t), label=f"无人机 {qn} - 期望 Roll", linestyle='--')
    plt.ylabel("滚转角 Roll (度)")
    plt.legend()

    # 绘制俯仰角 pitch 随时间变化的图 (实际 vs 期望)
    plt.subplot(3, 1, 2)
    plt.plot(t, pitch, label=f"无人机 {qn} - 实际 Pitch")
    plt.plot(t, [desired_pitch] * len(t), label=f"无人机 {qn} - 期望 Pitch", linestyle='--')
    plt.ylabel("俯仰角 Pitch (度)")
    plt.legend()

    # 绘制偏航角 yaw 随时间变化的图 (实际 vs 期望)
    plt.subplot(3, 1, 3)
    plt.plot(t, yaw, label=f"无人机 {qn} - 实际 Yaw")
    plt.plot(t, [desired_yaw] * len(t), label=f"无人机 {qn} - 期望 Yaw", linestyle='--')
    plt.ylabel("偏航角 Yaw (度)")
    plt.xlabel("时间 (秒)")
    plt.legend()

    plt.tight_layout()
    plt.show()




   



# 绘制保存的每个四旋翼的位置和速度
for qn in range(nquad):
    QP.truncate_hist()

    #   # 创建一个图像和轴对象
    # fig, ax = plt.subplots(3, 1, figsize=(8, 6))  # 创建3个子图

    # # 绘制实际位置
    # ax[0].plot(QP.time_hist, QP.state_hist[0, :], 'r-', label=' x (vic)')
    # ax[1].plot(QP.time_hist, QP.state_hist[1, :], 'r-', label=' y (vic)')
    # ax[2].plot(QP.time_hist, QP.state_hist[2, :], 'r-', label=' z (vic)')

    # # 设置轴标签和网格
    # for i in range(3):
    #     ax[i].set_xlabel('Time [s]')
    #     ax[i].set_ylabel('Position [m]')
    #     ax[i].grid(True)
    #     ax[i].legend()

    #     # 创建一个图像和轴对象
    # fig, ax = plt.subplots(3, 1, figsize=(8, 6))  # 创建3个子图

    # # 绘制期望位置
    # ax[0].plot(QP.time_hist, QP.state_des_hist[0, :], 'b-', label='期望位置 x (des)')
    # ax[1].plot(QP.time_hist, QP.state_des_hist[1, :], 'b-', label='期望位置 y (des)')
    # ax[2].plot(QP.time_hist, QP.state_des_hist[2, :], 'b-', label='期望位置 z (des)')

    # # 设置轴标签和网格
    # for i in range(3):
    #     ax[i].set_xlabel('Time [s]')
    #     ax[i].set_ylabel('Position [m]')
    #     ax[i].grid(True)
    #     ax[i].legend()

    # 创建一个图像和轴对象
    fig, ax = plt.subplots(3, 1, figsize=(8, 6))  # 创建3个子图

    # 绘制实际位置和期望位置
    ax[0].plot(QP.time_hist, QP.state_hist[0, :], 'r-', label='Actual x (vic)')
    ax[0].plot(QP.time_hist, QP.state_des_hist[0, :], 'b--', label='Desire x (des)')

    ax[1].plot(QP.time_hist, QP.state_hist[1, :], 'r-', label='Actual y (vic)')
    ax[1].plot(QP.time_hist, QP.state_des_hist[1, :], 'b--', label='Desire y (des)')

    ax[2].plot(QP.time_hist, QP.state_hist[2, :], 'r-', label='Actual z (vic)')
    ax[2].plot(QP.time_hist, QP.state_des_hist[2, :], 'b--', label='Desire z (des)')

    # 设置轴标签和网格
    for i in range(3):
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Position [m]')
        ax[i].grid(True)
        ax[i].legend()

     # 创建一个图像和轴对象
    fig, ax = plt.subplots(3, 1, figsize=(8, 6))  # 创建3个子图
    # 绘制实际位置和期望位置
    ax[0].plot(QP.time_hist, QP.state_hist[3, :], 'r-', label='Actual x (vic)')
    ax[0].plot(QP.time_hist, QP.state_des_hist[3, :], 'b--', label='Desire x (des)')

    ax[1].plot(QP.time_hist, QP.state_hist[4, :], 'r-', label='Actual y (vic)')
    ax[1].plot(QP.time_hist, QP.state_des_hist[4, :], 'b--', label='Desire y (des)')

    ax[2].plot(QP.time_hist, QP.state_hist[5, :], 'r-', label='Actual z (vic)')
    ax[2].plot(QP.time_hist, QP.state_des_hist[5, :], 'b--', label='Desire z (des)')

    # 设置轴标签和网格
    for i in range(3):
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Velocity [m]')
        ax[i].grid(True)
        ax[i].legend()

    # # 绘制位置
    # h_pos = plt.figure(f'Quad {qn} : position')
    # plot_state(h_pos, QP.state_hist[:3, :], QP.time_hist, 'pos', 'vic')  # 绘制实际位置
    # plot_state(h_pos, QP.state_des_hist[:3, :], QP.time_hist, 'pos', 'des')  # 绘制期望位置

    # # 绘制速度
    # h_vel = plt.figure(f'Quad {qn} : velocity')
    # plot_state(h_vel, QP.state_hist[3:6, :], QP.time_hist, 'vel', 'vic')  # 绘制实际速度
    # h_vel1 = plt.figure(f'Quad {qn} : velocitys')
    # plot_state(h_vel1, QP.state_des_hist[3:6, :], QP.time_hist, 'vel', 'des')  # 绘制期望速度
    h_fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # 绘制实际位置
    plot_state(h_fig, QP.state_hist[:3, :], QP.time_hist, name='pos', plot_type='vic', view='sep', ax=axs)


    # # 创建一个 3D 轴
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')


    # 绘制期望位置
    plot_state(h_fig, QP.state_des_hist[:3, :], QP.time_hist, name='pos', plot_type='des', view='sep', ax=axs)

    # 添加图例，确保两条线的可见性
    axs[0].legend(['Actual (vic)', 'Desire (des)'])



    v_fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # 绘制实际位置
    plot_state(v_fig, QP.state_hist[3:6, :], QP.time_hist, name='vel', plot_type='vic', view='sep', ax=axs)


    # # 创建一个 3D 轴
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')


    # 绘制期望位置
    plot_state(v_fig, QP.state_des_hist[3:6, :], QP.time_hist, name='vel', plot_type='des', view='sep', ax=axs)

    # 添加图例，确保两条线的可见性
    axs[0].legend(['Actual (vic)', 'Desire (des)'])
    try:
        plt.show()
    except KeyboardInterrupt:
        print("程序被手动中断。")



# 如果出现错误，则抛出异常
if err is not None:
    raise RuntimeError(err)

print('仿真结束。')


