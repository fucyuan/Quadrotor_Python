import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.integrate import solve_ivp  # solve_ivp 是 ode45 的等效函数

import time

# 假设这些函数已经定义好
from utils import QuadPlot, init_state, crazyflie, terminate_check, plot_state, quadEOM
from trajectories import step, circle, diamond
from controller import controller

# 你可以在这里更改轨迹
# trajhandle = step
trajhandle = circle
# trajhandle = diamond  # 使用 diamond 轨迹

# 控制器
controlhandle = controller

# 实时运行
real_time = True

# *********** 你不需要修改以下任何内容 **********
# 四旋翼数量
nquad = 1

# 最大时间
time_tol = 25  # 仿真最大时间限制

# 仿真参数
params = crazyflie()  # 调用模块内的 crazyflie 参数

# **************************** 图像设置 *****************************
print('初始化图像...')
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')  # 3D 绘图
ax.set_box_aspect([1, 1, 1])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.grid(True)

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
OUTPUT_TO_VIDEO = 0
if OUTPUT_TO_VIDEO == 1:
    import cv2
    video_filename = 'diamond.avi'
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 20.0, (640, 480))

print('仿真运行中....')
for iter in range(max_iter):
    timeint = np.arange(current_time, current_time + cstep, tstep)  # 时间间隔
    
    tic = time.time()

    # 遍历每个四旋翼
    for qn in range(nquad):
        # 初始化四旋翼图像
        if iter == 0:  # 注意这里需要修正为 iter == 0，第一次绘图
            QP = QuadPlot(qn, x0[qn], 0.1, 0.04, quadcolors(qn), max_iter, ax)
            desired_state = trajhandle(current_time, qn)
            QP.update_quad_plot(x[qn], np.hstack([desired_state['pos'], desired_state['vel']]), current_time)

        # 使用 odeint 进行仿真
        def quad_eom(s, t, qn, controlhandle, trajhandle, params):
            return quadEOM(t, s, qn, controlhandle, trajhandle, params)

        # 仿真四旋翼动力学
        # xsave = odeint(quadEOM, x[qn], timeint, args=(qn, controlhandle, trajhandle, params), atol=1e-9, rtol=1e-6)
        xsave = odeint(quad_eom, x[qn], timeint, args=(qn, controlhandle, trajhandle, params), atol=1e-9, rtol=1e-6)
        x[qn] = xsave[-1, :]  # 更新状态

        # 计算实际可以保存的步数，使用 min 确保长度一致
        num_steps = min(xsave.shape[0] - 1, len(timeint) - 1)

        # 保存轨迹和时间
        xtraj[qn][iter * num_steps:(iter + 1) * num_steps, :] = xsave[:num_steps, :]
        ttraj[qn][iter * num_steps:(iter + 1) * num_steps] = timeint[:num_steps]


        # # 保存轨迹
        # xtraj[qn][iter * (nstep - 1):(iter + 1) * (nstep - 1), :] = xsave[:-1, :]  # 保存轨迹
        # ttraj[qn][iter * (nstep - 1):(iter + 1) * (nstep - 1)] = timeint[:-1]  # 保存时间
        # import numpy as np

        # # 假设 xtraj 和 ttraj 是预先定义的列表，每个列表中包含 NumPy 数组
        # # qn 是索引，iter 和 nstep 是循环控制变量

        # xtraj[qn][(iter-1)*nstep : iter*nstep, :] = xsave[:-1, :]
        # ttraj[qn][(iter-1)*nstep : iter*nstep] = timeint[:-1]
    

        # 更新四旋翼图像
        desired_state = trajhandle(current_time + cstep, qn)
        QP.update_quad_plot(x[qn], np.hstack([desired_state['pos'], desired_state['vel']]), current_time + cstep)

        if OUTPUT_TO_VIDEO == 1:
            fig.canvas.draw()  # 更新绘图
            frame = np.array(fig.canvas.buffer_rgba())  # 获取图像数据
            out.write(cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR))  # 保存为视频

    current_time += cstep  # 更新仿真时间
    elapsed = time.time() - tic  # 计算仿真步长耗时

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

if OUTPUT_TO_VIDEO == 1:
    out.release()  # 关闭视频文件

# ************************* 后处理 *************************
# 截取 xtraj 和 ttraj
for qn in range(nquad):
    xtraj[qn] = xtraj[qn][:iter * nstep, :]  # 截取轨迹数据
    ttraj[qn] = ttraj[qn][:iter * nstep]  # 截取时间数据

# 绘制保存的每个四旋翼的位置和速度
for qn in range(nquad):
    QP.truncate_hist()

    # 绘制位置
    h_pos = plt.figure(f'Quad {qn} : position')
    plot_state(h_pos, QP.state_hist[:3, :], QP.time_hist, 'pos', 'vic')  # 绘制实际位置
    plot_state(h_pos, QP.state_des_hist[:3, :], QP.time_hist, 'pos', 'des')  # 绘制期望位置

    # 绘制速度
    h_vel = plt.figure(f'Quad {qn} : velocity')
    plot_state(h_vel, QP.state_hist[3:6, :], QP.time_hist, 'vel', 'vic')  # 绘制实际速度
    plot_state(h_vel, QP.state_des_hist[3:6, :], QP.time_hist, 'vel', 'des')  # 绘制期望速度

# 保持所有窗口打开
plt.show()


# 如果出现错误，则抛出异常
if err is not None:
    raise RuntimeError(err)

print('仿真结束。')
