import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import time

# 假设这些函数已经定义好
from utils import QuadPlot, init_state, crazyflie, terminate_check, plot_state,quadEOM
from trajectories import step, circle, diamond
from controller import controller



# 你可以在这里更改轨迹
# trajhandle = step
# trajhandle = circle
trajhandle = diamond

# 控制器
controlhandle = controller

# 实时运行
real_time = True

# *********** 你不需要修改以下任何内容 **********
# 四旋翼数量
nquad = 1

# 最大时间
time_tol = 25

# 仿真参数
params = crazyflie.crazyflie()  # 调用模块内的函数

# **************************** 图像设置 *****************************
print('初始化图像...')
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1,1,1])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.grid(True)

quadcolors = plt.cm.get_cmap('hsv', nquad)

# *********************** 初始条件设置 ***********************
print('设置初始条件...')
max_iter = 5000  # 最大迭代次数
starttime = 0  # 仿真开始时间（秒）
tstep = 0.01  # 解决方案给定的时间步长
cstep = 0.05  # 图像捕获时间间隔
nstep = int(cstep / tstep)
current_time = starttime  # 当前时间
err = None

# 初始化每个四旋翼的状态
xtraj = []
ttraj = []
x0 = []
stop = []
for qn in range(nquad):
    des_start = trajhandle.diamond(0, qn)
    des_stop = trajhandle.diamond(np.inf, qn)
    stop.append(des_stop['pos'])
    x0.append(init_state.init_state(des_start['pos'], 0))
    xtraj.append(np.zeros((max_iter * nstep, len(x0[qn]))))
    ttraj.append(np.zeros(max_iter * nstep))

x = x0  # 状态
pos_tol = 0.01  # 位置容差
vel_tol = 0.01  # 速度容差

# ************************* 运行仿真 *************************
OUTPUT_TO_VIDEO = 1
if OUTPUT_TO_VIDEO == 1:
    import cv2
    video_filename = 'diamond.avi'
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 20.0, (640, 480))

print('仿真运行中....')
for iter in range(max_iter):
    timeint = np.arange(current_time, current_time + cstep, tstep)
    
    tic = time.time()

    # 遍历每个四旋翼
    for qn in range(nquad):
        # 初始化四旋翼图像
        if iter == 1:
            QP = QuadPlot(qn, x0[qn], 0.1, 0.04, quadcolors(qn), max_iter, ax)
            desired_state = trajhandle(time, qn)
            QP.update_quad_plot(x[qn], np.hstack([desired_state['pos'], desired_state['vel']]), time)

        # 使用 odeint 进行仿真
        def quad_eom(s, t, qn, controlhandle, trajhandle, params):
            return quadEOM.quadEOM(t, s, qn, controlhandle, trajhandle, params)

        xsave = odeint(quad_eom, x[qn], timeint, args=(qn, controlhandle, trajhandle, params))
        x[qn] = xsave[-1, :]

        # 保存轨迹
        xtraj[qn][(iter) * nstep:(iter + 1) * nstep, :] = xsave[:-1, :]
        ttraj[qn][(iter) * nstep:(iter + 1) * nstep] = timeint[:-1]

        # 更新四旋翼图像
        desired_state = trajhandle(time + cstep, qn)
        QP.update_quad_plot(x[qn], np.hstack([desired_state['pos'], desired_state['vel']]), time + cstep)

        if OUTPUT_TO_VIDEO == 1:
            frame = np.array(fig.canvas.renderer.buffer_rgba())
            out.write(cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR))

    time += cstep  # 更新仿真时间
    elapsed = time.time() - tic

    # 检查 odeint 是否超时
    if elapsed > cstep * 50:
        err = 'odeint 不稳定'
        break

    # 实时暂停
    if real_time and (elapsed < cstep):
        time.sleep(cstep - elapsed)

    # 检查终止条件
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol):
        break

if OUTPUT_TO_VIDEO == 1:
    out.release()

# ************************* 后处理 *************************
# 截取 xtraj 和 ttraj
for qn in range(nquad):
    xtraj[qn] = xtraj[qn][:iter * nstep, :]
    ttraj[qn] = ttraj[qn][:iter * nstep]

# 绘制保存的每个机器人的位置和速度
for qn in range(nquad):
    QP.truncate_hist()

    # 绘制位置
    h_pos = plt.figure(f'Quad {qn} : position')
    plot_state(h_pos, QP.state_hist[:3, :], QP.time_hist, '位置', '实际')
    plot_state(h_pos, QP.state_des_hist[:3, :], QP.time_hist, '位置', '期望')

    # 绘制速度
    h_vel = plt.figure(f'Quad {qn} : velocity')
    plot_state(h_vel, QP.state_hist[3:6, :], QP.time_hist, '速度', '实际')
    plot_state(h_vel, QP.state_des_hist[3:6, :], QP.time_hist, '速度', '期望')

if err is not None:
    raise RuntimeError(err)

print('仿真结束。')
