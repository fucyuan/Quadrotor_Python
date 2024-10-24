import matplotlib.pyplot as plt

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
