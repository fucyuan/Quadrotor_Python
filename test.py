import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# 初始化图像
print('初始化图像...')
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 设置初始轴标签
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

# 启用网格
ax.grid(True)

# 模拟动态数据更新
for i in range(100):  # 假设有 100 个时间步
    # 清除上一次绘图内容
    ax.cla()

    # 设置新的坐标轴比例
    ax.set_box_aspect([1, 1, 1])  # 保持比例不变

    # 设置新的标签
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    
    # 更新动态数据
    x = np.random.rand(10)  # 模拟新的 x 数据
    y = np.random.rand(10)  # 模拟新的 y 数据
    z = np.random.rand(10)  # 模拟新的 z 数据
    
    # 绘制更新后的数据
    ax.scatter(x, y, z, c='r', marker='o')  # 3D 散点图

    # 设置网格
    ax.grid(True)
    
    # 重新绘制图像
    plt.pause(0.1)  # 暂停 0.1 秒，模拟实时数据更新

plt.show()
