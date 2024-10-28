import numpy as np

nquad = 3  # 无人机数量
max_iter = 100  # 最大迭代次数
nstep = 10  # 每个迭代的步数

# 创建存储 desired_Euler 的矩阵
desired_Euler = []
for qn in range(nquad):
    desired_Euler.append(np.zeros((max_iter * nstep, 3)))  # 3 表示 roll, pitch, yaw

# 添加数据示例
desired_Euler[0][0, :] = [0.0, 0.0, 0.0]  # 设置第一个无人机第一个时间点的欧拉角
desired_Euler[1][1, :] = [10.0, 5.0, 2.0]  # 设置第二个无人机第二个时间点的欧拉角
desired_Euler[2][2, :] = [20.0, 15.0, 10.0]  # 设置第三个无人机第三个时间点的欧拉角

print(desired_Euler[1])
