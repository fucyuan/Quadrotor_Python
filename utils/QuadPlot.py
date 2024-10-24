import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import QuatToRot, quad_pos  # 假设这些函数已定义

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
