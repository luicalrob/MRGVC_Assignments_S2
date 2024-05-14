import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.stats as st

def matlab_style_gauss2D(shape=(3,3), sigma=0.5):
    """
    Generates a 2D Gaussian mask.
    """
    m, n = [(ss-1.)/2. for ss in shape]
    y, x = np.ogrid[-m:m+1, -n:n+1]
    h = np.exp(-(x*x + y*y) / (2.*sigma*sigma))
    h[h < np.finfo(h.dtype).eps * h.max()] = 0
    sumh = h.sum()
    if sumh != 0:
        h /= sumh
    return h

class Environment:
    def __init__(self, size, period, A, B, C, q):
        """
        Initialize the environment with given parameters.
        """
        self.size = size
        self.period = period
        self.A = A
        self.B = B
        self.C = C
        self.q = q
        self.map = np.zeros(size)
        self.k = 0
        self.end_k = 1000
        self.previous_states = np.zeros((size[0], size[1], self.end_k))
        self.robot_x = 10
        self.robot_y = 10
        self.sigma = matlab_style_gauss2D((13, 13), sigma=2.4)
        self.robot_action_half_size = self.sigma.shape[0] // 2
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.desired = 50
        self.errors = []

    def run(self, route_x, route_y, end_k):
        """
        Run the simulation for a given number of iterations.
        """
        self.end_k = end_k
        F = np.exp(self.A * self.period)
        G = (self.B / self.A) * (np.exp(self.A * self.period) - 1)
        # K = 10000.0  # Assume K is a constant
        coverage_local = self.map[self.robot_x-self.robot_action_half_size:self.robot_x+self.robot_action_half_size+1,  self.robot_y-self.robot_action_half_size:self.robot_y+self.robot_action_half_size+1]
        K = self.C*(np.sum(np.sum(self.B * self.sigma * (self.desired-coverage_local))))**(2*self.q-1)
        alpha = K * self.sigma

        for self.k in range(self.end_k):
            self.previous_states[:, :, self.k] = self.map
            action_map = np.zeros(self.map.shape)
            action_map[self.robot_x-self.robot_action_half_size:self.robot_x+self.robot_action_half_size+1,
                       self.robot_y-self.robot_action_half_size:self.robot_y+self.robot_action_half_size+1] = alpha
            self.map = F * self.map + G * action_map

            self.draw_map()
            self.draw_error()

            self.robot_x += route_x[self.k]
            self.robot_y += route_y[self.k]

    def draw_map(self):
        """
        Draw the coverage map.
        """
        self.ax1.clear()
        self.ax1.imshow(self.map, cmap='jet', vmin=0, vmax=100)
        self.ax1.set_title('Coverage Map')
        self.fig.canvas.draw()
        plt.pause(0.0001)

    def draw_error(self):
        """
        Draw the coverage error over time.
        """
        self.errors.append(np.sum((np.ones(self.size) * self.desired - self.map)**2) / (self.size[0] * self.size[1]))
        self.ax2.clear()
        self.ax2.plot(self.errors)
        self.ax2.set_title('Coverage Error')
        self.fig.canvas.draw()
        plt.pause(0.0001)

if __name__ == '__main__':
    # Initialize environment with given parameters
    my_environment = Environment([100, 100], 1, -1/200, 1/100, 120000, 2)

    end_k = 1000
    route_x = []
    route_y = []

    for j in range(4):
        route_y.extend([1] * 80)
        route_x.extend([0] * 80)
        route_y.extend([0] * 8)
        route_x.extend([1] * 8)
        route_y.extend([-1] * 80)
        route_x.extend([0] * 80)
        route_y.extend([0] * 8)
        route_x.extend([1] * 8)

    route_x.extend([0] * 100)
    route_y.extend([0] * 100)

    my_environment.run(route_x, route_y, end_k)
