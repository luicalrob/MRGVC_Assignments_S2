import threading

import numpy as np
import matplotlib.pyplot as plt

class Enclosing:
    def __init__(self, qi_x, qi_y, ci_x, ci_y, plot_results=False):
        self.lock = threading.Lock()

        # Robots qi positions, qn is assumed to be the target
        self.qi = np.array([qi_x,qi_y]).T

        self.num_pos = self.qi.shape[0]

        # Desired robot location
        self.ci = np.array([ci_x,ci_y]).T

        # Simulation params
        self.iters = 300
        self.K_c = 5
        self.delta_t = 0.01
        self.random_x = 1
        self.random_y = 1


        # Gather information to plot results
        self.plot_results = plot_results
        self.record_x = np.zeros((self.num_pos, self.iters))
        self.record_y = np.zeros((self.num_pos, self.iters))

        print("Starting positions: \n{}\nDesired positions: \n{}\n".format(self.qi.T, self.ci.T))
        plt.ion()
        self.run()

    def run(self):

        for iteration in range(self.iters):
            # Run the algorithm
            Q = self.compute_inter_positions(self.qi)
            C = self.compute_inter_positions(self.ci)
            A = C.T @ Q
            U, S, V_t = np.linalg.svd(A)
            d = np.sign(np.linalg.det(V_t.T @ U.T))
            D = np.array([[1,0],[0,d]])
            R = V_t.T @ D @ U.T
            # Compute the positions of the target relative to the robots
            q_Ni = -Q[self.num_pos-1::self.num_pos]
            c_Ni = -C[self.num_pos-1::self.num_pos]
            # Compute control input for each robot
            q_dot = np.zeros((self.num_pos, 2))
            for agent in range(self.num_pos):
                q_dot[agent,:] = self.K_c * (q_Ni[agent,:] - R @ c_Ni[agent,:])
            
            # Random movement of the target
            if np.random.uniform(0,1) > 0.80:
                q_dot[self.num_pos-1, :] += [np.random.uniform(-8,8), np.random.uniform(-8,8)]

            # Apply control
            self.qi += q_dot * self.delta_t
            
            # Record positions for plotting
            self.record_x[:,iteration] = self.qi[:,0]
            self.record_y[:,iteration] = self.qi[:,1]

            self.plot_robots()

        if self.plot_results:
            self.plot_results_over_time()


       
    # def compute_inter_positions(self, positions):
    
    #     rel_pos = np.zeros((self.num_pos*self.num_pos, 2))
    #     # Compute inter-robot relative positions
    #     for i in range(self.num_pos):
    #         for j in range(self.num_pos):
    #             #print("Position {} with {}".format(i,j))
    #             rel_pos[i + self.num_pos*j, 0] = positions[j,0] - positions[i,0]
    #             rel_pos[i + self.num_pos*j, 1] = positions[j,1] - positions[i,1]
    #     return rel_pos
    
    def compute_inter_positions(self, positions):
        diff = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
        return diff.reshape(-1, 2)

    def plot_robots(self):
        plt.clf()
        plt.xlim(-5,5)
        plt.ylim(-5,5)
        for i in range(self.num_pos-1):
                plt.plot(self.qi[i,0],self.qi[i,1],'bo')

        plt.plot(self.qi[self.num_pos-1,0],self.qi[self.num_pos-1,1],'ro')

        plt.draw()
        plt.pause(0.01)

    def plot_results_over_time(self):
        x_axis = np.arange(self.iters)

        plt.figure()
        plt.ioff()
        plt.plot(x_axis, self.record_x[-1, :],'r', label=f'Target')
        for i in range(self.num_pos-1):
            plt.plot(x_axis, self.record_x[i, :], label=f'Robot {i+1}') 
        plt.xlabel('Iteration')
        plt.ylabel('X-Coordinate')   
        plt.legend()
        plt.draw()

        plt.figure()
        plt.ioff()
        plt.plot(x_axis, self.record_y[-1, :],'r', label=f'Target')
        for i in range(self.num_pos-1):
            plt.plot(x_axis, self.record_y[i, :], label=f'Robot {i+1}')
        plt.xlabel('Iteration')
        plt.ylabel('Y-Coordinate')
        plt.legend()
        plt.draw()
        
        print("Close windows to finish...")
        plt.show(block=True)

if __name__ == "__main__":

    qx = [2.0, -3.0, -2.0, 3.0, 0]
    qy = [0.5, 1.0, -0.5, -1.0, 0]
    
    # Desired positions for a square formation
    side_length = 1.5
    cx = [side_length, -side_length, -side_length, side_length, 0]
    cy = [side_length, side_length, -side_length, -side_length, 0]
    
    enc = Enclosing(qx,qy,cx,cy, True)