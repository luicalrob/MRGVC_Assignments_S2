#!/usr/bin/env python
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import animation
matplotlib.use("TkAgg")

import rospy
from std_msgs.msg import String
from mr_rendezvous_deployment.msg import queue_position_plot
from mr_rendezvous_deployment.srv import gossip_update, gossip_updateResponse


class Plotter():
    def __init__(self, n_robots):
        # Class attributes
        self.n_robots = n_robots
        # Positions of all the robots. Assume consecutive ids 1..n, here indexed as 0..n-1 
        self.v_robot_x = np.zeros([n_robots])
        self.v_robot_y = np.zeros([n_robots])
        self.sub = rospy.Subscriber('mr_rendezvous_deployment/queue_position_plot', queue_position_plot,
        self.position_cb)
        self.draw_robot()

    def draw_robot(self):
        while not rospy.is_shutdown():
            plt.clf()
            plt.xlim(-10,10)
            plt.ylim(-10,10)
            
            if np.any(self.v_robot_x != 0) or np.any(self.v_robot_y != 0):
                plt.scatter(self.v_robot_x, self.v_robot_y, c='blue', marker='o')
                plt.draw()
                plt.show(block=False)
                plt.pause(0.01)
            
            rospy.sleep(0.25) # twice per second. You can increase this rate
    
    def position_cb(self, new_position):
        self.v_robot_x[new_position.robot_id-1]=new_position.x
        self.v_robot_y[new_position.robot_id-1]=new_position.y
        print(self.v_robot_x)
        print(self.v_robot_y)
        
if __name__ == '__main__':
    sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
    num_args=len(sysargv)
    if (num_args >= 1):
        n_robots = int(sysargv[1])
    else:
        n_robots = 1
    try:
        rospy.init_node('position_plotter', anonymous=False)
        my_plotter=Plotter(n_robots)
        print('[visualization_node] Printing the positions of ', my_plotter.n_robots)
    except rospy.ROSInterruptException:
        pass