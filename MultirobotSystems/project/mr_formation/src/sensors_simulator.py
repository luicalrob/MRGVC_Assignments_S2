#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point32, Polygon
import numpy as np
from random import uniform

class SensorsSimulatorNode:
    def __init__(self, n_robots):

        self.n_robots = n_robots
        self.positions = {i: Point32(0, 0, 0) for i in range(n_robots)}
        self.position_subs = [rospy.Subscriber(f'/robot_{i}/position', Point32, self.position_callback, i) for i in range(1, n_robots + 1)]
        self.neighbors_pubs = [rospy.Publisher(f'/robot_{i}/neighbors_positions', Polygon, queue_size=10) for i in range(1, n_robots + 1)]

        self.rate = rospy.Rate(2)  # 2 Hz

    def position_callback(self, msg, robot_id):
        self.positions[robot_id] = msg

    def publish_neighbors_positions(self):
        for i in range(self.n_robots):
            neighbors_poly = Polygon()
            for j in range(self.n_robots):
                if i != j:
                    neighbors_poly.points.append(Point32(self.positions[j].x, self.positions[j].y, 0))
            
            self.neighbors_pubs[i].publish(neighbors_poly)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_neighbors_positions()
            self.rate.sleep()

if __name__ == '__main__':
    sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
    num_args=len(sysargv)
    if (num_args >= 1):
        n_robots = int(sysargv[1])
    else:
        n_robots = 1

    try:
        rospy.init_node('sensors_simulator', anonymous=False)
        node=SensorsSimulatorNode(n_robots)
        node.run()
    except rospy.ROSInterruptException:
        pass
