#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, Point32
from mr_formation.msg import queue_position_plot
import numpy as np
import math

class RobotNode:
    def __init__(self):
        rospy.init_node('robot_node')

        self.robot_id = rospy.get_param('~robot_id')
        self.position = Point32()
        self.goal_points = Polygon()
        self.goal_point = Point32()
        self.goal_point.x = 0
        self.goal_point.y = 0
        self.num_goals = 0
        self.neighbor_positions = {}  # Positions of neighbors
        self.ri = np.zeros((1, 2))
        self.weight = 0.018

        # Publishers
        self.position_pub = rospy.Publisher(f'/robot_{self.robot_id}/position', Point32, queue_size=10)
        self.plot_pub = rospy.Publisher('mr_formation/queue_position_plot', queue_position_plot, queue_size=10)

        # Subscribers
        rospy.Subscriber('/goal_points', Polygon, self.goal_points_callback)
        rospy.Subscriber(f'/robot_{self.robot_id}/desired_r', Point32, self.relative_positions_callback)
        rospy.Subscriber(f'/robot_{self.robot_id}/neighbors_positions', Polygon, self.neighbors_positions_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def goal_points_callback(self, msg):
        # Ensure robot_id is within the range of available points
        self.goal_points = msg.points
        self.num_goals = len(msg.points)
        if 0 < self.robot_id <= len(msg.points):
            point = msg.points[self.robot_id - 1]  # Convert to 0-based index
            rospy.loginfo(f"Robot ID {self.robot_id}: x={point.x}, y={point.y}, z={point.z}")
            self.goal_point.x = point.x
            self.goal_point.y = point.y
        else:
            rospy.logwarn(f"Robot ID {self.robot_id} is out of range. Total points available: {len(msg.points)}")

    def relative_positions_callback(self, msg):  
        self.ri[0,0] = msg.x
        self.ri[0,1] = msg.y

    def neighbors_positions_callback(self, msg):

        desired_r = np.zeros((1,2))
        rel_pos_matrix = np.zeros((self.num_goals, 2))

        for j, point in enumerate(msg.points):
            self.neighbor_positions[j] = [point.x, point.y]

            if (self.robot_id != j and (point.x != 0 or point.y != 0)):
                rel_pos_matrix[j, 0] = (self.goal_points[j].x - self.goal_point.x)
                rel_pos_matrix[j, 1] = (self.goal_points[j].y - self.goal_point.y)
        
        self.ri[0,0] = -np.sum(rel_pos_matrix[:,0])
        self.ri[0,1] = -np.sum(rel_pos_matrix[:,1])

                

    def compute_new_position(self):
        if not self.goal_point:
            return  # If no goal_point positions are available, do nothing

        # Consensus algorithm with weights
        new_position = Point32()
        new_position.x = self.position.x
        new_position.y = self.position.y

        for (x, y) in self.neighbor_positions.values():
            if (x != 0 or y != 0):
                new_position.x += self.weight * (x - self.position.x)
                new_position.y += self.weight * (y - self.position.y)

        new_position.x += self.weight * self.ri[0, 0]
        new_position.y += self.weight * self.ri[0, 1]

        position_change = math.sqrt((self.position.x - new_position.x) ** 2 + (self.position.y - new_position.y) ** 2)
        print("change: "+ str(position_change))
        if position_change > 0.095:
            self.position = new_position
        

    def publish_position(self):
        pos_msg = queue_position_plot()
        pos_msg.robot_id = self.robot_id
        pos_msg.x = self.position.x
        pos_msg.y = self.position.y
        self.plot_pub.publish(pos_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.compute_new_position()
            self.position_pub.publish(self.position)
            self.publish_position()
            self.rate.sleep()

if __name__ == '__main__':
    RobotNode().run()
