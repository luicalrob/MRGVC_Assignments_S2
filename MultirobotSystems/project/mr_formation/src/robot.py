#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, Point32
from mr_shape_formation.msg import queue_position_plot
import numpy as np

class RobotNode:
    def __init__(self):
        rospy.init_node('robot_node')

        self.robot_id = rospy.get_param('~robot_id')
        self.position = Point32()
        self.goal_point = Point32()
        self.goal_point.x = 0
        self.goal_point.y = 0
        self.positions = {}  # Positions of neighbors

        # Publishers
        self.position_pub = rospy.Publisher(f'/robot_{self.robot_id}/position', Point32, queue_size=10)
        self.plot_pub = rospy.Publisher('mr_shape_formation/queue_position_plot', queue_position_plot, queue_size=10)
        rospy.Subscriber('/goal_points', Polygon, self.goal_points_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def goal_points_callback(self, msg):
        # Ensure robot_id is within the range of available points
        if 0 < self.robot_id <= len(msg.points):
            point = msg.points[self.robot_id - 1]  # Convert to 0-based index
            rospy.loginfo(f"Robot ID {self.robot_id}: x={point.x}, y={point.y}, z={point.z}")
            self.goal_point.x = point.x
            self.goal_point.y = point.y
        else:
            rospy.logwarn(f"Robot ID {self.robot_id} is out of range. Total points available: {len(msg.points)}")


    def compute_new_position(self):
        # Placeholder for the consensus algorithm logic
        # Update self.position based on self.goal_point, self.positions, and the consensus algorithm
        self.position.x += (self.goal_point.x - self.position.x) * 0.04  # Simple proportional controller
        self.position.y += (self.goal_point.y - self.position.y) * 0.04  # Simple proportional controller

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
