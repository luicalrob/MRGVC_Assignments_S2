#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, Point32
from mr_formation.msg import queue_position_plot
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
        self.relative_positions = np.zeros((10, 2))
        self.weight = 0.1 

        # Publishers
        self.position_pub = rospy.Publisher(f'/robot_{self.robot_id}/position', Point32, queue_size=10)

        self.position_pub = rospy.Publisher(f'/robot_{self.robot_id}/position', Point32, queue_size=10)
        self.plot_pub = rospy.Publisher('mr_formation/queue_position_plot', queue_position_plot, queue_size=10)

        # Subscribers
        rospy.Subscriber('/goal_points', Polygon, self.goal_points_callback)
        rospy.Subscriber('/relative_positions', Polygon, self.relative_positions_callback)
        rospy.Subscriber(f'/robot_{self.robot_id}/neighbors_positions', Polygon, self.neighbors_positions_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def goal_points_callback(self, msg):
        # Ensure robot_id is within the range of available points
        if 0 < self.robot_id <= len(msg.points):
            point = msg.points[self.robot_id - 1]  # Convert to 0-based index
            rospy.loginfo(f"Robot ID {self.robot_id}: x={point.x}, y={point.y}, z={point.z}")
            self.goal_point.x = point.y
            self.goal_point.y = -point.x
        else:
            rospy.logwarn(f"Robot ID {self.robot_id} is out of range. Total points available: {len(msg.points)}")

    def relative_positions_callback(self, msg):
        self.relative_positions = np.zeros((len(msg.points), 2))
        for i, point in enumerate(msg.points):
            self.relative_positions[i, 0] = point.x
            self.relative_positions[i, 1] = point.y

    def neighbors_positions_callback(self, msg):
        self.positions = {}
        for i, point in enumerate(msg.points):
            self.positions[i] = (point.x, point.y)

    def compute_new_position(self):
        if not self.positions:
            return  # If no neighbors' positions are available, do nothing

        # Consensus algorithm with weights
        new_x = self.position.x
        new_y = self.position.y
        for i, (x, y) in self.positions.items():
            rel_x = self.relative_positions[i, 0]
            rel_y = self.relative_positions[i, 1]
            new_x += self.weight * (x - self.position.x - rel_x)
            new_y += self.weight * (y - self.position.y - rel_y)

        self.position.x = new_x
        self.position.y = new_y

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
