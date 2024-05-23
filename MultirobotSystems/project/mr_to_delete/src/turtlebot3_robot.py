#!/usr/bin/env python
import rospy
import random
from turtlebot3_msgs.msg import SensorState
from std_srvs.srv import Empty, EmptyResponse


class TurtleBot3Robot:

    def __init__(self):
        rospy.init_node("turtlebot3_robot")

        # Class attributes
        self.robot_id = 0
        self.t_local = 1/2.0  # default
        self.position = [0, 0]
        self.neighbors = []
        self.available_neighbors = []

        # Read parameters
        if self.read_params():
            rospy.loginfo("[TurtleBot3 Robot] Successfully launched robot {} with position {} and neighbors {}".format(
                self.robot_id, self.position, self.neighbors))

        # Create publisher
        self.sensor_publisher = rospy.Publisher(
            '/turtlebot3/sensor_state', SensorState, queue_size=10)

        # Create service
        self.move_service = rospy.Service(
            'move_turtlebot3', Empty, self.move_turtlebot3_cb)

        # Obtain initial position
        self.update_position()

        # Create timer to request gossip update every t_local cycles
        self.timer = rospy.Timer(rospy.Duration(
            self.t_local), self.request_gossip_update)

    def read_params(self):
        try:
            self.robot_id = int(rospy.get_param("~robot_id"))
            self.neighbors = rospy.get_param("~neighbors").split(',')
            self.t_local = float(rospy.get_param("~t_local"))

            return True
        except rospy.ServiceException as e:
            print("Parameters not set: " + str(e))
            return False

    def update_position(self):
        # Simulate obtaining position from sensors
        self.position[0] = random.uniform(-10, 10)
        self.position[1] = random.uniform(-10, 10)

    def move_turtlebot3_cb(self, req):
        # Simulate movement
        rospy.loginfo("[TurtleBot3 Robot {}] Moving to a new position".format(
            self.robot_id))
        self.update_position()
        return EmptyResponse()

    def request_gossip_update(self, event):  # timer callback
        if self.available_neighbors is not None:
            target_id = random.choice(self.available_neighbors)
        else:
            return

        rospy.wait_for_service('move_turtlebot3')

        try:
            move_turtlebot3 = rospy.ServiceProxy(
                'move_turtlebot3', Empty)
            res = move_turtlebot3()

            rospy.loginfo("[TurtleBot3 Robot {}] Position updated".format(
                self.robot_id))
        except rospy.ServiceException as e:
            print("[TurtleBot3 Robot {}] Service call failed: {}".format(
                self.robot_id, str(e)))


if __name__ == "__main__":
    robot_node = TurtleBot3Robot()
    rospy.spin()
