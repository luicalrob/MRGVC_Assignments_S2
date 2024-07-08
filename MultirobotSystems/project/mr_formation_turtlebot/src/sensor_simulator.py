#!/usr/bin/env python
import random
import math
import rospy
from mr_formation_turtlebot.msg import queue_position_plot, range_bearing
from mr_formation_turtlebot.srv import sensor_measurement, sensor_measurementResponse
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class SensorSimulator:

    def __init__(self):
        rospy.init_node("sensor_simulator")

        #Wait for things to be initialized
        rospy.sleep(3.0)        
        
        #State subscribers
        self.n_robots = rospy.get_param("~n_robots")
        self.robot_positions = {i: None for i in range(self.n_robots)}
        self.odom_subs = []
        for i in range(1, self.n_robots+1):
            odom_sub = rospy.Subscriber(f"/robot_{i}/odom", Odometry, self.position_cb, i)
            rospy.loginfo("[sensor_simulator] Subscribing to topic robot_{}/odom".format(i))
            self.odom_subs.append(odom_sub)
       
         # Create server
        self.sensor_measurements_server = rospy.Service(
            '/sensor_measurements_server', sensor_measurement, self.sensor_measurement_cb)

        
    def position_cb(self, msg, robot_id):
        
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        self.robot_positions[robot_id] = (position.x, position.y, yaw)

    def sensor_measurement_cb(self, req):
        

        rospy.loginfo("[sensor_simulator] Received request from robot {}".format(req.robot_id))

        res = sensor_measurementResponse()
        if req.robot_id not in self.robot_positions or self.robot_positions[req.robot_id] is None:
            return res

        requesting_robot_position = self.robot_positions[req.robot_id]

        #Go through all robots other than the requesting one
        for robot_id, position in self.robot_positions.items():
            if robot_id != req.robot_id and position is not None:
                #compute relative distance and bearing with each of them
                distance, bearing = self.compute_range_and_bearing(
                    requesting_robot_position, position)
                #select those that are within range
                if distance <= req.max_range:

                    measurement = range_bearing()
                    measurement.robot_id = robot_id
                    measurement.range = distance
                    measurement.bearing = bearing
                    res.measurements.append(measurement)
        
        return res
    
    def compute_range_and_bearing(self, pos1, pos2):
        
        x1, y1, yaw1 = pos1
        x2, y2, _ = pos2

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        bearing = angle_to_target - yaw1

        # Normalize bearing to [-pi, pi]
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi

        return distance, bearing


if __name__ == "__main__":
    
    robot_node = SensorSimulator()
    rospy.spin()