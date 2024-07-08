#!/usr/bin/env python
import random
import math
import rospy
from mr_formation_turtlebot.msg import range_bearing
from mr_formation_turtlebot.srv import gossip_update, gossip_updateResponse
from mr_formation_turtlebot.srv import sensor_measurement
from geometry_msgs.msg import PoseStamped, Pose, Twist, Polygon, Point32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class FormationControl:

    def __init__(self):
        rospy.init_node("robot")

        self.Kp_v = 0.1
        self.Kp_alpha = 2.0
        self.Kp_beta = -0.1
        self.rho = 0.0
        self.alpha = 0.0
        self.beta = 0.0

        self.ri = np.zeros((1, 2))
        self.num_goals = 0
        self.goal_points = Polygon() #Goals for all robots
        self.goal_point = Point32() #Goal for this particular robot
        self.neighbor_positions = {}

        self.weight = 0.025

        self.pose = Pose()
        self.target_pose = Pose()
        self.error = PoseStamped()
        self.target_reached = False
        self.velocity = Twist()
        self.path_received = False

        #Wait for things to initialize
        rospy.sleep(5.0)

        # Read parameters
        if self.read_params():
            rospy.loginfo("[rendezvous robot] Succesfully launched robot {} with position {} and neighbors {}".format(
                self.robot_id, self.pose.position, self.neighbors))
            
        #State subscriber
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.position_cb)
        self.goal_sub = rospy.Subscriber('/goal_points', Polygon, self.goal_points_callback)
        self.relative_goal_sub = rospy.Subscriber(f'/robot_{self.robot_id}/desired_r', Point32, self.relative_positions_callback)


        # Create publishers
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        #This is for debug
        self.error_publisher = rospy.Publisher("debug/error", PoseStamped, queue_size=10)

        
        # Create timer to perform velocity control
        self.control_timer = rospy.Timer(rospy.Duration(
            0.1), self.control_update)

    def read_params(self):
        try:

            self.robot_id = int(rospy.get_param("~robot_id"))
            self.neighbors = rospy.get_param("~neighbors")
            self.max_range = rospy.get_param("/sensor_range")          
            self.neighbors = [int(x)
                                for x in self.neighbors.split()]
            self.available_neighbors = []
            for neighbor in self.neighbors:
                if neighbor < self.robot_id:
                    self.available_neighbors.append(neighbor)

            return True
        except rospy.ServiceException as e:
            print("Parameters not set: "+str(e))
            return False
        
    def position_cb(self, msg):
        
        self.pose = msg.pose.pose

    def goal_points_callback(self, msg):
        # Ensure robot_id is within the range of available points
        self.goal_points = msg.points
        self.num_goals = len(msg.points)

        if 0 < self.robot_id <= len(msg.points):
            point = msg.points[self.robot_id - 1]  # Convert to 0-based index
            rospy.loginfo(f"Robot ID {self.robot_id}: x={point.x}, y={point.y}, z={point.z}")
            # self.target_pose.position.x = point.x
            # self.target_pose.position.y = point.y
            self.goal_point.x = point.x
            self.goal_point.y = point.y

            self.path_received = True
        else:
            rospy.logwarn(f"Robot ID {self.robot_id} is out of range. Total points available: {len(msg.points)}")

    
    def relative_positions_callback(self, msg):        
        self.ri[0,0] = msg.x
        self.ri[0,1] = msg.y

    def neighbor_search(self):

        rospy.loginfo("[Robot {}] asking for measurements".format(self.robot_id))
        rospy.wait_for_service("/sensor_measurements_server")

        try:

            service_neighbors_update = rospy.ServiceProxy(
            "/sensor_measurements_server", sensor_measurement)
            
            res = service_neighbors_update(self.robot_id, self.max_range)
            self.available_neighbors = []
            self.neighbor_positions = {}

            rel_pos_matrix = np.zeros((self.num_goals, 2))

            for measurement in res.measurements:
                neighbor = measurement.robot_id
                distance = measurement.range
                bearing = measurement.bearing

                neighbor_x, neighbor_y = self.compute_neighbor_position(
                    (self.pose.position.x, self.pose.position.y, euler_from_quaternion(
                        [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[2]),
                    distance, bearing)
                
                self.neighbor_positions[neighbor - 1] = (neighbor_x, neighbor_y)

                rel_pos_matrix[neighbor - 1, 0] = (self.goal_points[neighbor - 1].x - self.goal_point.x)
                rel_pos_matrix[neighbor - 1, 1] = (self.goal_points[neighbor - 1].y - self.goal_point.y)
                
                rospy.loginfo("[Robot {}] found neighbor {} at distance {}, angle {}".format(self.robot_id, neighbor, distance, bearing))
                self.available_neighbors.append(neighbor)

            #Update desired positions
            self.ri[0,0] = -np.sum(rel_pos_matrix[:,0])
            self.ri[0,1] = -np.sum(rel_pos_matrix[:,1])

            self.update_references()

            return True

        except rospy.ServiceException as e:
            print("[rendezvous robot] Service call failed: "+str(e))
            return False  


    def update_references(self):
        if not self.neighbor_positions:
            return  # If no neighbors' positions are available, do nothing

        # Consensus algorithm with weights
        for i, (x, y) in self.neighbor_positions.items():
            self.target_pose.position.x += self.weight * (x - self.target_pose.position.x)
            self.target_pose.position.y += self.weight * (y - self.target_pose.position.y)

        self.target_pose.position.x += self.weight * self.ri[0, 0]
        self.target_pose.position.y += self.weight * self.ri[0, 1]


    def compute_neighbor_position(self, pos, distance, bearing):
        x, y, yaw = pos

        neighbor_x = x + distance * math.cos(bearing + yaw)
        neighbor_y = y + distance * math.sin(bearing + yaw)

        return neighbor_x, neighbor_y

    #High frequency control callback to send velocity commands
    def control_update(self, event):

        if self.path_received is False: return

        if self.neighbor_search() is False: return

        # Calculate the heading to the target
        self.error.pose.position.x = self.target_pose.position.x - self.pose.position.x
        self.error.pose.position.y = self.target_pose.position.y - self.pose.position.y

        self.rho = math.sqrt(self.error.pose.position.x*self.error.pose.position.x + self.error.pose.position.y*self.error.pose.position.y)

        target_angle = math.atan2(self.error.pose.position.y, self.error.pose.position.x)

        # Get current robot orientation in Euler angles
        orientation_q = self.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calculate angular difference
        self.alpha = target_angle - yaw

        if (self.alpha < -math.pi): self.alpha = 2.0*math.pi - abs(self.alpha)
        if (self.alpha > math.pi): self.alpha = -2.0*math.pi + self.alpha

        self.beta = - self.alpha - yaw
        if (self.beta < -math.pi): beta = 2.0*math.pi - abs(self.beta)
        if (self.beta > math.pi): beta = -2.0*math.pi + self.beta

        # Normalize the angular difference
        self.error.pose.orientation.z= self.alpha

        #Publish error for debug
        self.error.header.stamp = rospy.Time.now()
        self.error_publisher.publish(self.error)

        self.target_reached = self.rho < 0.2

        if self.target_reached: return

        self.command_velocity()

    def command_velocity(self):
        
        # Calculate linear velocity in x
        self.velocity.linear.x = self.Kp_v * self.rho

        # Calculate angular velocity in z
        self.velocity.angular.z = self.Kp_alpha * self.alpha + self.Kp_beta * self.beta

        # Publish the velocity command
        self.velocity_publisher.publish(self.velocity)



if __name__ == "__main__":
    
    robot_node = FormationControl()
    rospy.spin()