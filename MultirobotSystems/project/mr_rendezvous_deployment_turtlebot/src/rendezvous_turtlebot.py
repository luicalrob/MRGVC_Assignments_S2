#!/usr/bin/env python
import random
import math
import rospy
from mr_rendezvous_deployment_turtlebot.msg import queue_position_plot, range_bearing
from mr_rendezvous_deployment_turtlebot.srv import gossip_update, gossip_updateResponse
from mr_rendezvous_deployment_turtlebot.srv import sensor_measurement
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class RendezvousTurtlebot:

    def __init__(self):
        rospy.init_node("robot")

        self.Kp_v = 0.1
        self.Kp_alpha = 2.0
        self.Kp_beta = -0.1
        self.rho = 0.0
        self.alpha = 0.0
        self.beta = 0.0

        self.pose = Pose()
        self.target_pose = Pose()
        self.error = PoseStamped()
        self.target_reached = False
        self.velocity = Twist()

        rospy.sleep(1.0)
        #State subscriber
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.position_cb)
        # Create publishers
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        #This is for debug
        self.error_publisher = rospy.Publisher("debug/error", PoseStamped, queue_size=10)


        #Wait for things to be initialized
        rospy.sleep(2.0)

        # Read parameters
        if self.read_params():
            rospy.loginfo("[rendezvous robot] Succesfully launched robot {} with position {} and neighbors {}".format(
                self.robot_id, self.pose.position, self.neighbors))
            
         # Create server
        self.gossip_update_server = rospy.Service(
            '/gossip_update_'+str(self.robot_id), gossip_update, self.gossip_update_cb)
        
        # Create timer to request gossip update every t_local cycles
        self.gossip_timer = rospy.Timer(rospy.Duration(
            self.t_local), self.request_gossip_update)
        
        # Create timer to request gossip update every t_local cycles
        self.control_timer = rospy.Timer(rospy.Duration(
            0.1), self.control_update)

    def read_params(self):
        try:
            self.inter_distance_x = rospy.get_param("/inter_distance_x")
            self.inter_distance_y = rospy.get_param("/inter_distance_y")
            self.dynamic_neighbors = rospy.get_param("/dynamic_neighbors")
            self.max_range = rospy.get_param("/sensor_range")          

            self.formation = rospy.get_param("/formation")
            self.robot_id = int(rospy.get_param("~robot_id"))
            self.neighbors = rospy.get_param("~neighbors")            
            self.neighbors = [int(x)
                                for x in self.neighbors.split()]
            
            self.available_neighbors = []
            
            for neighbor in self.neighbors:
                if neighbor < self.robot_id:
                    self.available_neighbors.append(neighbor)

            self.t_local = float(rospy.get_param("~t_local"))

            if self.formation != 'rendezvous' and self.formation != 'line' and self.formation != 'other': 
                print("Formation unrecognized")
                return False

            return True
        except rospy.ServiceException as e:
            print("Parameters not set: "+str(e))
            return False
        
    def position_cb(self, msg):
        
        self.pose = msg.pose.pose
    

    def gossip_update_cb(self, req):


        res = gossip_updateResponse()

                
        res.x_resp = (req.x_req + self.pose.position.x) / 2.0
        res.y_resp = (req.y_req + self.pose.position.y) / 2.0

        
        # #Uncomment this to only update gossip after reaching next position
        # if not self.target_reached:
        #     #rospy.loginfo(" [rendezvous robot] Robot {} has not reached the target position yet.".format(self.robot_id))
        #     res.success = False
        #     return res  # Return an empty response if target not reached
        
        self.target_pose.position.x = res.x_resp
        self.target_pose.position.y = res.y_resp
        res.success = True

        if self.formation == 'line':
            self.target_pose.position.x = self.target_pose.position.x + self.inter_distance_x * (self.robot_id-1)
            self.target_pose.position.y = self.target_pose.position.y + self.inter_distance_y * (self.robot_id-1)

        rospy.loginfo(" [rendezvous robot] Target position of robot {} updated: {}".format(
                self.robot_id, self.target_pose.position))
        
        return res

    
    #Low-frequency callback to request gossip updates periodically and change target positions for each robot
    def request_gossip_update(self, event): 
                
        #request measurements and find neighbors in sensor range
        if(self.dynamic_neighbors):
            
            rospy.wait_for_service("/sensor_measurements_server")

            try:

                service_neighbors_update = rospy.ServiceProxy(
                "/sensor_measurements_server", sensor_measurement)
                
                res = service_neighbors_update(self.robot_id, self.max_range)
                self.available_neighbors = []

                for measurement in res.measurements:
                    neighbor = measurement.robot_id
                    distance = measurement.range
                    bearing = measurement.bearing
                    rospy.loginfo("[Robot {}] found neighbor {} at distance {}, angle {}".format(self.robot_id, neighbor, distance, bearing))
                    self.available_neighbors.append(neighbor)

            except rospy.ServiceException as e:
                print("[rendezvous robot] Service call failed: "+str(e))  
        
            
        #select one of the neighbors
        if self.available_neighbors:
            target_id = random.choice(self.available_neighbors)
        else:
            rospy.loginfo("[Robot {}] Could not find any neighbors".format(self.robot_id))
            return

        rospy.wait_for_service("/gossip_update_"+str(target_id))
        
        try:

            service_gossip_update = rospy.ServiceProxy(
                "/gossip_update_"+str(target_id), gossip_update)
            
            res = service_gossip_update( self.robot_id,
                self.pose.position.x, self.pose.position.y)
            
            if(res.success is False): return

            self.target_pose.position.x = res.x_resp 
            self.target_pose.position.y = res.y_resp

            if self.formation == 'line':
                self.target_pose.position.x  = self.target_pose.position.x + self.inter_distance_x * (self.robot_id-1)
                self.target_pose.position.y  = self.target_pose.position.y + self.inter_distance_y * (self.robot_id-1)
            
            rospy.loginfo("[rendezvous robot] Target position of robot {} updated: {}".format(
                self.robot_id, self.pose.position))

        except rospy.ServiceException as e:
            print("[rendezvous robot] Service call failed: "+str(e))

    #High frequency control callback to send velocity commands
    def control_update(self, event):

        
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
    
    robot_node = RendezvousTurtlebot()
    rospy.spin()