#!/usr/bin/env python
import random
import math
import rospy
from mr_rendezvous_deployment.msg import queue_position_plot
from mr_rendezvous_deployment.srv import gossip_update, gossip_updateResponse
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class RendezvousTurtlebot:

    def __init__(self):
        rospy.init_node("robot")

        self.Kp_v = 0.1
        self.Kp_w = 0.25

        self.pose = Pose()
        self.target_pose = Pose()
        self.error = PoseStamped()
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
            0.05), self.control_update)

    def read_params(self):
        try:
            self.inter_distance_x = rospy.get_param("/inter_distance_x")
            self.inter_distance_y = rospy.get_param("/inter_distance_y")
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

        self.target_pose.position.x = res.x_resp
        self.target_pose.position.y = res.y_resp

        rospy.loginfo(" [rendezvous robot] Target position of robot {} updated: {}".format(
                self.robot_id, self.target_pose.position))
        

        if self.formation == 'line':
            self.target_pose.position.x = self.target_pose.position.x + self.inter_distance_x * self.robot_id
            self.target_pose.position.y = self.target_pose.position.y + self.inter_distance_y * self.robot_id
        
        return res

    
    #Low-frequency callback to request gossip updates periodically and change target positions for each robot
    def request_gossip_update(self, event): 
                
        if self.available_neighbors is not None:
            target_id = random.choice(self.available_neighbors)
        else:
            return

        rospy.wait_for_service("/gossip_update_"+str(target_id))
        
        try:

            service_gossip_update = rospy.ServiceProxy(
                "/gossip_update_"+str(target_id), gossip_update)
            
            res = service_gossip_update( self.robot_id,
                self.pose.position.x, self.pose.position.y)
            
            self.target_pose.position.x = res.x_resp 
            self.target_pose.position.y = res.y_resp
            
            rospy.loginfo("[rendezvous robot] Target position of robot {} updated: {}".format(
                self.robot_id, self.pose.position))

            if self.formation == 'line':
                self.target_pose.position.x  = self.target_pose.position.x + self.inter_distance_x * self.robot_id
                self.target_pose.position.y  = self.target_pose.position.y + self.inter_distance_y * self.robot_id
            

        except rospy.ServiceException as e:
            print("[rendezvous robot] Service call failed: "+str(e))

    #High frequency control callback to send velocity commands
    def control_update(self, event):

        # Calculate the heading to the target
        self.error.pose.position.x = self.target_pose.position.x - self.pose.position.x
        self.error.pose.position.y = self.target_pose.position.y - self.pose.position.y

        d = math.sqrt(self.error.pose.position.x*self.error.pose.position.x + self.error.pose.position.y*self.error.pose.position.y)

        target_angle = math.atan2(self.error.pose.position.y, self.error.pose.position.x)

        # Get current robot orientation in Euler angles
        orientation_q = self.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calculate angular difference
        self.error.pose.orientation.z = target_angle - yaw

        # Normalize the angular difference
        self.error.pose.orientation.z= math.atan2(math.sin(self.error.pose.orientation.z), math.cos(self.error.pose.orientation.z))

        if (d < 0.1 and abs(self.error.pose.orientation.z) < 0.1): return

        #Publish error for debug
        self.error.header.stamp = rospy.Time.now()
        self.error_publisher.publish(self.error)

        self.command_velocity()

    def command_velocity(self):
        
        # Calculate linear velocity in x
        self.velocity.linear.x = self.Kp_v * self.error.pose.position.x

        # Calculate angular velocity in z
        self.velocity.angular.z = self.Kp_w * self.error.pose.orientation.z

        # Publish the velocity command
        self.velocity_publisher.publish(self.velocity)




if __name__ == "__main__":
    
    robot_node = RendezvousTurtlebot()
    rospy.spin()