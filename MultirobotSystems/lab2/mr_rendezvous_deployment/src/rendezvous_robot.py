#!/usr/bin/env python
import random

import rospy
from mr_rendezvous_deployment.msg import queue_position_plot
from mr_rendezvous_deployment.srv import gossip_update, gossip_updateResponse
from geometry_msgs.msg import PointStamped

class NaiveRobot:

    def __init__(self):
        rospy.init_node("robot")

        # Class attributes 
        self.robot_id = 0
        self.t_local = 1/2.0 # default
        self.position = [0,0]
        self.neighbors = []
        self.available_neighbors = []

        # Read parameters
        if self.read_params():
            rospy.loginfo("[rendezvous robot] Succesfully launched robot {} with position {} and neighbors {}".format(
                self.robot_id, self.position, self.neighbors))

        # Create server
        self.gossip_update_server = rospy.Service(
            'gossip_update_'+str(self.robot_id), gossip_update, self.gossip_update_cb)
        
        # Create publisher
        self.position_publisher = rospy.Publisher('mr_rendezvous_deployment/queue_position_plot', queue_position_plot, queue_size=10)
        self.position_publisher_unique = rospy.Publisher("mr_rendezvous_deployment/robot_" + str(self.robot_id) + "/queue_position_plot", PointStamped, queue_size=10)

        #Obtain initial position
        position_update = queue_position_plot()
        position_update.x = self.position[0]
        position_update.y = self.position[1]
        position_update.robot_id = self.robot_id

        position_update_unique = PointStamped()
        position_update_unique.header.stamp = rospy.Time.now()
        position_update_unique.point.x = self.position[0]
        position_update_unique.point.y = self.position[1]
        
        # Publish initial position to neighbours and wait for a bit
        rospy.sleep(0.5)
        self.position_publisher.publish(position_update)
        self.position_publisher_unique.publish(position_update_unique)
        rospy.sleep(2.0)
        
        # Create timer to request gossip update every t_local cycles
        self.timer = rospy.Timer(rospy.Duration(
            self.t_local), self.request_gossip_update)

    def read_params(self):
        try:
            self.inter_distance_x = rospy.get_param("/inter_distance_x")
            self.inter_distance_y = rospy.get_param("/inter_distance_y")
            self.formation = rospy.get_param("~/formation")
            self.robot_id = int(rospy.get_param("~robot_id"))

            self.position = rospy.get_param("~position")
            if type(self.position) is str:
                self.position = [float(x)
                                 for x in self.position.split()]

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

    def gossip_update_cb(self, req):
        
        res = gossip_updateResponse()
        
        res.x_resp = (req.x_req + self.position[0]) / 2.0
        res.y_resp = (req.y_req + self.position[1]) / 2.0

        self.position[0] = res.x_resp
        self.position[1] = res.y_resp

        rospy.loginfo(" [rendezvous robot] Position of robot {} updated: {}".format(
                self.robot_id, self.position))
        
        position_update = queue_position_plot()
        position_update_unique = PointStamped()
        
        if self.formation == 'rendezvous':
            position_update.x = self.position[0] 
            position_update.y = self.position[1]

        elif self.formation == 'line':
            position_update.x = self.position[0] + self.inter_distance_x * self.robot_id
            position_update.y = self.position[1] + self.inter_distance_y * self.robot_id
        
        position_update.robot_id = self.robot_id

        position_update_unique.header.stamp = rospy.Time.now()
        position_update_unique.point.x = position_update.x
        position_update_unique.point.y = position_update.y
        
        self.position_publisher.publish(position_update)
        self.position_publisher_unique.publish(position_update_unique)

        return res

    def request_gossip_update(self, event): #timer callback
        
        if self.available_neighbors is not None:
            target_id = random.choice(self.available_neighbors)
        else:
            return

        rospy.wait_for_service('gossip_update_'+str(target_id))
        
        try:

            service_gossip_update = rospy.ServiceProxy(
                'gossip_update_'+str(target_id), gossip_update)
            
            res = service_gossip_update( self.robot_id,
                self.position[0], self.position[1])
            
            self.position[0] = res.x_resp 
            self.position[1] = res.y_resp
            
            rospy.loginfo("[rendezvous robot] Position of robot {} updated: {}".format(
                self.robot_id, self.position))
            
            position_update = queue_position_plot()
            position_update_unique = PointStamped()

            if self.formation == 'rendezvous':
                position_update.x = self.position[0] 
                position_update.y = self.position[1]

            elif self.formation == 'line':
                position_update.x = self.position[0] + self.inter_distance_x * self.robot_id
                position_update.y = self.position[1] + self.inter_distance_y * self.robot_id
            
            position_update.robot_id = self.robot_id

            position_update_unique.header.stamp = rospy.Time.now()
            position_update_unique.point.x = position_update.x
            position_update_unique.point.y = position_update.y
            
            self.position_publisher.publish(position_update)  
            self.position_publisher_unique.publish(position_update_unique)
        
        except rospy.ServiceException as e:
            print("[rendezvous robot] Service call failed: "+str(e))


if __name__ == "__main__":
    
    robot_node = NaiveRobot()
    rospy.spin()