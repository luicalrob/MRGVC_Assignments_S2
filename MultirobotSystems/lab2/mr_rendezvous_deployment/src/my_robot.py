#!/usr/bin/env python
import random

import rospy
from mr_rendezvous_deployment.msg import queue_position_plot
from mr_rendezvous_deployment.srv import gossip_update, gossip_updateResponse


class MyRobot:

    def __init__(self):
        rospy.init_node("robot")

        # Class attributes 
        self.robot_id = 0
        self.position = [0,0]
        self.neighbors = []

        self.read_parameters()

        self.gossip_update_server = rospy.Service('gossip_update_'+str(self.robot_id), gossip_update, self.handle_gossip_update)
        self.position_publisher = rospy.Publisher('queue_position_plot', queue_position_plot, queue_size=10)

        position_update = queue_position_plot()
        position_update.x = self.position[0]
        position_update.y = self.position[1]
        position_update.robot_id = self.robot_id

        rospy.sleep(0.5)
        self.position_publisher.publish(position_update)
        rospy.sleep(1.5)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.request_gossip_update)

    def read_parameters(self):
        try:
            self.inter_distance_x = rospy.get_param("/inter_distance_x")
            self.inter_distance_y = rospy.get_param("/inter_distance_y")

            self.robot_id = int(rospy.get_param("~robot_id"))

            self.position = rospy.get_param("~position")
            if type(self.position) is str:
                self.position = [float(x)
                                 for x in self.position.split()]

            self.neighbors = rospy.get_param("~neightbors")
            
            return True
        except rospy.ServiceException as e:
            print("Parameters not set: "+str(e))
            return False

    def handle_gossip_update(self, req):
        res = gossip_updateResponse()
        res.x_resp = (req.x_req + self.position[0]) / 2
        res.y_resp = (req.y_req + self.position[1]) / 2

        self.position[0] = res.x_resp
        self.position[1] = res.y_resp
        
        position_update = queue_position_plot()
        position_update.x = self.position[0] 
        position_update.y = self.position[1]
        position_update.robot_id = self.robot_id
        self.position_publisher.publish(position_update)
        return res

    def request_gossip_update(self, event):

        rospy.wait_for_service('gossip_update_'+str(target_id))
        
        service_gossip_update = rospy.ServiceProxy('gossip_update_'+str(target_id), gossip_update)
        res = service_gossip_update( self.robot_id, self.position[0], self.position[1])
        self.position[0] = res.x_resp 
        self.position[1] = res.y_resp
            
        position_update = queue_position_plot()
            
        position_update.x = self.position[0] 
        position_update.y = self.position[1]
        position_update.robot_id = self.robot_id
        
        self.position_publisher.publish(position_update)



if __name__ == "__main__":
    robot_node = MyRobot()
    rospy.spin()