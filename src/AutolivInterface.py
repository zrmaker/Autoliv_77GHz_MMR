#!/usr/bin/env python
# created by Fuheng Deng on 10/26/2017
# updated by Renyuan Zhang on 3/19/2018

import rospy
from autoliv.msg import Targets
from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from dynamic_reconfigure.server import Server
# from geometry_msgs.msg import Point32

class AutolivInterface:
    def __init__(self):
        self.frame_id = 'autoliv'
        self.sub_radar = rospy.Subscriber('/autoliv/targets', Targets, self.target_to_marker)
        self.pub_radar_marker = rospy.Publisher('/autoliv/markers', Marker, queue_size=10)
    
    def target_to_marker(self, targets):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = targets.target_id
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(.04)
        marker.type = Marker.SPHERE

        marker.pose.position.x = targets.x
        marker.pose.position.y = targets.y
        marker.pose.position.z = 0. 

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 225
        marker.color.b = 225
        self.pub_radar_marker.publish(marker)

if __name__ == "__main__":
    rospy.init_node('autoliv_interface_node')
    AutolivInterface()
    rospy.spin()

