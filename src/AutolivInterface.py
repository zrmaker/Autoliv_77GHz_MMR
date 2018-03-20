#!/usr/bin/env python
# created by Fuheng Deng on 10/26/2017
# updated by Renyuan Zhang on 3/19/2018

import rospy
from autoliv.msg import Targets
from visualization_msgs.msg import Marker
#import visualization_msgs.msg import MarkerArray
#from dynamic_reconfigure.server import Server
#from geometry_msgs.msg import Point32

class AutolivInterface:
    def __init__(self):
        self.frame_id = 'autoliv'
	self.sub_radar_track = rospy.Subscriber('/autoliv/targets', Targets, self.recv_radar_track)
	self.pub_radar_marker = rospy.Publisher('/autoliv/marker', Marker, queue_size=10)
    
    def recv_radar_track(self, track):
	marker = Marker()
	marker.header.frame_id = self.frame_id
	marker.header.stamp = rospy.Time.now()
	marker.id = track.track_id
	marker.action = Marker.ADD
	marker.lifetime = rospy.Duration(.04)
	marker.type = Marker.SHPERE
	
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = 0	

	marker.scale.x = 1
	marker.scale.y = 1
	marker.scale.z = 1

	marker.color.a = 1.0
	marker.color.r = 1.0 / 255
	marker.color.g = 245.0 / 255
	marker.color.b = 3.0 / 255
	self.marker_publiser.publish(marker)

if __name__ == "__main__":
    rospy.init_node('autoliv_radar_interface')
    AutolivInterface()
    rospy.spin()

