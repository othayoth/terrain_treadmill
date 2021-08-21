#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from visualization_msgs.msg import Marker

class tag_marker():
	def __init__(self):
		self.got_new_msg = False

		# Create subscribers
		sub_pose = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.callback)
		# Create publishers
		marker_pub = rospy.Publisher('tag_marker',Marker,queue_size=1)

		# Initialize Marker
		self.i = 0
		self.marker = Marker()
		self.marker.header.frame_id = "base_link"
		self.marker.ns = "tag_2D"
		self.marker.id = self.i
		self.marker.type = Marker.SPHERE
		self.marker.action = Marker.ADD
		self.marker.scale.x = .05
		self.marker.scale.y = .05
		self.marker.scale.z = .05
		self.marker.color.r = 0.0
		self.marker.color.g = 0.0
		self.marker.color.b = 1.0
		self.marker.color.a = 1.0
		self.marker.pose.position.z = 0.0
		self.marker.pose.orientation.x = 0.0
		self.marker.pose.orientation.y = 0.0
		self.marker.pose.orientation.z = 0.0
		self.marker.pose.orientation.w = 0.0
		self.marker.lifetime = rospy.Duration(0.5)

		while not rospy.is_shutdown():
			if self.got_new_msg:
				marker_pub.publish(self.marker)
				self.got_new_msg = False

	def callback(self, msg):
		self.got_new_msg = True

		self.marker.header.stamp = rospy.Time.now()
		self.marker.pose.position.x = msg.pose.position.x
		self.marker.pose.position.y = msg.pose.position.y
		self.i = self.i + 1
		




if __name__ == "__main__":
	rospy.init_node('tag_marker')

	try:
		tag_marker()
	except rospy.ROSInterruptException: pass