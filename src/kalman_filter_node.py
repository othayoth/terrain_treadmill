#!/usr/bin/python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from terrain_treadmill.msg import TagState

from dynamic_reconfigure.server import Server
from terrain_treadmill.cfg import kalmanConfig


class kalman_filter():
	def __init__(self):
		self.srv = Server(kalmanConfig, self.config_callback)
		self.t_old = rospy.Time.now()
		self.t_new = 0
		self.dt = 0

		self.missed_frames = 0

		self.posex = 0
		self.posey = 0
		self.yaw = 0

		# Kalman Filter Matrices
		self.x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T  # State
		self.P = np.diag([.01, .01, .01, .01, .01, .01])        # State Covariance
		self.F = np.zeros((6,6))                                # Dynamics (placeholder)        
		self.H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],     # Measurement
							[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
							[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
		self.R = np.matrix([[0.00001, 0.0, 0.0],                # Measurement Covariance
							[0.0, 0.00001, 0.0],
							[0.0, 0.0, 0.00001]])
		self.sigmaA = 0.05                                     # Covariance std
		self.Q = np.zeros((6,6))*self.sigmaA                    # Process Covariance (placeholder)
		self.I = np.eye(6)                                      # Identity

		# Messages 
		self.got_new_msg = False
		self.first_msg = True
		self.tag_state = TagState()

		# Create subscribers and publishers.
		sub_pose   = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.callback)
		pub_state = rospy.Publisher("/kalman_state", TagState, queue_size=10)

		r = rospy.Rate(50) # 50 hz
		while not rospy.is_shutdown():
			
			while self.first_msg:
				rospy.wait_for_message("/aruco_single/pose",PoseStamped)

			# Update dt
			self.t_new = rospy.Time.now()
			self.dt = (self.t_new - self.t_old).to_sec()
			self.t_old = self.t_new

			# Calculate dynamics
			self.F = np.matrix([[1.0, 0.0, 0.0, self.dt, 0.0, 0.0],     # Dynamics
							[0.0, 1.0, 0.0, 0.0, self.dt, 0.0],
							[0.0, 0.0, 1.0, 0.0, 0.0, self.dt],
							[0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
							[0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
							[0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
			
			self.Q = np.matrix([                                        # Process Covariance
					[0.25*self.dt**4, 0, 0, 0.5*self.dt**2, 0, 0],
					[0, 0.25*self.dt**4, 0, 0, 0.5*self.dt**2, 0],
					[0, 0, 0.25*self.dt**4, 0, 0, 0.5*self.dt**2],
					[0.5*self.dt**2, 0, 0, self.dt**2, 0, 0],
					[0, 0.5*self.dt**2, 0, 0, self.dt**2, 0],
					[0, 0.5*self.dt**2, 0, 0, 0, self.dt**2]])*self.sigmaA

			# Time Update (Prediction)
			# ========================
			# Project the state ahead
			self.x = self.F*self.x

			# Project the error covariance ahead
			self.P = self.F*self.P*self.F.T + self.Q

			if self.got_new_msg:
				# Measurement Update (Correction)
				# ===============================
				# Compute the Kalman Gain
				S = self.H*self.P*self.H.T + self.R
				K = (self.P*self.H.T) * np.linalg.pinv(S)

				# Update the estimate via z
				Z = np.matrix([[self.posex, self.posey, self.yaw]]).T
				y = Z - (self.H*self.x)
				self.x = self.x + (K*y)

				# Update the error covariance
				self.P = (self.I - (K*self.H)) *self.P

				self.missed_frames = 0
				self.got_new_msg = False
			else:
				self.missed_frames = self.missed_frames + 1

			if self.missed_frames > 15:
				##  Use zero as the measurement
				self.posex = 0
				self.posey = 0
				self.yaw = 0

				# Compute the Kalman Gain
				S = self.H*self.P*self.H.T + self.R
				K = (self.P*self.H.T) * np.linalg.pinv(S)

				# Update the estimate via z
				Z = np.matrix([[self.posex, self.posey, self.yaw]]).T
				y = Z - (self.H*self.x)
				self.x = self.x + (K*y)

				# Update the error covariance
				self.P = (self.I - (K*self.H)) *self.P

			# Write states to message
			self.tag_state.header.stamp = rospy.Time.now()
			self.tag_state.x = self.x.item(0)
			self.tag_state.y = self.x.item(1)
			self.tag_state.theta = self.x.item(2)
			self.tag_state.xdot = self.x.item(3)
			self.tag_state.ydot = self.x.item(4)
			self.tag_state.thetadot = self.x.item(5)
			self.tag_state.dt = self.dt

			pub_state.publish(self.tag_state)

			r.sleep()
	
	# Callback triggered when new pose recieved
	def callback(self, msg):
		# Convert quaternions to Euler angles.
		(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

		self.posex = msg.pose.position.x
		self.posey = msg.pose.position.y
		self.yaw = y

		# Initialize State
		if self.first_msg == True:
			self.x = np.matrix([[self.posex, self.posey, self.yaw, 0.0, 0.0, 0.0]]).T
			self.first_msg = False

		self.got_new_msg = True

	def config_callback(self, config, level):
		self.sigmaA = config["sigmaA"]
		return config

# Main function.    
if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('kalman_filter_node')
	
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		kalman_filter()
	except rospy.ROSInterruptException: pass