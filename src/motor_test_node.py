#!/usr/bin/python
import rospy
from terrain_treadmill.msg import MotorVelocity

from dynamic_reconfigure.server import Server
from terrain_treadmill.cfg import motor_testConfig


class motor_test():
	def __init__(self):
		self.srv = Server(motor_testConfig, self.config_callback)

		self.got_new_message = False

		# Messages 
		self.velocity = MotorVelocity()
		self.velocity.w1 = 0.0
		self.velocity.w2 = 0.0
		self.velocity.w3 = 0.0
		self.w1 = 0.0
		self.w2 = 0.0
		self.w3 = 0.0
		self.freq = 50

		# Create subscribers and publishers.
		pub_state = rospy.Publisher("/arduino/desired_velocity_counts", MotorVelocity, queue_size=10)

		r = rospy.Rate(self.freq)
		while not rospy.is_shutdown():
			if self.got_new_message:
				self.velocity.w1 = self.w1
				self.velocity.w2 = self.w2
				self.velocity.w3 = self.w3
				self.got_new_message = False


			# self.velocity.header.stamp = rospy.Time.now()
			pub_state.publish(self.velocity)
			r.sleep()

	def config_callback(self, config, level):
		self.got_new_message = True
		self.w1 = config["w1"]
		self.w2 = config["w2"]
		self.w3 = config["w3"]
		return config
  
if __name__ == '__main__':
	
	rospy.init_node('motor_test_node')
	
	try:
		motor_test()
	except rospy.ROSInterruptException: pass