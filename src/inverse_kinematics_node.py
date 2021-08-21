#!/usr/bin/python
import rospy
import math
from terrain_treadmill.msg import BallVelocity
from terrain_treadmill.msg import MotorVelocity

class inverse_kinematics():
    def __init__(self):
        self.got_new_msg = False
        self.motor_vel = MotorVelocity()
        self.ball_vel = BallVelocity()
        self.phi = math.radians(45.0)
        self.Rball = .2413 # meters
        self.Rmotor = .1
        self.Kz = 0.0
        self.Hz = rospy.get_param('~Hz',default=200);
        self.counts = rospy.get_param('Counts',default=1633.25)

        # Create subscribers and publishers.
        sub_vel = rospy.Subscriber("/arduino/actual_velocity_counts", MotorVelocity, self.kinematics_cb)
        pub_vel = rospy.Publisher("/arduino/actual_velocity", BallVelocity,queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_vel.publish(self.velocity_msg)
                self.got_new_msg = False

    # Callback triggered when velocity information is recieved
    def kinematics_cb(self, msg):
    	self.toRadPerSec(msg)
    	self.got_new_msg = True

    # Convert actual motor velocity from counts/interrupt -> rad/s
    def toRadPerSec(self, msg):
        self.motor_vel.w1 = msg.w1 * self.Hz / self.counts * (2*math.pi)
        self.motor_vel.w2 = msg.w2 * self.Hz / self.counts * (2*math.pi)
        self.motor_vel.w3 = msg.w3 * self.Hz / self.counts * (2*math.pi)
        self.kinematics()

    # Convert actual motor velocity into actual ball velocity
    def kinematics(self):
    	self.ball_vel.xdot = (math.sqrt(3)*(self.motor_vel.w2-self.motor_vel.w3))/(3*math.cos(self.phi))
    	self.ball_vel.ydot = (self.motor_vel.w2 - 2*self.motor_vel.w1 + self.motor_vel.w3)/(3*math.cos(self.phi))
    	self.ball_vel.wdot = -(self.motor_vel.w1 + self.motor_vel.w2 + self.motor_vel.w3)/(3*self.Rball*math.sin(self.phi))
    	self.ball_vel.header.stamp = rospy.Time.now()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('inverse_kinematics_node')
    try:
        inverse_kinematics()
    except rospy.ROSInterruptException: pass