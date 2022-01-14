#!/usr/bin/python
import rospy
import math
from terrain_treadmill.msg import ControlEffort
from terrain_treadmill.msg import MotorVelocity

class forward_kinematics():
    def __init__(self):
        self.got_new_msg = False

        self.velocity_msg = MotorVelocity() # Desired Motor Velocities

        self.phi = math.radians(45.0)   # Motor Tilt Angle
        self.Rball = .29845             # Radius of the Ball [meters]
        self.Rmotor = .1                # Radius of Motor Wheels [meters]
        self.Hz = 200                   # Arduino Motor Control Freq
        self.counts = 1633.25           # Encoder Counts per Rev

        self.A = math.sqrt(3.0)/2.0 # Useful Constants
        self.B = math.cos(self.phi)
        self.C = -self.Rball*math.sin(self.phi)
        self.D = 1.0/self.Hz * self.counts / (2*math.pi)

        # Create subscribers and publishers.
        sub_vel = rospy.Subscriber("/control_effort", ControlEffort, self.kinematics_cb)
        pub_vel = rospy.Publisher("/arduino/desired_velocity_counts", MotorVelocity,queue_size=10)

        # Main while loop.
        rospy.wait_for_message("/control_effort",ControlEffort)
        self.timer = rospy.Time.now()
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_vel.publish(self.velocity_msg)
                self.got_new_msg = False
                self.timer = rospy.Time.now()

        # Turn off motors on exit
        self.velocity_msg.w1 = 0
        self.velocity_msg.w2 = 0
        self.velocity_msg.w3 = 0
        pub_vel.publish(self.velocity_msg)

    # Callback triggered when control effort information is recieved
    def kinematics_cb(self, msg):
    	self.kinematics(msg)
    	self.got_new_msg = True

    # Convert 2D velocity to desired motor velocity
    def kinematics(self, msg):
        self.velocity_msg.w1 =  (-msg.Uy*self.B+ self.C*msg.Ut)/self.Rmotor
        self.velocity_msg.w2 =  ((self.A*msg.Ux+0.5*msg.Uy+ self.C*msg.Ut)*self.B)/self.Rmotor
        self.velocity_msg.w3 = ((-self.A*msg.Ux+0.5*msg.Uy+ self.C*msg.Ut)*self.B)/self.Rmotor
        self.CountsPerInterrupt()

    # Convert desired motor velocity from rad/s -> counts/interrupt
    def CountsPerInterrupt(self):
        self.velocity_msg.w1 = self.velocity_msg.w1 * self.D
        self.velocity_msg.w2 = self.velocity_msg.w2 * self.D
        self.velocity_msg.w3 = self.velocity_msg.w3 * self.D
        # self.velocity_msg.header.stamp = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node')
    try:
        forward_kinematics()
    except rospy.ROSInterruptException: pass
