#!/usr/bin/python
import rospy
import math
from terrain_treadmill.msg import TagState
from terrain_treadmill.msg import ControlEffort

from dynamic_reconfigure.server import Server
from terrain_treadmill.cfg import PIDConfig

class controller():
    def __init__(self):
        self.srv = Server(PIDConfig, self.config_callback)
        self.On = True
        self.got_new_msg = False

        self.controlEffort = ControlEffort() # Control Effort in X, Y, Theta Directions
        self.zeroEffort = ControlEffort()
        self.zeroEffort.Ux = 0
        self.zeroEffort.Uy = 0
        self.zeroEffort.Ut = 0

        self.Ex = 0.0       # Proportional Error
        self.Ey = 0.0
        self.Et = 0.0

        self.Exold = 0.0    # Previous Proportional Error
        self.Eyold = 0.0
        self.Etold = 0.0

        self.Exint = 0.0    # Integral Error
        self.Eyint = 0.0
        self.Etint = 0.0

        self.Exdot = 0.0    # Derivative Error
        self.Eydot = 0.0
        self.Etdot = 0.0

        self.Kp = 0.0       # Proportional Gain (Set using dynamic-reconfigure)
        self.Ki = 0.0       # Integral Gain
        self.Kd = 0.0       # Derivative Gain

        # Create subscribers and publishers.
        sub = rospy.Subscriber("/kalman_state", TagState, self.PID_Controller)
        pub = rospy.Publisher("/control_effort", ControlEffort,queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
            	if self.On:
                	pub.publish(self.controlEffort)
                else:
                	pub.publish(self.zeroEffort)
            	self.got_new_msg = False

    def PID_Controller(self, msg):
       	# Calculate Error
        self.Ex = msg.x
        self.Ey = msg.y
        # self.Et = msg.theta

        # Calculate Integral Error
        self.Exint = self.Exint + self.Ex
        self.Eyint = self.Eyint + self.Ey
        # self.Etint = self.Etint + self.Et

        # Calculate Derivative Error
        self.Exdot = self.Ex - self.Exold
        self.Eydot = self.Ey - self.Eyold
        # self.Etdot = self.Et - self.Etold

        # Update Previous Error
        self.Exold = self.Ex
        self.Eyold = self.Ey
        # self.Etold = self.Et

        # Calculate Control Effort
        self.controlEffort.Ux = self.Kp*self.Ex + self.Ki*self.Exint + self.Kd*self.Exdot
        self.controlEffort.Uy = self.Kp*self.Ey + self.Ki*self.Eyint + self.Kd*self.Eydot
        self.controlEffort.Ut = 0.0 #self.Kp*self.Et + self.Ki*self.Etint + self.Kd*self.Etdot
        self.controlEffort.header.stamp = rospy.Time.now()

        # Cutoff control within 3 mm
        if -0.003 < self.Ex < 0.003:
            self.Ux = 0.0
        if -0.003 < self.Ey < 0.003:
            self.Uy = 0.0

        self.got_new_msg = True


    def config_callback(self, config, level):
        self.Kp = config["Kp"]
        self.Ki = config["Ki"]
        self.Kd = config["Kd"]
        self.On = config["On"]
        return config

if __name__ == '__main__':
    rospy.init_node('controller_node')
    try:
        controller()
    except rospy.ROSInterruptException: pass
