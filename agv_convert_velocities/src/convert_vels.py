#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ConverVelocities():
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.right_wheel_pub = rospy.Publisher('/agv/right_wheel_controller/command', Float64, queue_size=1)
        self.left_wheel_pub = rospy.Publisher('/agv/left_wheel_controller/command', Float64, queue_size=1)
        self.twist_vels = Twist()
        self.vr = Float64()
        self.vl = Float64()
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        self.twist_vels = msg
    
    def pub_wheel_vels(self):
        while not rospy.is_shutdown():
            self.convert_velocities()
            self.right_wheel_pub.publish(self.vr)
            self.left_wheel_pub.publish(self.vl)
            self.rate.sleep()

    def convert_velocities(self):
        L = 0.61
        R = 0.08
        self.vr = ((2*self.twist_vels.linear.x) + (self.twist_vels.angular.z*L))/(2*R)
        self.vl = ((2*self.twist_vels.linear.x) - (self.twist_vels.angular.z*L))/(2*R)

if __name__ == '__main__':
    rospy.init_node('convert_vels_node', anonymous=True)
    cv = ConverVelocities()
    cv.pub_wheel_vels()  