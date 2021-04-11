#! /usr/bin/env python

import rospy
import math
#from __future__ import division
from math import pi, sin, cos
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Imu, JointState
from agv_gazebo.msg import encoders
from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:
    def __init__(self):
        self.ticks_sub = rospy.Subscriber('/agv/joint_states',JointState, self.getTicks)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.odom_broadcaster = TransformBroadcaster()
        self.odom = Odometry()
        self.rate = rospy.Rate(50)
        self.lastLeftTicks = 0
        self.lastRightTicks = 0
        self.currentLeftTicks = 0
        self.currentRightTicks = 0
        self.last_time = rospy.Time.now()
        self.L = 0.635
        self.R = 0.08
        self.N = 360
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.xi = 0
        self.yi = 0
        self.zi = 0
        self.wi = 0

        self.last_th=0

    def getTicks(self, msg):
        self.currentLeftTicks = msg.position[0]
        self.currentRightTicks = msg.position[1]

    def getOrientation(self, msg):
        self.yi = msg.orientation.x
        self.zi = msg.orientation.y
        self.wi = msg.orientation.z
        self.xi = msg.orientation.w


    
    def updatePose(self):

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            d_l = (self.currentLeftTicks - self.lastLeftTicks)*self.R
            d_r = (self.currentRightTicks - self.lastRightTicks)*self.R
            #d_l = 2 * pi * self.R * delta_l / self.N
            #d_r = 2 * pi * self.R * delta_r / self.N

            self.lastLeftTicks = self.currentLeftTicks
            self.lastRightTicks = self.currentRightTicks

            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - self.last_time).to_sec()


            delta_s = (d_r + d_l) / 2
            th = (d_r - d_l) / self.L
            #th = math.atan2(self.yi*self.zi+self.xi*self.wi,0.5-self.zi**2-self.wi**2)
            delta_th = th - self.last_th

            delta_x = delta_s * cos(self.th)
            delta_y = delta_s * sin(self.th)

            self.x += delta_x
            self.y += delta_y
            self.th += th
            
            

            if not dt:
                v=0
                w=0
            else:
                v = delta_s/dt
                w = th/dt

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
            """rospy.loginfo(odom_quat)
            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )"""
            

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
            odom.pose.covariance = [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

            # set the velocity
            odom.child_frame_id = "base_link"
            if not dt:
                odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            else:
                odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            # publish the message
            self.odom_pub.publish(odom)

            self.last_time = current_time
            self.last_th = th
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pub_odom')
    oc = OdometryClass()
    oc.updatePose()