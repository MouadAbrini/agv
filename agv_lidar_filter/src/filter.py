#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoFilter:
    def __init__(self):

        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("filtered_scan", LaserScan, queue_size=10)

    def callback(self, data):

        newdata = data
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)

        for x in range(70,94):
            newdata.ranges[x]=0
        for x in range(266,291):
            newdata.ranges[x]=0
        for x in range(429,454):
            newdata.ranges[x]=0
        for x in range(626,650):
            newdata.ranges[x]=0

        self.pub.publish(newdata)


if __name__ == '__main__':

    # Initialize
    rospy.init_node('LidarFilter', anonymous=False)
    lidar = DoFilter()

    rospy.spin()