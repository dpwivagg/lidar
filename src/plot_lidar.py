#!/usr/bin/env python

"""
To run this node:
Make sure the roscore is running
Run the following command:

    rosrun lidar_slam plot_lidar.py

"""

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray


class Lidar:
    def __init__(self):

        #  This constructor sets up class variables and pubs/subs
        self._corner_pub = rospy.Publisher('/corners', PoseArray, queue_size=1)

        rospy.Subscriber('/scan', LaserScan, self.sensorCallback, queue_size=1)

    def sensorCallback(self, data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        inc = data.angle_increment
        ranges = np.asarray(data.ranges)

        # theta = np.arange(angle_min, angle_max, inc)
        theta = np.linspace(angle_min, angle_max, ranges.size)
        # Based on inspection, using a linspace is approximately good enough to get the angles

        point = Pose()
        point.position.x = 2
        point.position.y = 2
        point.position.z = 0
        point.orientation.w = 0
        point.orientation.x = 1
        point.orientation.y = 0
        point.orientation.z = 0
        poses = [point]
        msg = PoseArray()
        msg.poses = poses
        msg.header.frame_id = 'laser_frame'
        self._corner_pub.publish(msg)


if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('sensor_read')
    sensor = Lidar()

    while not rospy.is_shutdown():
        pass




