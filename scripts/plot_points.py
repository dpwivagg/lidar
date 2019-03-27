#!/usr/bin/env python

"""
To run this node:
Make sure the roscore is running
Run the following command:

    rosrun lidar_slam plot_points.py

"""

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from sklearn.cluster import MeanShift
from tf.transformations import quaternion_from_euler
from collections import Counter

def scanCallback(data):
    print "Occurrences of 0: %d" % data.ranges.count(0.0)
    print "Total number of data points: %d" % len(data.ranges)
    plt.hist(data.ranges)
    plt.show()


if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('plot_points')
    rospy.Subscriber('/scan', LaserScan, scanCallback, queue_size=1)

    while not rospy.is_shutdown():
        pass
