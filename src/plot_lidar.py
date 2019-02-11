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
from std_msgs.msg import Float32MultiArray
# TODO: Add an import for the correct message type or else we cannot use it

def euclid_distance(x, xi):
    return np.sqrt(np.sum((x - xi)**2))

def neighbourhood_points(X, x_centroid, distance = 5):
    eligible_X = []
    for x in X:
        distance_between = euclid_distance(x, x_centroid)
        # print('Evaluating: [%s vs %s] yield dist=%.2f' % (x, x_centroid, distance_between))
        if distance_between <= distance:
            eligible_X.append(x)
    return eligible_X

def gaussian_kernel(distance, bandwidth):
    val = (1/(bandwidth*math.sqrt(2*math.pi))) * np.exp(-0.5*((distance / bandwidth))**2)
    return val

class Lidar:
    def __init__(self):

        #  This constructor sets up class variables and pubs/subs
        self._corner_pub = rospy.Publisher('/corners', Float32MultiArray, queue_size=1)

        rospy.Subscriber('/scan', LaserScan, self.sensorCallback, queue_size=1)

    def sensorCallback(self, data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        inc = data.angle_increment
        ranges = np.asarray(data.ranges)

        # theta = np.arange(angle_min, angle_max, inc)
        theta = np.linspace(angle_min, angle_max, ranges.size)
        # Based on inspection, using a linspace is approximately good enough to get the angles

        msg = Float32MultiArray()
        msg.data = 1

        self._corner_pub.publish(msg)


if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('sensor_read')
    sensor = Lidar()

    while not rospy.is_shutdown():
        pass




