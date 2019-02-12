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
from sklearn.cluster import MeanShift
from tf.transformations import quaternion_from_euler
from collections import Counter
from time import time

class Repub:
    def __init__(self):
        self.last_time = time()
        #  This constructor sets up class variables and pubs/subs
        self._feature_serv = rospy.Publisher('/rescan', LaserScan, queue_size=1)

        rospy.Subscriber('/scan', LaserScan, self.sensorCallback, queue_size=1)

    def sensorCallback(self, data):
        if time() - self.last_time > 10:
            print "Updating the message"
            self._feature_serv.publish(data)
            self.last_time = time()
        else:
            print "Doing nothing"


if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('republisher')
    sensor = Repub()

    while not rospy.is_shutdown():
        pass




