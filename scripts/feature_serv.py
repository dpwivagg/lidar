#!/usr/bin/env python

"""
To run this node:
Make sure the roscore is running
Run the following command:

    rosrun lidar_slam feature_serv.py

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
import sys
import keyboard

class Repub:
    def __init__(self):
        self.last_time = time()
	self.current_msg = None
        #  This constructor sets up class variables and pubs/subs
        self._feature_serv = rospy.Publisher('/rescan', LaserScan, queue_size=1)

        rospy.Subscriber('/scan', LaserScan, self.sensorCallback, queue_size=1)

    def sensorCallback(self, data):
	self.current_msg = data
	return
        if time() - self.last_time > 10:
            print "\nRepublishing one message from topic /scan to topic /rescan"
            self._feature_serv.publish(data)
            self.last_time = time()
        else:
            sys.stdout.write("\rDoing nothing for %i more seconds" % (10 - (time() - self.last_time)))
            sys.stdout.flush()


    def push(self):
	self._feature_serv.publish(self.current_msg)


if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('republisher')
    sensor = Repub()

    while not rospy.is_shutdown():
        raw_input('Press enter to republish the latest message (Ctrl+C and enter to quit)')
	sensor.push()




