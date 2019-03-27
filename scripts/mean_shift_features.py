#!/usr/bin/env python

"""
To run this node:
Make sure the roscore is running
Run the following command:

    rosrun lidar_slam mean_shift_features.py

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

class Lidar:
    def __init__(self):
        self.feature_dict = {}
        self.feature_list_list = []
        #  This constructor sets up class variables and pubs/subs
        self._corner_pub = rospy.Publisher('/corners', PoseArray, queue_size=1)

        rospy.Subscriber('/rescan', LaserScan, self.sensorCallback, queue_size=1)

    def sensorCallback(self, data):
        print "Received new points, checking for feature matches"
        angle_min = data.angle_min
        angle_max = data.angle_max
        inc = data.angle_increment
        ranges = np.asarray(data.ranges)

        # theta = np.arange(angle_min, angle_max, inc)
        theta = np.linspace(angle_min, angle_max, ranges.size)
        # Based on inspection, using a linspace is approximately good enough to get the angles
        theta = theta[ranges != 0]
        ranges = ranges[ranges != 0]
        points = np.array([np.multiply(ranges, np.cos(theta)), np.multiply(ranges, np.sin(theta))]).T
        clustering = MeanShift(bandwidth=.6).fit(points)
        all_clusters = clustering.cluster_centers_
        orientations = self.compute_orientations(points, clustering.labels_)
        poses = []
        poses_generic = []
        for i in range(np.shape(all_clusters)[0]):
            point = Pose()
            point.position.x = all_clusters[i,0]
            point.position.y = all_clusters[i,1]
            point.position.z = 0
            yaw = orientations[i]
            pitch, roll = 0, 0
            q = quaternion_from_euler(yaw, pitch, roll)
            point.orientation.w = q[0]
            point.orientation.x = q[1]
            point.orientation.y = q[2]
            point.orientation.z = q[3]
            poses.append(point)
            poses_generic.append((round(all_clusters[i,0]),
                                  round(all_clusters[i,1])))

        score = [0] * len(self.feature_list_list)
        # print self.feature_list_list
        # print poses_generic
        for i in range(len(self.feature_list_list)):
            for pose in poses_generic:
                if pose in self.feature_list_list[i]:
                    score[i] += 1.0
            score[i] = round(score[i] / len(self.feature_list_list[i]), 2)

        self.feature_list_list.append(poses_generic)

        # Generally, the highest score greater than 0.5 is considered the matching location
        # If no scores meet this category, then no match has been found
        print "Strength of matches with existing poses: ", score

        msg = PoseArray()
        msg.poses = poses
        msg.header.frame_id = 'laser_frame'
        self._corner_pub.publish(msg)

    def compute_orientations(self, data, labels):
        label_ids = Counter(labels)
        orientations = []
        for i, _ in label_ids.items():
            values = data[labels == i,:]
            max_orientation = self.get_max_hist(values)
            orientations.append(max_orientation)

        return orientations

    def get_max_hist(self, values):
        hist = [np.arctan2(values[i+1,1] - values[i-1,1], values[i+1,0] - values[i-1,0])
                for i in range(1, values.shape[0] - 1)]
        counts, bins = np.histogram(hist)
        return bins[np.argmax(counts)]



if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('mean_shift_features')
    sensor = Lidar()

    while not rospy.is_shutdown():
        pass

