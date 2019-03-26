# LIDAR
Fun with LIDAR and SLAM from scratch

## Introduction
This repository contains work done for a Master's Degree capstone project in Robotics Engineering.
The sensor used is the YDLIDAR X4. Documentation and details are 
available on the [product webpage](http://www.ydlidar.cn/product/X4).

## Dependencies
- This project was written using ROS Kinetic.
- The YDLIDAR ROS nodes are necessary to operate the sensor. They can be found at the 
[product download page](http://www.ydlidar.cn/download) in the 'ROS.zip' file. There is also 
a helpful set of instructions for installing RVIZ, which is necessary for viewing LIDAR output, in
the User Manual PDF.
- The Point Cloud Library (PCL) is used in this project. PCL can be downloaded at 
[http://www.pointclouds.org/downloads/](http://www.pointclouds.org/downloads/).
- Python dependencies include Scikit Learn for the Mean-Shift algorithm, Numpy, and the TF transformations
library. These can be installed with `pip install -r requirements.txt` in the project root.

## Setup
1. Clone this repository into your existing Catkin workspace using Git
2. Install the necessary dependencies
3. Run `catkin_make` in the root of your Catkin workspace to build the project files

## Usage
After plugging in the sensor, the following commands can be used to begin data collection, view the data,
extract features using the Mean-Shift algorithm, and match point clouds.
##### Sensor startup
In a terminal, run:
```bash
$ roslaunch ydlidar x4_view.launch
```
This will open RVIZ and display the points seen by the sensor.

##### Feature server node
The feature server node re-publishes the latest LIDAR data message whenever the user presses 'Enter'. 
This is necessary to avoid overloading the Mean-Shift algorithm with too many points, since it takes about
10 seconds to extract features each time the callback is triggered. To start the feature server, run:
```bash
$ rosrun lidar_slam feature_serv.py
```

##### Mean-Shift algorithm node
To extract Mean-Shift features from incoming point clouds, run the node:
```bash
$ rosrun lidar_slam plot_lidar.py
```
The features are published to the topic `/corners`, since Mean-Shift commonly identifies corners to be features.
To view these features, subscribe to the `/corners` topic using RVIZ. The message type is PoseArray and 
the features will be represented as 2D vectors.

##### Point cloud matching algorithm node
To match new point clouds to ones that have been seen before, run the node:
```bash
$ rosrun lidar_slam lidar_slam
```
This runs the Iterative Closest Point algorithm on an incoming point cloud to determine if it matches one 
that has been seen before. The node keeps a database of all unique point clouds that have been seen before
and returns the closest match to an incoming point cloud along with the homogeneous transformation matrix 
between the two point clouds. To view the old point cloud (the one that is the best match for the new point
cloud), subscribe to the `/pclmatch` topic in RVIZ. The message type is PointCloud2 and making the points a 
different color will help distinguish them from the real-time LIDAR input.
