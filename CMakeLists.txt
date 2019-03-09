cmake_minimum_required(VERSION 2.8.3)
project(lidar_slam)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs laser_geometry)
find_package(PCL 1.3 REQUIRED)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_slam src/pcl_demo.cpp)
target_link_libraries(lidar_slam ${catkin_LIBRARIES} ${PCL_LIBRARIES})