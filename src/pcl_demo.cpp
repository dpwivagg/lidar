//
// Created by dpwivagg on 3/8/19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"

#include <sstream>

laser_geometry::LaserProjection projector_;

void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan_in, cloud);

    chatter_pub.publish(cloud);
}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "pcl_demo");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud>("chatter", 1000);
    ros::Subscriber lidar_point_clouds = n.subscribe("scan", 1, callback);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}