//
// Created by dpwivagg on 3/8/19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl/io/pcd_io.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>


class LaserScanDatabase {

public:
    LaserScanDatabase() {
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("pclmatch", 1000);
        pub_b = n_.advertise<sensor_msgs::PointCloud2>("scanbefore", 1000);
        pub_a = n_.advertise<sensor_msgs::PointCloud2>("scanafter", 1000);
        sub_ = n_.subscribe("rescan", 1, &LaserScanDatabase::callback, this);
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        std::cout << "Recieved new point, checking for matches in the database" << std::endl;
        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::PointCloud2 cloud_publish;
        projector_.projectLaser(*scan_in, cloud);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(cloud, *cloud_in);
        icp.setInputSource(cloud_in);

        if(saved_clouds.size() == 0) saved_clouds.push_back(cloud_in);

        for(int i = 0; i < saved_clouds.size(); i++) {
            // do something to compare the cloud with each one in the DB
            // if no match, add the cloud to the DB
            *cloud_out = *saved_clouds[i];
            pcl::toROSMsg(*cloud_out, cloud_publish);
            pub_b.publish(cloud_publish);
            icp.setInputTarget(cloud_out);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);

            std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                      icp.getFitnessScore() << std::endl;
            if(icp.getFitnessScore() < 0.15) {
                std::cout << "final match selected" << std::endl;
                pcl::toROSMsg(*cloud_out, cloud_publish);
                pub_.publish(cloud_publish);
                return;
            } // If it matches one, do nothing, end the callback
//            std::cout << icp.getFinalTransformation() << std::endl;
        }
        saved_clouds.push_back(cloud_out);
        std::cout << "Size of database: " << saved_clouds.size() << std::endl;
    }


private:
    laser_geometry::LaserProjection projector_;

    ros::NodeHandle n_;
    ros::Publisher pub_, pub_b, pub_a;
    ros::Subscriber sub_;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> saved_clouds;

};



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

    LaserScanDatabase lsdb;

    ros::Rate loop_rate(10); // 2 seconds

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}