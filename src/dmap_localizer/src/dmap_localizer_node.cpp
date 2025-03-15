#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "../include/dmap_localizer.h"
class DMapLocalizerNode {
public:
    DMapLocalizerNode() {
        map_sub = nh.subscribe("/map", 10, &DMapLocalizerNode::mapCallback, this);
        scan_sub = nh.subscribe("/scan", 10, &DMapLocalizerNode::scanCallback, this);
        initial_pose_sub = nh.subscribe("/initialpose", 1, &DMapLocalizerNode::initialPoseCallback, this);
        odom_sub = nh.subscribe("/odom", 10, &DMapLocalizerNode::odomCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub, scan_sub, initial_pose_sub, odom_sub;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        ROS_INFO("Got map!");
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {}
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {}
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizerNode node;
    ros::spin();
    return 0;
}