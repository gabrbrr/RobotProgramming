#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include "../include/dmap_localizer.h"

class DMapLocalizerNode {
public:
    DMapLocalizerNode()  :
    X_prev(Eigen::Isometry2f::Identity()),
    odom_prev(Eigen::Isometry2f::Identity()),
    odom_current(Eigen::Isometry2f::Identity()) {
        map_sub = nh.subscribe("/map", 10, &DMapLocalizerNode::mapCallback, this);
        scan_sub = nh.subscribe("/scan", 10, &DMapLocalizerNode::scanCallback, this);
        initial_pose_sub = nh.subscribe("/initialpose", 1, &DMapLocalizerNode::initialPoseCallback, this);
        odom_sub = nh.subscribe("/odom", 10, &DMapLocalizerNode::odomCallback, this);
        has_map = false;
        transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();

    }

private:
    
    ros::NodeHandle nh;
    ros::Subscriber map_sub, scan_sub, initial_pose_sub, odom_sub;
    DMapLocalizer dmap_localizer;
    Eigen::Isometry2f X_prev, odom_prev, odom_current;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
    bool  has_map;
    std::string base_frame="base_link";
    std::string odom_frame="odom";
    std::string map_frame="map";
    ros::Time odom_stamp;
    float influence_range=1.0f;
    int occ_threshold=127;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        GridMap grid_map(msg->info.resolution, msg->info.height, msg->info.width);
        Eigen::Vector2f origin(msg->info.origin.position.x, msg->info.origin.position.y);
        grid_map.reset(origin, msg->info.resolution);

        for (size_t i = 0; i < msg->data.size(); ++i) {
            int8_t val = msg->data[i];
            grid_map.cells[i] = (val == -1) ? 127 : (val < 50) ? 255 : 0;
        }

        dmap_localizer.setMap(grid_map, influence_range, occ_threshold);
        has_map = true;
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        X_prev = convertPoseToEigen(msg->pose.pose);
        odom_prev=odom_current;
    }

    Eigen::Isometry2f convertPoseToEigen(const geometry_msgs::Pose& pose) {
        Eigen::Isometry2f iso;
        iso.translation() << pose.position.x, pose.position.y;
        iso.linear() = Eigen::Rotation2Df(tf2::getYaw(pose.orientation)).toRotationMatrix();
        return iso;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_current = convertPoseToEigen(msg->pose.pose);
        odom_stamp = msg->header.stamp; 
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (!has_map )
            return;

        std::vector<Eigen::Vector2f> measurements;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max)
                continue;
            float angle = msg->angle_min + i * msg->angle_increment;
            measurements.push_back(Eigen::Vector2f(range * cos(angle), range * sin(angle)));
        }

        Eigen::Isometry2f delta_odom = odom_prev.inverse() * odom_current;
        dmap_localizer.X = X_prev * delta_odom;
        
        if (dmap_localizer.localize(measurements, 10)) {
           //if(1){   
            X_prev = dmap_localizer.X;
            publishTransform();
            
        }



        odom_prev = odom_current;
    }
    void publishTransform() {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = odom_stamp;
        transform_msg.header.frame_id = map_frame;
        transform_msg.child_frame_id = odom_frame;

        Eigen::Isometry2f map_T_odom = X_prev * odom_current.inverse();
        transform_msg.transform.translation.x = map_T_odom.translation().x();
        transform_msg.transform.translation.y = map_T_odom.translation().y();

        tf2::Quaternion q;
        q.setRPY(0, 0, Eigen::Rotation2Df(map_T_odom.linear()).angle());
        transform_msg.transform.rotation = tf2::toMsg(q);

        transform_broadcaster->sendTransform(transform_msg);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizerNode node;
    ros::spin();
    return 0;
}